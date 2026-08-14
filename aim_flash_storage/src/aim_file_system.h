#ifndef AIM_FILE_SYSTEM_H
#define AIM_FILE_SYSTEM_H

#include <Arduino.h>
#include "littlefs/lfs.h"
#include <logger.h>

// Interface for hardware-specific flash storage drivers.
// LittleFS requires uniform power-of-two block/sector geometry callbacks.
class AimBlockDevice {
 public:
  virtual ~AimBlockDevice() {}
  virtual bool begin() = 0;
  virtual int read(const struct lfs_config* c, lfs_block_t block, lfs_off_t off, void* buffer, lfs_size_t size) = 0;
  virtual int prog(const struct lfs_config* c, lfs_block_t block, lfs_off_t off, const void* buffer, lfs_size_t size) = 0;
  virtual int erase(const struct lfs_config* c, lfs_block_t block) = 0;
  virtual int sync(const struct lfs_config* c) = 0;

  virtual lfs_size_t read_size() const = 0;
  virtual lfs_size_t prog_size() const = 0;
  virtual lfs_size_t block_size() const = 0;
  virtual lfs_size_t block_count() const = 0;
  virtual int32_t block_cycles() const = 0;
  virtual lfs_size_t cache_size() const = 0;
  virtual lfs_size_t lookahead_size() const = 0;
};

#ifdef ARDUINO_ARCH_ESP32
#include <esp_partition.h>
class ESP32PartitionDriver : public AimBlockDevice {
 public:
  ESP32PartitionDriver(const char* label) : _label(label), _partition(nullptr) {}
  bool begin() override {
    _partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, _label);
    if (!_partition) LOG_ERROR("Flash partition '%s' not found", _label);
    return _partition != nullptr;
  }
  int read(const struct lfs_config* c, lfs_block_t block, lfs_off_t off, void* buffer, lfs_size_t size) override {
    return (esp_partition_read(_partition, (size_t)block * c->block_size + off, buffer, size) == ESP_OK) ? 0 : LFS_ERR_IO;
  }
  int prog(const struct lfs_config* c, lfs_block_t block, lfs_off_t off, const void* buffer, lfs_size_t size) override {
    return (esp_partition_write(_partition, (size_t)block * c->block_size + off, buffer, size) == ESP_OK) ? 0 : LFS_ERR_IO;
  }
  int erase(const struct lfs_config* c, lfs_block_t block) override {
    return (esp_partition_erase_range(_partition, (size_t)block * c->block_size, c->block_size) == ESP_OK) ? 0 : LFS_ERR_IO;
  }
  int sync(const struct lfs_config* c) override { (void)c; return 0; }
  // ESP32 SPI flash controller 16-byte word alignment requirement
  lfs_size_t read_size() const override { return 16; }
  lfs_size_t prog_size() const override { return 16; }
  lfs_size_t block_size() const override { return 4096; }
  lfs_size_t block_count() const override { return _partition ? (_partition->size / 4096) : 0; }
  int32_t block_cycles() const override { return 500; }
  lfs_size_t cache_size() const override { return 256; }
  lfs_size_t lookahead_size() const override { return 32; }
 private:
  const char* _label;
  const esp_partition_t* _partition;
};
#endif

#include <SPI.h>
class SpiNorFlashDriver : public AimBlockDevice {
 public:
  // JEDEC 4KB sector erase size (opcode 0x20). LittleFS blocks map 1:1 to sectors.
  static constexpr uint32_t kSectorSize = 4096U;

  // JEDEC Page Program (opcode 0x02) limit. NOR flash chip internal buffer wraps 
  // writes at 256B page boundaries; multi-byte writes across 256B boundaries MUST be chunked.
  static constexpr uint32_t kPageSize   = 256U;

  // Maximum hardware busy timeout (Status Reg 1 WIP bit 0). Physical sector erase duration is ~400ms max. 
  // 500ms bounds SPI hardware stalls while remaining well under 2s WDT limits.
  static constexpr uint32_t kWaitMs     = 500U;

  SpiNorFlashDriver(uint8_t csPin, SPIClass& spi)
    : _cs(csPin), _spi(spi), _sectors(0) {}

  bool begin() override {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    _spi.begin();

    uint8_t id[3];
    _select(); _spi.transfer(0x9F); // Read JEDEC ID (Opcode 0x9F)
    id[0] = _spi.transfer(0); id[1] = _spi.transfer(0); id[2] = _spi.transfer(0);
    _deselect();

    if (id[0] == 0xFF || id[0] == 0x00) { LOG_ERROR("SpiNorFlash: no device"); return false; }
    if (id[2] < 16 || id[2] > 28) { LOG_ERROR("SpiNorFlash: bad capacity byte 0x%02X", id[2]); return false; }
    _sectors = (1UL << id[2]) / kSectorSize;
    LOG_INFO("SpiNorFlash: mfr=0x%02X %luMB %lu sectors",
             id[0], (1UL << id[2]) >> 20, (unsigned long)_sectors);
    return true;
  }

  int read(const struct lfs_config* c, lfs_block_t block,
           lfs_off_t off, void* buf, lfs_size_t size) override {
    (void)c;
    uint32_t addr = block * kSectorSize + off;
    _select(); _spi.transfer(0x03); _addr(addr); // Read Data (Opcode 0x03)
    uint8_t* p = static_cast<uint8_t*>(buf);
    for (lfs_size_t i = 0; i < size; ++i) p[i] = _spi.transfer(0);
    _deselect(); return 0;
  }

  int prog(const struct lfs_config* c, lfs_block_t block,
           lfs_off_t off, const void* buf, lfs_size_t size) override {
    (void)c;
    const uint8_t* p = static_cast<const uint8_t*>(buf);
    uint32_t addr = block * kSectorSize + off;
    lfs_size_t rem = size;
    while (rem > 0) {
      lfs_size_t chunk = kPageSize - (addr % kPageSize);
      if (chunk > rem) chunk = rem;
      _write_enable();
      _select(); _spi.transfer(0x02); _addr(addr); // Page Program (Opcode 0x02)
      for (lfs_size_t i = 0; i < chunk; ++i) _spi.transfer(*p++);
      _deselect();
      if (!_wait()) return LFS_ERR_IO;
      addr += chunk; rem -= chunk;
    }
    return 0;
  }

  int erase(const struct lfs_config* c, lfs_block_t block) override {
    (void)c;
    _write_enable();
    _select(); _spi.transfer(0x20); _addr(block * kSectorSize); _deselect(); // Sector Erase 4KB (Opcode 0x20)
    return _wait() ? 0 : LFS_ERR_IO;
  }

  int sync(const struct lfs_config* c) override { (void)c; return _wait() ? 0 : LFS_ERR_IO; }

  lfs_size_t read_size()      const override { return 1; }
  lfs_size_t prog_size()      const override { return 1; }
  lfs_size_t block_size()     const override { return kSectorSize; }
  lfs_size_t block_count()    const override { return _sectors; }
  int32_t    block_cycles()   const override { return 100000; }
  lfs_size_t cache_size()     const override { return kPageSize; }
  lfs_size_t lookahead_size() const override { return 32; }

 private:
  uint8_t   _cs;
  SPIClass& _spi;
  uint32_t  _sectors;

  void _select()   { _spi.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0)); digitalWrite(_cs, LOW); }
  void _deselect() { digitalWrite(_cs, HIGH); _spi.endTransaction(); }

  void _addr(uint32_t a) {
    _spi.transfer((a >> 16) & 0xFF);
    _spi.transfer((a >>  8) & 0xFF);
    _spi.transfer( a        & 0xFF);
  }

  void _write_enable() { _select(); _spi.transfer(0x06); _deselect(); } // Write Enable WREN (Opcode 0x06)

  bool _wait() {
    uint32_t start = millis();
    do {
      _select(); _spi.transfer(0x05); // Read Status Reg 1 (Opcode 0x05)
      uint8_t s = _spi.transfer(0);
      _deselect();
      if (!(s & 0x01)) return true; // Bit 0 WIP (Write In Progress) cleared
    } while ((uint32_t)(millis() - start) < kWaitMs);
    LOG_ERROR("SpiNorFlash: busy timeout"); return false;
  }
};

// Core LittleFS filesystem manager and hardware block device adapter.
// NOT THREAD-SAFE: All LittleFS operations are non-reentrant. Concurrent access 
// from multiple FreeRTOS tasks MUST be synchronized externally.
class AimFileSystem {
 public:
  explicit AimFileSystem(AimBlockDevice* device);
  ~AimFileSystem();

  bool begin();
  void end();
  bool format();
  bool isReady() const { return _mounted; }
  lfs_t* getLfs() { return &_lfs; }

  uint32_t getTotalSize() const {
    return _lfs_cfg.block_count * _lfs_cfg.block_size;
  }

  uint32_t getUsedSize();
  bool removeFile(const char* path);

 private:
  // Copies block-device geometry into _lfs_cfg; shared by begin() and format().
  void fillGeometry();
  bool _isGeometryValid() const;

  static int lfs_read(const struct lfs_config* c, lfs_block_t block, lfs_off_t off, void* buffer, lfs_size_t size);
  static int lfs_prog(const struct lfs_config* c, lfs_block_t block, lfs_off_t off, const void* buffer, lfs_size_t size);
  static int lfs_erase(const struct lfs_config* c, lfs_block_t block);
  static int lfs_sync(const struct lfs_config* c);

  AimBlockDevice* _device;
  lfs_t _lfs;
  struct lfs_config _lfs_cfg;
  bool _mounted;
};

#endif // AIM_FILE_SYSTEM_H
