#ifndef AIM_FILE_SYSTEM_H
#define AIM_FILE_SYSTEM_H

#include <Arduino.h>
#include "littlefs/lfs.h"
#include <logger.h>

/**
 * @brief Interface for hardware-specific flash storage drivers.
 */
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

#include <SerialFlash.h>
class SerialFlashDriver : public AimBlockDevice {
 public:
  SerialFlashDriver(uint8_t csPin) : _csPin(csPin), _capacity(0) {}
  bool begin() override {
    if (!SerialFlash.begin(_csPin)) return false;
    uint8_t id[5]; SerialFlash.readID(id); _capacity = SerialFlash.capacity(id);
    return _capacity > 0;
  }
  int read(const struct lfs_config* c, lfs_block_t block, lfs_off_t off, void* buffer, lfs_size_t size) override {
    SerialFlash.read(block * c->block_size + off, buffer, size); return 0;
  }
  int prog(const struct lfs_config* c, lfs_block_t block, lfs_off_t off, const void* buffer, lfs_size_t size) override {
    SerialFlash.write(block * c->block_size + off, buffer, size);
    uint32_t start = millis();
    while (!SerialFlash.ready()) {
      if (millis() - start > 500) { LOG_ERROR("Flash prog timeout"); return LFS_ERR_IO; }
      yield();
    }
    return 0;
  }
  int erase(const struct lfs_config* c, lfs_block_t block) override {
    SerialFlash.eraseBlock(block * c->block_size);
    uint32_t start = millis();
    while (!SerialFlash.ready()) {
      if (millis() - start > 1000) { LOG_ERROR("Flash erase timeout"); return LFS_ERR_IO; }
      yield();
    }
    return 0;
  }
  int sync(const struct lfs_config* c) override {
    (void)c;
    uint32_t start = millis();
    while (!SerialFlash.ready()) {
      if (millis() - start > 500) { LOG_ERROR("Flash sync timeout"); return LFS_ERR_IO; }
      yield();
    }
    return 0;
  }
  lfs_size_t read_size() const override { return 1; }
  lfs_size_t prog_size() const override { return 1; }
  lfs_size_t block_size() const override { return SerialFlash.blockSize(); }
  lfs_size_t block_count() const override { return _capacity / block_size(); }
  int32_t block_cycles() const override { return 500; }
  lfs_size_t cache_size() const override { return 256; }
  lfs_size_t lookahead_size() const override { return 32; }
 private:
  uint8_t _csPin;
  uint32_t _capacity;
};

/**
 * @brief Core LittleFS filesystem manager.
 */
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
