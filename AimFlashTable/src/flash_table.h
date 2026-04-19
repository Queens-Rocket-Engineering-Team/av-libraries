#ifndef FLASHTABLE_H
#define FLASHTABLE_H

#include <Arduino.h>
#include <SPI.h>

// https://github.com/PaulStoffregen/SerialFlash
#include <SerialFlash.h>

enum FlashTableState : uint8_t {
  FLASHTABLE_STATE_IDLE = 0U,
  FLASHTABLE_STATE_DUMP = 1U,
  FLASHTABLE_STATE_ERASE = 2U
};

enum FlashTableServiceResult : uint8_t {
  FLASHTABLE_SERVICE_IDLE = 0U,
  FLASHTABLE_SERVICE_ACTIVE = 1U,
  FLASHTABLE_SERVICE_DONE = 2U,
  FLASHTABLE_SERVICE_ABORTED = 3U,
  FLASHTABLE_SERVICE_ERROR = 4U
};

class FlashTable {
 public:
  FlashTable(SerialFlashChip* serialFlash,
             uint8_t numCols,
             uint16_t originRefreshInt,
             uint32_t maxSize,
             uint8_t tableNum,
             uint16_t buffSize,
             uint32_t* lastValsBuffer,
             uint8_t* ioBuffer);
  ~FlashTable();

  // Must be called after SerialFlash.begin(...) succeeds.
  void init(Stream* stream);

  bool writeRow(uint32_t rowData[]);
  uint32_t unsignify(int32_t value);
  uint32_t getCurSize();
  uint32_t getWritePosition();
  uint32_t getMaxSize();
  uint32_t readAt(uint32_t offset, uint8_t* out, uint32_t len);

  FlashTableState state(void);
  bool isReady(void);
  bool isBusy(void);
  bool commandInfo(Stream* stream);
  bool commandDump(Stream* stream, uint32_t maxBytes, uint32_t* usedBytes, uint32_t* dumpBytes);
  bool commandErase(Stream* stream);
  bool beginDump(uint32_t maxBytes, uint32_t* usedBytes, uint32_t* dumpBytes);
  void cancelDump(void);
  FlashTableServiceResult serviceDump(Stream* stream, uint16_t lineBytes);
  bool beginErase(void);
  FlashTableServiceResult serviceErase(void);

  // Data range is [strtPos, endPos), i.e. end position is exclusive.
  void beginDataDump(Stream* stream, uint32_t strtPos, uint32_t endPos);
  void beginDataDump(Stream* stream);
  void beginDataDump();

 private:
  static constexpr uint16_t LVL_2_MAX = 8191U;         // 2^13 - 1
  static constexpr uint32_t LVL_3_MAX = 2097151UL;     // 2^21 - 1
  static constexpr uint8_t kEmptyValue = 0xFFU;
  static constexpr uint8_t kNumEmptyTrigger = 16U;
  static constexpr uint32_t kCoarseSeekStep = 512U;
  static constexpr uint16_t kDumpBlockSize = 512U;
  static constexpr uint16_t kServiceDumpMaxLineBytes = 64U;

  bool writeByte(uint8_t in);
  bool writeUint32(uint32_t in);
  void flushPendingBuffer();
  void seekToEmpty(Stream* stream);
  void writeHexByte(Stream* stream, uint8_t value);

  SerialFlashChip* _flash;
  SerialFlashFile _file;
  uint32_t _maxSize;
  uint8_t _tableNum;
  uint16_t _buffSize;

  uint8_t _numCols;
  uint16_t _originRefreshInt;
  uint16_t _rowsSinceRaw;
  bool _initialized;
  uint32_t* _lastValsPtr;

  FlashTableState _state;
  uint32_t _dumpLimitBytes;
  uint32_t _dumpOffsetBytes;

  uint8_t* _bufferPtr;
  uint16_t _bufPos;
};

#endif