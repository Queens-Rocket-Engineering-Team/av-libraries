#ifndef FLASHTABLE_H
#define FLASHTABLE_H

#include <Arduino.h>
#include <SPI.h>

// https://github.com/PaulStoffregen/SerialFlash
#include <SerialFlash.h>

class FlashTable {
 public:
  FlashTable(uint8_t numCols,
             uint16_t originRefreshInt,
             uint32_t maxSize,
             uint8_t tableNum,
             uint16_t buffSize);
  ~FlashTable();

  // Must be called before using any file-backed operations.
  void init(SerialFlashChip* serialFlash, Stream* stream);

  bool writeRow(uint32_t rowData[]);
  uint32_t unsignify(int32_t value);
  uint32_t getCurSize();
  uint32_t getMaxSize();

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

  bool writeByte(uint8_t in);
  bool writeUint32(uint32_t in);
  void flushPendingBuffer();
  void seekToEmpty(Stream* stream);

  SerialFlashFile _file;
  uint32_t _maxSize;
  uint8_t _tableNum;
  uint16_t _buffSize;

  uint8_t _numCols;
  uint16_t _originRefreshInt;
  uint16_t _rowsSinceRaw;
  bool _initialized;
  uint32_t* _lastValsPtr;

  uint8_t* _bufferPtr;
  uint16_t _bufPos;
};

#endif