#ifndef AIM_FLIGHT_RECORDER_H
#define AIM_FLIGHT_RECORDER_H

#include <Arduino.h>
#include "AimFileSystem.h"

/**
 * @brief High-speed telemetry recorder using RDES compression.
 * 
 * This class provides space-efficient sensor logging and watchdog-safe
 * asynchronous data dumping over a serial stream.
 */
class AimFlightRecorder {
 public:
  static constexpr uint8_t MAX_COLUMNS = 16;

  AimFlightRecorder(AimFileSystem& fs, uint8_t numCols, uint16_t originRefreshInt, uint32_t maxLogSize);
  ~AimFlightRecorder();

  /**
   * @brief Writes a compressed row of telemetry to the flash.
   * @param rowData Array of unsigned 32-bit values to log.
   * @return true if written successfully.
   */
  bool writeRow(uint32_t rowData[]);

  /**
   * @brief Forces a sync of the current log file to flash.
   */
  bool syncLog();

  // --- Watchdog-Safe Streaming Dump ---

  /**
   * @brief Opens the log file for reading and starts a streaming dump.
   * @param stream The serial stream to dump hex data to.
   * @return true if started successfully.
   */
  bool startDump(Stream* stream);

  /**
   * @brief Services a chunk of the active dump. Must be called periodically.
   * @param maxBytes Maximum number of raw bytes to process in this tick.
   * @return true if the dump is still in progress, false if finished or failed.
   */
  bool serviceDump(size_t maxBytes);

  /**
   * @brief Aborts an active dump and closes the read handle.
   */
  void stopDump();

  /**
   * @brief Checks if a dump is currently active.
   */
  bool isDumping() const { return _dumping; }

  /**
   * @brief Helper to convert signed values (like RSSI) to unsigned for RDES.
   */
  static uint32_t unsignify(int32_t val) {
    return static_cast<uint32_t>(val);
  }

 private:
  AimFileSystem& _fs;
  
  uint8_t _numCols;
  uint16_t _originRefreshInt;
  uint32_t _maxLogSize;
  uint16_t _rowsSinceRaw;
  uint32_t _lastVals[MAX_COLUMNS];
  bool _rdesInitialized;

  // Logging state
  lfs_file_t _logFile;
  bool _logFileOpen;
  uint8_t _syncCounter;

  // Dump state
  bool _dumping;
  Stream* _dumpStream;
  lfs_file_t _dumpFile;
  uint16_t _dumpBlockSize;
  uint16_t _dumpNumBlocks;
  uint32_t _dumpTotalBytes;
  uint16_t _dumpCurrentBlock;
  uint16_t _dumpCurrentBlockOffset;
  lfs_soff_t _dumpLastPos;

  static const char* kLogPath;

  // RDES implementation constants
  static constexpr uint16_t LVL_2_MAX = 8191U;       // 2^13 - 1
  static constexpr uint32_t LVL_3_MAX = 1048575UL;   // 2^20 - 1

  void encodeRaw31(uint8_t* buf, uint32_t val);
  void encodeRaw32(uint8_t* buf, uint32_t val);
};

#endif // AIM_FLIGHT_RECORDER_H
