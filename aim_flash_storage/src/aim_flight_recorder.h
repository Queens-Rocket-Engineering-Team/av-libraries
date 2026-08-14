#ifndef AIM_FLIGHT_RECORDER_H
#define AIM_FLIGHT_RECORDER_H

#include <Arduino.h>
#include "aim_file_system.h"

#include <rdes.h>

// High-speed telemetry recorder using RDES compression over LittleFS.
class AimFlightRecorder {
 public:
  static constexpr uint8_t MAX_COLUMNS = 16;

  // Dump wire protocol — shared contract with extract_tool/main.py.
  static constexpr char    kDumpStartChar   = '#';  // device: handshake start byte
  static constexpr char    kDumpCmdNext     = 'N';  // host: ack block / request next
  static constexpr char    kDumpCmdResend   = 'L';  // host: resend last block
  static constexpr uint8_t kHandshakeName   = 32U;  // boardName field width in handshake
  static constexpr uint8_t kHandshakeHeader = 32U;  // per-header field width in handshake
  static constexpr uint16_t kDumpBlockSize  = 512U; // dump block/chunk size on the wire

  // Headers: pointer to column schema mapping of supported data types to column names.
  AimFlightRecorder(AimFileSystem& fs, uint8_t numCols, uint16_t originRefreshInt,
                    uint32_t maxLogSize = 0, const char* const* headers = nullptr);
  ~AimFlightRecorder();

  // Initializes the flight recorder and locates active log file. Must be called after AimFileSystem::begin().
  bool begin();

  // Writes a compressed row of telemetry to flash. Bounded LFS metadata cost (<< 2s watchdog).
  bool writeRow(uint32_t rowData[]);

  // Forces an immediate sync of the current log file to flash.
  bool syncLog();

  // Syncs and closes active log file handle. Must be called before AimFileSystem::format().
  bool closeLog();

  // Checks if the log file handle is currently open.
  bool isLogging() const { return _logFileOpen; }

  // --- Watchdog-Safe Streaming Dump ---

  // Opens active log file for reading and starts serial block dump with self-describing handshake.
  bool startDump(Stream* stream, const char* boardName);

  // Opens latest log file for reading and starts streaming dump over serial.
  bool startDumpLatest(Stream* stream, const char* boardName);

  // Opens a specific log file by index number (/log_XXX.bin) and starts serial dump.
  bool startDumpIndex(Stream* stream, const char* boardName, uint16_t index);

  // Scans LittleFS and lists all flight log files and byte sizes to stream.
  uint16_t listLogs(Stream* stream);

  // Deletes all flight log files (/log_*.bin) while preserving non-log configuration files.
  uint16_t clearLogs();

  // Services a chunk of active dump. Returns true while dump is in progress.
  bool serviceDump(size_t maxBytes);

  // Aborts an active dump and closes read handle.
  void stopDump();

  // Checks if a serial dump is currently active.
  bool isDumping() const { return _dumping; }

  // Converts signed integers to unsigned bit patterns for RDES compression.
  static uint32_t unsignify(int32_t val) {
    return static_cast<uint32_t>(val);
  }

 private:
  AimFileSystem&     _fs;
  const char* const* _headers;

  uint8_t  _numCols;
  uint16_t _originRefreshInt;
  uint32_t _maxLogSize;
  uint16_t _rowsSinceRaw;
  uint32_t _lastVals[MAX_COLUMNS];
  bool     _rdesInitialized;
  bool     _disabled;

  // Logging state
  lfs_file_t _logFile;
  bool       _logFileOpen;
  uint8_t    _syncCounter;

  // Dump state
  bool        _dumping;
  Stream*     _dumpStream;
  lfs_file_t  _dumpFile;
  uint16_t    _dumpNumBlocks;
  uint32_t    _dumpTotalBytes;
  uint16_t    _dumpCurrentBlock;
  uint16_t    _dumpCurrentBlockOffset;
  lfs_soff_t  _dumpLastPos;
  uint8_t     _savedLogMask;

  char _activeLogPath[32];

  // RDES-encodes rowData into buf. Returns byte count written.
  // Updates _lastVals, _rdesInitialized, _rowsSinceRaw.
  size_t _encodeRow(uint8_t* buf, const uint32_t* rowData);

  bool startDumpFile(Stream* stream, const char* boardName, const char* path);
};

#endif // AIM_FLIGHT_RECORDER_H
