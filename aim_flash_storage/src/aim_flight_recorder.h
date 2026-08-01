#ifndef AIM_FLIGHT_RECORDER_H
#define AIM_FLIGHT_RECORDER_H

#include <Arduino.h>
#include "aim_file_system.h"

 /**
   * @brief Supported data types and bit-widths used for telemtry optimization.
   */ 
  enum class AimDataType : uint8_t {
    UINT8 = 1, 
    INT8 = 2, 
    UINT16 = 3,
    INT16 = 4,
    UINT32 = 5, 
    INT32 = 6,
    FLOAT32 = 7
  }; 

 /**
   * @brief Definition schema for an individual telemtry column, mapping name to data type. 
   */ 
  struct AimColumnDef {
    const char* name; 
    AimDataType type; 
  };

/**
 * @brief High-speed telemetry recorder using RDES compression.
 * 
 * This class provides space-efficient sensor logging and watchdog-safe
 * asynchronous data dumping over a serial stream.
 */
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

  // [UPDATE] - headers: pointer to column schema mapping of supported data types to column names
  AimFlightRecorder(AimFileSystem& fs, uint8_t numCols, uint16_t originRefreshInt,
                    uint32_t maxLogSize, const AimColumnDef* headers = nullptr);
  ~AimFlightRecorder();

  /**
   * @brief Reclaims flash space if free space is critically low at boot.
   *
   * Must be called once after AimFileSystem::begin() and before the first
   * writeRow(). 
   * kBootMinFreeBytes are available. Safe to call on a fresh filesystem.
   * @return true if storage is usable (even if no reclamation was needed).
   */
  bool begin();

  /**
   * @brief Writes a compressed row of telemetry to the flash.
   *
   * 
   * If log file has reached capacity, the recorder latches disabled, file is closed, and 
   * all subsequent calls return false immediately. 
   * On write failure, retries exactly once. 
   * If the retry fails the recorder latches disabled
   * and all subsequent calls return false immediately. 
   * Worst-case per-call cost: bounded LFS metadata ops, << 2s watchdog.
   * @param rowData Array of unsigned 32-bit values to log.
   * @return true if written successfully.
   */
  bool writeRow(uint32_t rowData[]);

  /**
   * @brief Forces a sync of the current log file to flash.
   */
  bool syncLog();

  /**
   * @brief Syncs and closes the active log file handle.
   *
   * Must be called before AimFileSystem::format() — formatting while the
   * handle is open leaves it stale and the next writeRow() trips a littlefs
   * assert. Resets RDES state so the next writeRow() starts a fresh log with
   * a raw origin row. Safe to call when no log file is open.
   * @return true if closed cleanly (or nothing was open).
   */
  bool closeLog();

  /**
   * @brief Checks if the log file handle is currently open.
   */
  bool isLogging() const { return _logFileOpen; }

  // --- Watchdog-Safe Streaming Dump ---

  /**
   * @brief Opens the log file for reading and starts a streaming dump.
   *
   * Sends a self-describing handshake: '#' + blockSize(2) + numBlocks(2) +
   * totalBytes(4) + boardName[32] + numCols(1) + headers[numCols][32].
   * Mutes the global logger for the duration of the dump — an async LOG_*
   * line interleaved with the binary block stream corrupts it (no checksums).
   * stopDump() restores the previous log mask.
   * @param stream    The serial stream to dump to.
   * @param boardName Board identity string (null-padded to 32 bytes on wire).
   * @return true if started successfully.
   */
  bool startDump(Stream* stream, const char* boardName);

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
  AimFileSystem&     _fs;
  const AimColumnDef* _columns;   // [UPDATE] - points to column structure

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

  static const char* kLogPath;

  // Minimum free bytes required before begin() considers storage healthy.
  static constexpr uint32_t kBootMinFreeBytes = 64U * 1024U;

  // RDES implementation constants
  static constexpr uint16_t LVL_2_MAX        = 8191U;    // 2^13 - 1
  static constexpr uint32_t LVL_3_MAX        = 1048575UL; // 2^20 - 1
  static constexpr uint8_t  kRdesLvl2Prefix  = 0x80U;    // bit prefix 10x
  static constexpr uint8_t  kRdesLvl3Prefix  = 0xC0U;    // bit prefix 110x
  static constexpr uint8_t  kRdesRaw32Prefix = 0xE0U;    // bit prefix 111

  void   encodeRaw16(uint8_t* buf, uint32_t in);
  void   encodeRaw31(uint8_t* buf, uint32_t in);
  void   encodeRaw32(uint8_t* buf, uint32_t in);


  // RDES-encodes rowData into buf. Returns byte count written.
  // Updates _lastVals, _rdesInitialized, _rowsSinceRaw.
  size_t _encodeRow(uint8_t* buf, const uint32_t* rowData);
};

#endif // AIM_FLIGHT_RECORDER_H
