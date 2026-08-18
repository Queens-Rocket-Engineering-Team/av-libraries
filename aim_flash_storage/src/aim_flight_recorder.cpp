#include "aim_flight_recorder.h"
#include "aim_console.h"
#include <logger.h>
#include <string.h>
#include <algorithm>

AimFlightRecorder::AimFlightRecorder(AimFileSystem& fs, uint8_t numCols, uint16_t originRefreshInt,
                                     uint32_t maxLogSize, const char* const* headers,
                                     uint32_t syncIntervalMs)
    : _fs(fs),
      _headers(headers),
      _numCols(numCols > MAX_COLUMNS ? MAX_COLUMNS : numCols),
      _originRefreshInt(originRefreshInt),
      _maxLogSize(maxLogSize),
      _syncIntervalMs(syncIntervalMs),
      _rowsSinceRaw(0),
      _lastSyncMs(0),
      _rdesInitialized(false),
      _disabled(false),
      _logFileOpen(false),
      _dumping(false),
      _dumpStream(nullptr),
      _dumpNumBlocks(0),
      _dumpTotalBytes(0),
      _dumpCurrentBlock(0),
      _dumpCurrentBlockOffset(0),
      _dumpLastPos(0),
      _savedLogMask(0) {
  memset(_lastVals, 0, sizeof(_lastVals));
  _activeLogPath[0] = '\0';
}

AimFlightRecorder::~AimFlightRecorder() {
  stopDump();
  closeLog();
}

bool AimFlightRecorder::begin() {
  if (!_fs.isReady()) return false;

  lfs_t* lfs = _fs.getLfs();
  lfs_dir_t dir;
  uint16_t maxIdx = 0;

  const uint32_t t0 = millis();
  if (lfs_dir_open(lfs, &dir, "/") == LFS_ERR_OK) {
    lfs_info info;
    while (lfs_dir_read(lfs, &dir, &info) > 0) {
      if (info.type == LFS_TYPE_REG) {
        uint16_t idx = 0;
        if (sscanf(info.name, "log_%03hu.bin", &idx) == 1) {
          if (idx > maxIdx) maxIdx = idx;
        }
      }
    }
    lfs_dir_close(lfs, &dir);
  }
  const uint16_t activeLogIndex = maxIdx + 1;
  snprintf(_activeLogPath, sizeof(_activeLogPath), "/log_%03u.bin", activeLogIndex);

  const uint32_t t1 = millis();
  if (!_logFileOpen) {
    if (lfs_file_open(lfs, &_logFile, _activeLogPath,
                      LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND) != LFS_ERR_OK) {
      LOG_ERROR("AimFlightRecorder: file open failed (%s)", _activeLogPath);
      _disabled = true;
      return false;
    }
    _logFileOpen = true;
  }
  const uint32_t t2 = millis();
  LOG_INFO("FlightRecorder ready: %s (scan=%lums open=%lums sync=%lums)",
           _activeLogPath,
           static_cast<unsigned long>(t1 - t0),
           static_cast<unsigned long>(t2 - t1),
           static_cast<unsigned long>(_syncIntervalMs));

  return true;
}

size_t AimFlightRecorder::_encodeRow(uint8_t* buf, const uint32_t* rowData) {
  const bool forceOrigin = (!_rdesInitialized) || (_originRefreshInt > 0 && _rowsSinceRaw >= _originRefreshInt);
  size_t bytesWritten = rdes_encode_row_inline(buf, rowData, _lastVals, _numCols, forceOrigin);
  if (forceOrigin) {
    _rowsSinceRaw = 0;
    _rdesInitialized = true;
  } else {
    _rowsSinceRaw++;
  }
  return bytesWritten;
}


bool AimFlightRecorder::closeLog() {
  if (!_logFileOpen) return true;

  const int err = lfs_file_close(_fs.getLfs(), &_logFile);
  _logFileOpen = false;
  _lastSyncMs = 0;
  _rowsSinceRaw = 0;
  _rdesInitialized = false;  // next writeRow() emits a raw origin row
  return err == LFS_ERR_OK;
}

bool AimFlightRecorder::syncLog() {
  if (!_fs.isReady() || !_logFileOpen) return false;
  return lfs_file_sync(_fs.getLfs(), &_logFile) == LFS_ERR_OK;
}

bool AimFlightRecorder::writeRow(const uint32_t* rowData, uint32_t nowMs) {
  if (_disabled || !_logFileOpen || !rowData || _numCols == 0) return false;
#ifndef FLIGHT_BUILD
  // Silently mute writes during interactive console sessions without reporting false I/O failures
  if (aimConsoleIsActive()) return true;
#endif
  if (!_fs.isReady()) return false;

  lfs_t* lfs = _fs.getLfs();

  // Enforce per-file storage boundary cap if explicitly set
  if (_maxLogSize > 0U && static_cast<uint32_t>(lfs_file_size(lfs, &_logFile)) >= _maxLogSize) {
    _disabled = true;
    closeLog();
    LOG_WARN("Flight recorder: max log size (%u bytes) reached", static_cast<unsigned>(_maxLogSize));
    return false;
  }

  uint8_t buffer[80];  // 5 bytes × 16 cols max
  size_t ptr = _encodeRow(buffer, rowData);

  // Write compressed row to LittleFS RAM cache buffer.
  lfs_ssize_t written = lfs_file_write(lfs, &_logFile, buffer, ptr);
  if (written != static_cast<lfs_ssize_t>(ptr)) {
    closeLog();
    _disabled = true;
    LOG_ERROR("Flight recorder disabled: write error (%d)", static_cast<int>(written));
    return false;
  }

  // Bounded time-based sync
  if (_syncIntervalMs > 0U) {
    const uint32_t curMs = (nowMs != 0U) ? nowMs : millis();
    if (static_cast<uint32_t>(curMs - _lastSyncMs) >= _syncIntervalMs) {
      lfs_file_sync(lfs, &_logFile);
      _lastSyncMs = curMs;
    }
  }

  return true;
}

bool AimFlightRecorder::startDump(Stream* stream, const char* boardName) {
  return startDumpLatest(stream, boardName);
}

bool AimFlightRecorder::startDumpLatest(Stream* stream, const char* boardName) {
  lfs_t* lfs = _fs.getLfs();
  lfs_dir_t dir;
  uint16_t maxIdx = 0;
  bool found = false;

  if (lfs_dir_open(lfs, &dir, "/") == LFS_ERR_OK) {
    lfs_info info;
    while (lfs_dir_read(lfs, &dir, &info) > 0) {
      if (info.type == LFS_TYPE_REG) {
        uint16_t idx = 0;
        if (sscanf(info.name, "log_%03hu.bin", &idx) == 1) {
          if (!found || idx > maxIdx) {
            maxIdx = idx;
            found = true;
          }
        }
      }
    }
    lfs_dir_close(lfs, &dir);
  }

  char targetPath[32];
  if (found) {
    snprintf(targetPath, sizeof(targetPath), "/log_%03u.bin", maxIdx);
  } else if (_activeLogPath[0] != '\0') {
    strncpy(targetPath, _activeLogPath, sizeof(targetPath));
  } else {
    return false;
  }

  return startDumpFile(stream, boardName, targetPath);
}

bool AimFlightRecorder::startDumpIndex(Stream* stream, const char* boardName, uint16_t index) {
  char targetPath[32];
  snprintf(targetPath, sizeof(targetPath), "/log_%03u.bin", index);
  return startDumpFile(stream, boardName, targetPath);
}

uint16_t AimFlightRecorder::listLogs(Stream* stream) {
  if (!_fs.isReady() || !stream) return 0;
  if (_logFileOpen) {
    (void)syncLog();
  }
  lfs_t* lfs = _fs.getLfs();
  lfs_dir_t dir;
  uint16_t count = 0;

  if (lfs_dir_open(lfs, &dir, "/") == LFS_ERR_OK) {
    lfs_info info;
    while (lfs_dir_read(lfs, &dir, &info) > 0) {
      if (info.type == LFS_TYPE_REG) {
        uint16_t idx = 0;
        if (sscanf(info.name, "log_%03hu.bin", &idx) == 1) {
          stream->printf("  [%u] /%s (%u bytes)\r\n", idx, info.name, static_cast<unsigned>(info.size));
          count++;
        } else if (strcmp(info.name, "log.bin") == 0) {
          stream->printf("  [legacy] /%s (%u bytes)\r\n", info.name, static_cast<unsigned>(info.size));
          count++;
        }
      }
    }
    lfs_dir_close(lfs, &dir);
  }
  return count;
}

uint16_t AimFlightRecorder::clearLogs() {
  if (!_fs.isReady()) return 0;
  if (_logFileOpen) {
    (void)closeLog();
  }
  lfs_t* lfs = _fs.getLfs();
  lfs_dir_t dir;
  uint16_t deletedCount = 0;

  if (lfs_dir_open(lfs, &dir, "/") == LFS_ERR_OK) {
    lfs_info info;
    while (lfs_dir_read(lfs, &dir, &info) > 0) {
      if (info.type == LFS_TYPE_REG) {
        uint16_t idx = 0;
        if (sscanf(info.name, "log_%03hu.bin", &idx) == 1 || strcmp(info.name, "log.bin") == 0) {
          char path[LFS_NAME_MAX + 2];
          snprintf(path, sizeof(path), "/%s", info.name);
          if (lfs_remove(lfs, path) == LFS_ERR_OK) {
            deletedCount++;
          }
        }
      }
    }
    lfs_dir_close(lfs, &dir);
  }

  _rowsSinceRaw = 0;
  _rdesInitialized = false;
  _disabled = false;
  return deletedCount;
}

bool AimFlightRecorder::startDumpFile(Stream* stream, const char* boardName, const char* path) {
  if (!_fs.isReady() || _dumping || !stream || !path) return false;
  if (_logFileOpen) syncLog();

  lfs_t* lfs = _fs.getLfs();
  if (lfs_file_open(lfs, &_dumpFile, path, LFS_O_RDONLY) != LFS_ERR_OK) return false;

  _dumpTotalBytes          = static_cast<uint32_t>(lfs_file_size(lfs, &_dumpFile));
  _dumpNumBlocks           = static_cast<uint16_t>((_dumpTotalBytes + kDumpBlockSize - 1) / kDumpBlockSize);
  _dumpCurrentBlock        = 0;
  _dumpCurrentBlockOffset  = 0;
  _dumpLastPos             = 0;
  _dumpStream              = stream;
  _dumping                 = true;

  if (g_logger != nullptr) {
    _savedLogMask = g_logger->filterMask();
    g_logger->setFilterMask(0U);
  }

  char nameBuf[kHandshakeName];
  memset(nameBuf, 0, sizeof(nameBuf));
  if (boardName) { strncpy(nameBuf, boardName, sizeof(nameBuf) - 1U); }

  const uint16_t blockSize = kDumpBlockSize;
  _dumpStream->write(kDumpStartChar);
  _dumpStream->write(reinterpret_cast<const uint8_t*>(&blockSize),       2);
  _dumpStream->write(reinterpret_cast<const uint8_t*>(&_dumpNumBlocks),  2);
  _dumpStream->write(reinterpret_cast<const uint8_t*>(&_dumpTotalBytes), 4);
  _dumpStream->write(reinterpret_cast<const uint8_t*>(nameBuf), kHandshakeName);
  _dumpStream->write(_numCols);

  char hdrBuf[kHandshakeHeader];
  for (uint8_t i = 0U; i < _numCols; ++i) {
    memset(hdrBuf, 0, sizeof(hdrBuf));
    if (_headers && _headers[i]) {
      strncpy(hdrBuf, _headers[i], sizeof(hdrBuf) - 1U);
    }
    _dumpStream->write(reinterpret_cast<const uint8_t*>(hdrBuf), kHandshakeHeader);
  }

  return true;
}

bool AimFlightRecorder::serviceDump(size_t maxBytes) {
  if (!_dumping || !_dumpStream) return false;

  // Process any incoming commands ('N', 'L')
  while (_dumpStream->available() > 0) {
    char cmd = _dumpStream->read();
    lfs_t* lfs = _fs.getLfs();

    if (cmd == kDumpCmdNext) { // Next block
      if (_dumpCurrentBlock >= _dumpNumBlocks) {
        stopDump();
        return false;
      }
      _dumpLastPos = lfs_file_tell(lfs, &_dumpFile);
      _dumpCurrentBlock++;
      _dumpCurrentBlockOffset = 0;
    } else if (cmd == kDumpCmdResend) { // Last block (resend)
      lfs_file_seek(lfs, &_dumpFile, _dumpLastPos, LFS_SEEK_SET);
      _dumpCurrentBlockOffset = 0;
    }
  }

  // If we are between blocks, just wait
  if (_dumpCurrentBlock == 0 && _dumpTotalBytes > 0) return true; 
  
  if (_dumpCurrentBlock > _dumpNumBlocks) {
    stopDump();
    return false;
  }

  // Send data in small chunks to respect maxBytes and avoid blocking serial
  size_t totalSent = 0;
  lfs_t* lfs = _fs.getLfs();

  while (totalSent < maxBytes && _dumpCurrentBlockOffset < kDumpBlockSize) {
    uint8_t buf[32]; // Small chunk
    size_t toRead = std::min(sizeof(buf), maxBytes - totalSent);
    toRead = std::min(toRead, (size_t)(kDumpBlockSize - _dumpCurrentBlockOffset));
    
    lfs_ssize_t read = lfs_file_read(lfs, &_dumpFile, buf, toRead);
    if (read < 0) break;
    if (read == 0) {
      // EOF inside the final block: pad to the fixed block size. The host
      // reads exactly blockSize per block and trims to totalBytes at the end;
      // a short final block leaves it waiting (resend-retry) forever.
      memset(buf, 0xFF, toRead);
      read = static_cast<lfs_ssize_t>(toRead);
    }

    _dumpStream->write(buf, read);
    totalSent += read;
    _dumpCurrentBlockOffset += read;
  }

  return true;
}

void AimFlightRecorder::stopDump() {
  if (_dumping) {
    lfs_file_close(_fs.getLfs(), &_dumpFile);
    _dumping = false;
    _dumpStream = nullptr;
    if (g_logger != nullptr) {
      g_logger->setFilterMask(_savedLogMask);
    }
  }
}
