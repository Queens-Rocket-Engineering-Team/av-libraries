#include "aim_flight_recorder.h"
#include <logger.h>
#include <string.h>
#include <algorithm>

AimFlightRecorder::AimFlightRecorder(AimFileSystem& fs, uint8_t numCols, uint16_t originRefreshInt,
                                     uint32_t maxLogSize, const char* const* headers)
    : _fs(fs),
      _headers(headers),
      _numCols(numCols > MAX_COLUMNS ? MAX_COLUMNS : numCols),
      _originRefreshInt(originRefreshInt),
      _maxLogSize(maxLogSize),
      _rowsSinceRaw(0),
      _rdesInitialized(false),
      _disabled(false),
      _logFileOpen(false),
      _syncCounter(0),
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
  _syncCounter = 0;
  _rowsSinceRaw = 0;
  _rdesInitialized = false;  // next writeRow() emits a raw origin row
  return err == LFS_ERR_OK;
}

bool AimFlightRecorder::syncLog() {
  if (!_fs.isReady() || !_logFileOpen) return false;
  return lfs_file_sync(_fs.getLfs(), &_logFile) == LFS_ERR_OK;
}



bool AimFlightRecorder::writeRow(uint32_t rowData[]) {
  if (_disabled) return false;
  if (!_fs.isReady() || !rowData || _numCols == 0) return false;

  lfs_t* lfs = _fs.getLfs();

  if (!_logFileOpen) {
    if (lfs_file_open(lfs, &_logFile, _activeLogPath,
                      LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND) != LFS_ERR_OK) {
      return false;
    }
    _logFileOpen = true;
  }

  // Enforce storage boundary cap. If _maxLogSize is 0 (unspecified), reserve a mandatory
  // 64 KB headroom margin so LittleFS can update directory metadata, superblocks, and
  // lookahead buffers without hitting underlying NOR flash allocation errors.
  const lfs_soff_t fileSize = lfs_file_size(lfs, &_logFile);
  uint32_t effectiveMax = _maxLogSize;
  if (effectiveMax == 0) {
    effectiveMax = _fs.getTotalSize() > 65536U ? _fs.getTotalSize() - 65536U : 0;
  }
  if (fileSize > 0 && static_cast<uint32_t>(fileSize) > effectiveMax) {
    _disabled = true;
    if (_logFileOpen) {
      lfs_file_close(lfs, &_logFile); 
      _logFileOpen = false; 
    }

    LOG_ERROR("Flight recorder disabled: storage full (%u bytes)", static_cast<unsigned int>(fileSize));
    return false; 
  }

  uint8_t buffer[80];  // 5 bytes × 16 cols max
  size_t ptr = _encodeRow(buffer, rowData);

  // Write compressed row to LittleFS file.
  lfs_ssize_t written = lfs_file_write(lfs, &_logFile, buffer, ptr);
  if (written != static_cast<lfs_ssize_t>(ptr)) {
    if (_logFileOpen) {
      lfs_file_close(lfs, &_logFile);
      _logFileOpen = false;
    }
    _disabled = true;
    LOG_ERROR("Flight recorder disabled: storage full/unwritable");
    return false;
  }

  if (++_syncCounter >= 16) {
    lfs_file_sync(lfs, &_logFile);
    _syncCounter = 0;
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
