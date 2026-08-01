#include "aim_flight_recorder.h"
#include <logger.h>
#include <string.h>
#include <algorithm>

const char* AimFlightRecorder::kLogPath = "/log.bin";

AimFlightRecorder::AimFlightRecorder(AimFileSystem& fs, uint8_t numCols, uint16_t originRefreshInt,
                                     uint32_t maxLogSize, const AimColumnDef* columns)
    : _fs(fs),
      _columns(columns),
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
}

AimFlightRecorder::~AimFlightRecorder() {
  stopDump();
  closeLog();
}

bool AimFlightRecorder::begin() {
  if (!_fs.isReady()) return false;

  const uint32_t total = _fs.getTotalSize();
  const uint32_t used  = _fs.getUsedSize();
  const uint32_t freeBytes = (used >= total) ? 0U : (total - used);

  if (freeBytes < kBootMinFreeBytes) {
    const uint32_t prevFree = freeBytes;
    lfs_t* lfs = _fs.getLfs();
    const uint32_t used2  = _fs.getUsedSize();
    const uint32_t free2  = (used2 >= total) ? 0U : (total - used2);
    if (free2 < kBootMinFreeBytes) {
      lfs_remove(lfs, kLogPath);
    }
    LOG_WARN("Flight recorder: reclaimed space at boot (was %uB free)", prevFree);
  }
  return true;
}

size_t AimFlightRecorder::_encodeRow(uint8_t* buf, const uint32_t* rowData) {
  size_t ptr = 0;
  const bool refreshOrigin = (_originRefreshInt > 0) && (_rowsSinceRaw >= _originRefreshInt);

  if (!_rdesInitialized || refreshOrigin) {
    for (uint8_t col = 0; col < _numCols; col++) {
      _lastVals[col] = rowData[col];
      if (rowData[col] & 0x80000000U) {
        encodeRaw32(&buf[ptr], rowData[col]);
        ptr += 5;
      } else {
        encodeRaw31(&buf[ptr], rowData[col]);
        ptr += 4;
      }
    }
    _rowsSinceRaw    = 0;
    _rdesInitialized = true;
  } else {
    for (uint8_t col = 0; col < _numCols; col++) {
      const uint32_t lastVal = _lastVals[col];
      const uint32_t curVal  = rowData[col];
      const bool signAdd     = (curVal >= lastVal);
      const uint32_t offset  = signAdd ? (curVal - lastVal) : (lastVal - curVal);

      if (offset <= LVL_2_MAX) {
        buf[ptr++] = kRdesLvl2Prefix | (signAdd ? 0x20U : 0U) | (static_cast<uint8_t>(offset >> 8) & 0x1FU);
        buf[ptr++] = static_cast<uint8_t>(offset);
      } else if (offset <= LVL_3_MAX) {
        buf[ptr++] = kRdesLvl3Prefix | (signAdd ? 0x10U : 0U) | (static_cast<uint8_t>(offset >> 16) & 0x0FU);
        buf[ptr++] = static_cast<uint8_t>(offset >> 8);
        buf[ptr++] = static_cast<uint8_t>(offset);
      } else if (curVal <= 0x7FFFFFFFU) {
        encodeRaw31(&buf[ptr], curVal);
        ptr += 4;
      } else {
        encodeRaw32(&buf[ptr], curVal);
        ptr += 5;
      }
      _lastVals[col] = curVal;
    }
    _rowsSinceRaw++;
  }
  return ptr;
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

void AimFlightRecorder::encodeRaw31(uint8_t* buf, uint32_t in) {
  buf[0] = static_cast<uint8_t>(0b01111111U & (in >> 24));
  buf[1] = static_cast<uint8_t>(in >> 16);
  buf[2] = static_cast<uint8_t>(in >> 8);
  buf[3] = static_cast<uint8_t>(in);
}

void AimFlightRecorder::encodeRaw32(uint8_t* buf, uint32_t in) {
  buf[0] = kRdesRaw32Prefix;
  buf[1] = static_cast<uint8_t>(in >> 24);
  buf[2] = static_cast<uint8_t>(in >> 16);
  buf[3] = static_cast<uint8_t>(in >> 8);
  buf[4] = static_cast<uint8_t>(in);
}

bool AimFlightRecorder::writeRow(uint32_t rowData[]) {
  if (_disabled) return false;
  if (!_fs.isReady() || !rowData || _numCols == 0) return false;

  lfs_t* lfs = _fs.getLfs();

  if (!_logFileOpen) {
    if (lfs_file_open(lfs, &_logFile, kLogPath,
                      LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND) != LFS_ERR_OK) {
      return false;
    }
    _logFileOpen = true;
  }

  // update: Size-triggered disable & log error
  const lfs_soff_t fileSize = lfs_file_size(lfs, &_logFile);
  if (fileSize > 0 && static_cast<uint32_t>(fileSize) > _maxLogSize) {
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

  // Write with NOSPC recovery: one retry.
  // Worst-case per-call cost: bounded LFS metadata ops, << 2s watchdog.
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
  if (!_fs.isReady() || _dumping || !stream) return false;
  if (_logFileOpen) syncLog();

  lfs_t* lfs = _fs.getLfs();
  if (lfs_file_open(lfs, &_dumpFile, kLogPath, LFS_O_RDONLY) != LFS_ERR_OK) return false;

  _dumpTotalBytes          = static_cast<uint32_t>(lfs_file_size(lfs, &_dumpFile));
  _dumpNumBlocks           = static_cast<uint16_t>((_dumpTotalBytes + kDumpBlockSize - 1) / kDumpBlockSize);
  _dumpCurrentBlock        = 0;
  _dumpCurrentBlockOffset  = 0;
  _dumpLastPos             = 0;
  _dumpStream              = stream;
  _dumping                 = true;

  // Mute async logging before the handshake — a LOG_* line interleaved with
  // the binary block stream corrupts it. Restored in stopDump().
  if (g_logger != nullptr) {
    _savedLogMask = g_logger->filterMask();
    g_logger->setFilterMask(0U);
  }

  // Self-describing handshake:
  //   '#' + blockSize(2LE) + numBlocks(2LE) + totalBytes(4LE)
  //   + boardName[32] + numCols(1) + (columnName[32] + dataType[1]) * numCols
  char nameBuf[kHandshakeName];
  memset(nameBuf, 0, sizeof(nameBuf));
  if (boardName) { strncpy(nameBuf, boardName, sizeof(nameBuf) - 1U); }

  const uint16_t blockSize = kDumpBlockSize;  // addressable copy for the LE wire write
  _dumpStream->write(kDumpStartChar);
  _dumpStream->write(reinterpret_cast<const uint8_t*>(&blockSize),       2);
  _dumpStream->write(reinterpret_cast<const uint8_t*>(&_dumpNumBlocks),  2);
  _dumpStream->write(reinterpret_cast<const uint8_t*>(&_dumpTotalBytes), 4);
  _dumpStream->write(reinterpret_cast<const uint8_t*>(nameBuf), kHandshakeName);
  _dumpStream->write(_numCols);

  char hdrBuf[kHandshakeHeader];
  for (uint8_t i = 0U; i < _numCols; ++i) {
    memset(hdrBuf, 0, sizeof(hdrBuf));
    if (_columns && _columns[i].name) {
      strncpy(hdrBuf, _columns[i].name, sizeof(hdrBuf) - 1U);
    }
    _dumpStream->write(reinterpret_cast<const uint8_t*>(hdrBuf), kHandshakeHeader); // write 32-byte col name 

    // write 1-byte data type enum specifier 
    uint8_t typeByte = _columns ? static_cast<uint8_t>(_columns[i].type) : static_cast<uint8_t>(AimDataType::UINT32); 
    _dumpStream->write(&typeByte, 1); 
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
