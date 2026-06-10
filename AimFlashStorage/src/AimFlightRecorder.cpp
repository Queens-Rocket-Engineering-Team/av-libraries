#include "AimFlightRecorder.h"
#include <logger.h>
#include <string.h>
#include <algorithm>

const char* AimFlightRecorder::kLogPath = "/log.bin";

AimFlightRecorder::AimFlightRecorder(AimFileSystem& fs, uint8_t numCols, uint16_t originRefreshInt, uint32_t maxLogSize)
    : _fs(fs),
      _numCols(numCols > MAX_COLUMNS ? MAX_COLUMNS : numCols),
      _originRefreshInt(originRefreshInt),
      _maxLogSize(maxLogSize),
      _rowsSinceRaw(0),
      _rdesInitialized(false),
      _logFileOpen(false),
      _syncCounter(0),
      _dumping(false),
      _dumpStream(nullptr),
      _dumpBlockSize(512),
      _dumpNumBlocks(0),
      _dumpTotalBytes(0),
      _dumpCurrentBlock(0),
      _dumpCurrentBlockOffset(0),
      _dumpLastPos(0) {
  memset(_lastVals, 0, sizeof(_lastVals));
}

AimFlightRecorder::~AimFlightRecorder() {
  stopDump();
  if (_logFileOpen) {
    lfs_file_close(_fs.getLfs(), &_logFile);
    _logFileOpen = false;
  }
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
  buf[0] = 0xE0; // Prefix 111
  buf[1] = static_cast<uint8_t>(in >> 24);
  buf[2] = static_cast<uint8_t>(in >> 16);
  buf[3] = static_cast<uint8_t>(in >> 8);
  buf[4] = static_cast<uint8_t>(in);
}

bool AimFlightRecorder::writeRow(uint32_t rowData[]) {
  if (!_fs.isReady() || !rowData || _numCols == 0) return false;

  lfs_t* lfs = _fs.getLfs();

  if (!_logFileOpen) {
    if (lfs_file_open(lfs, &_logFile, kLogPath, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND) != LFS_ERR_OK) return false;
    _logFileOpen = true;
  }

  // Rotation
  lfs_soff_t size = lfs_file_size(lfs, &_logFile);
  if (size > 0 && (uint32_t)size > _maxLogSize) {
    lfs_file_close(lfs, &_logFile);
    lfs_remove(lfs, "/log.bak");
    lfs_rename(lfs, kLogPath, "/log.bak");
    if (lfs_file_open(lfs, &_logFile, kLogPath, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND) != LFS_ERR_OK) {
      _logFileOpen = false;
      return false;
    }
    _rdesInitialized = false;
  }

  uint8_t buffer[80]; // Max 5 bytes * 16 cols = 80 bytes
  size_t ptr = 0;
  bool refreshOrigin = (_originRefreshInt > 0) && (_rowsSinceRaw >= _originRefreshInt);

  if (!_rdesInitialized || refreshOrigin) {
    for (uint8_t col = 0; col < _numCols; col++) {
      _lastVals[col] = rowData[col];
      // Use Raw-32 if bit 31 is set, otherwise Raw-31
      if (rowData[col] & 0x80000000) {
        encodeRaw32(&buffer[ptr], rowData[col]);
        ptr += 5;
      } else {
        encodeRaw31(&buffer[ptr], rowData[col]);
        ptr += 4;
      }
    }
    _rowsSinceRaw = 0;
    _rdesInitialized = true;
  } else {
    for (uint8_t col = 0; col < _numCols; col++) {
      uint32_t lastVal = _lastVals[col];
      uint32_t curVal = rowData[col];
      bool signAdd = (curVal >= lastVal);
      uint32_t offset = signAdd ? (curVal - lastVal) : (lastVal - curVal);

      if (offset <= LVL_2_MAX) {
        // Prefix 10 (2 bits), Sign (1 bit), Offset High (5 bits) = 2 bytes total
        buffer[ptr++] = 0x80 | (signAdd ? 0x20 : 0) | (static_cast<uint8_t>(offset >> 8) & 0x1F);
        buffer[ptr++] = static_cast<uint8_t>(offset);
      } else if (offset <= LVL_3_MAX) {
        // Prefix 110 (3 bits), Sign (1 bit), Offset High (4 bits) = 3 bytes total
        buffer[ptr++] = 0xC0 | (signAdd ? 0x10 : 0) | (static_cast<uint8_t>(offset >> 16) & 0x0F);
        buffer[ptr++] = static_cast<uint8_t>(offset >> 8);
        buffer[ptr++] = static_cast<uint8_t>(offset);
      } else if (curVal <= 0x7FFFFFFF) {
        // Prefix 0 (1 bit), Raw 31-bit (4 bytes total)
        encodeRaw31(&buffer[ptr], curVal);
        ptr += 4;
      } else {
        // Prefix 111 (3 bits), Raw 32-bit (5 bytes total)
        encodeRaw32(&buffer[ptr], curVal);
        ptr += 5;
      }
      _lastVals[col] = curVal;
    }
    _rowsSinceRaw++;
  }

  lfs_ssize_t written = lfs_file_write(lfs, &_logFile, buffer, ptr);
  
  // Periodic sync to reduce wear and improve performance
  if (++_syncCounter >= 16) {
    lfs_file_sync(lfs, &_logFile);
    _syncCounter = 0;
  }

  return written == (lfs_ssize_t)ptr;
}

bool AimFlightRecorder::startDump(Stream* stream) {
  if (!_fs.isReady() || _dumping || !stream) return false;
  if (_logFileOpen) syncLog();

  lfs_t* lfs = _fs.getLfs();
  if (lfs_file_open(lfs, &_dumpFile, kLogPath, LFS_O_RDONLY) != LFS_ERR_OK) return false;
  
  _dumpTotalBytes = static_cast<uint32_t>(lfs_file_size(lfs, &_dumpFile));
  _dumpBlockSize = 512;
  _dumpNumBlocks = static_cast<uint16_t>((_dumpTotalBytes + _dumpBlockSize - 1) / _dumpBlockSize);
  _dumpCurrentBlock = 0;
  _dumpCurrentBlockOffset = 0;
  _dumpLastPos = 0;
  _dumpStream = stream;
  _dumping = true;

  // MDE Handshake: 1 byte start '#' + 2 byte blockSize + 2 byte numBlocks + 4 byte totalBytes
  _dumpStream->write('#');
  _dumpStream->write(reinterpret_cast<const uint8_t*>(&_dumpBlockSize), 2);
  _dumpStream->write(reinterpret_cast<const uint8_t*>(&_dumpNumBlocks), 2);
  _dumpStream->write(reinterpret_cast<const uint8_t*>(&_dumpTotalBytes), 4);
  
  return true;
}

bool AimFlightRecorder::serviceDump(size_t maxBytes) {
  if (!_dumping || !_dumpStream) return false;

  // Process any incoming commands ('N', 'L')
  while (_dumpStream->available() > 0) {
    char cmd = _dumpStream->read();
    lfs_t* lfs = _fs.getLfs();

    if (cmd == 'N') { // Next block
      if (_dumpCurrentBlock >= _dumpNumBlocks) {
        stopDump();
        return false;
      }
      _dumpLastPos = lfs_file_tell(lfs, &_dumpFile);
      _dumpCurrentBlock++;
      _dumpCurrentBlockOffset = 0;
    } else if (cmd == 'L') { // Last block (resend)
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

  while (totalSent < maxBytes && _dumpCurrentBlockOffset < _dumpBlockSize) {
    uint8_t buf[32]; // Small chunk
    size_t toRead = std::min(sizeof(buf), maxBytes - totalSent);
    toRead = std::min(toRead, (size_t)(_dumpBlockSize - _dumpCurrentBlockOffset));
    
    lfs_ssize_t read = lfs_file_read(lfs, &_dumpFile, buf, toRead);
    if (read <= 0) break;

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
  }
}
