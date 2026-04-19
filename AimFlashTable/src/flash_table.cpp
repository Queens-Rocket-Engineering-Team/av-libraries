#include "flash_table.h"

namespace {
constexpr uint32_t kDumpStreamWaitMaxIters = 100000U;
constexpr uint32_t kDumpStreamDrainMaxBytes = 1024U;
}

// NOTE: `lastValsBuffer` and `ioBuffer` are caller-managed storage.
// FlashTable does not allocate, resize, or free either buffer.
// - `lastValsBuffer` must be non-null when `numCols > 0`.
// - `ioBuffer` must be non-null when `buffSize > 0`.
// Both buffers must remain valid for the entire lifetime of the
// FlashTable instance because their pointers are retained internally.
FlashTable::FlashTable(SerialFlashChip* serialFlash,
                       uint8_t numCols,
                       uint16_t originRefreshInt,
                       uint32_t maxSize,
                       uint8_t tableNum,
                       uint16_t buffSize,
                       uint32_t* lastValsBuffer,
                       uint8_t* ioBuffer)
    : _flash(serialFlash),
      _maxSize(maxSize),
      _tableNum(tableNum),
      _buffSize(buffSize),
      _numCols(numCols),
      _originRefreshInt(originRefreshInt),
      _rowsSinceRaw(0U),
      _initialized(false),
      _lastValsPtr(lastValsBuffer),
      _state(FLASHTABLE_STATE_IDLE),
      _dumpLimitBytes(0U),
      _dumpOffsetBytes(0U),
      _bufferPtr(ioBuffer),
      _bufPos(0U) {
  const bool missingLastVals = (_numCols > 0U) && (_lastValsPtr == nullptr);
  const bool missingIoBuffer = (_buffSize > 0U) && (_bufferPtr == nullptr);
  if (missingLastVals || missingIoBuffer) {
    // Invalid construction parameters: keep instance in a not-ready state.
    _flash = nullptr;
    _numCols = 0U;
    _buffSize = 0U;
  }
}

FlashTable::~FlashTable() {
  flushPendingBuffer();
}

void FlashTable::init(Stream* stream) {
  (void)stream;
  if (_flash == nullptr) {
    _initialized = false;
    _state = FLASHTABLE_STATE_IDLE;
    _dumpLimitBytes = 0U;
    _dumpOffsetBytes = 0U;
    return;
  }

  char fName[8];
  // Preserve legacy behavior: table number is rendered in base-8.
  itoa(_tableNum, fName, 8);

  bool findEmptySpot = true;
  if (!_flash->exists(fName)) {
    _flash->create(fName, _maxSize);
    findEmptySpot = false;
  }

  _file = _flash->open(fName);
  if (!_file) {
    _state = FLASHTABLE_STATE_IDLE;
    _dumpLimitBytes = 0U;
    _dumpOffsetBytes = 0U;
    return;
  }

  _bufPos = 0U;
  _rowsSinceRaw = 0U;
  _initialized = false;
  _state = FLASHTABLE_STATE_IDLE;
  _dumpLimitBytes = 0U;
  _dumpOffsetBytes = 0U;

  if (findEmptySpot) {
    seekToEmpty(stream);
  }
}

uint32_t FlashTable::unsignify(int32_t value) {
  // apply offset of ((2^30)-1)//2
  return static_cast<uint32_t>(value + 536870911U);
}

uint32_t FlashTable::getCurSize() {
  flushPendingBuffer();
  // Preserve legacy +1 behavior for compatibility.
  return _file.position() + 1U;
}

uint32_t FlashTable::getWritePosition() {
  flushPendingBuffer();
  return _file.position();
}

uint32_t FlashTable::getMaxSize() {
  return _file.size();
}

uint32_t FlashTable::readAt(uint32_t offset, uint8_t* out, uint32_t len) {
  if ((out == nullptr) || (len == 0U)) {
    return 0U;
  }

  flushPendingBuffer();

  const uint32_t fileSize = _file.size();
  if (offset >= fileSize) {
    return 0U;
  }

  uint32_t readLen = len;
  const uint32_t maxReadable = fileSize - offset;
  if (readLen > maxReadable) {
    readLen = maxReadable;
  }

  const uint32_t origPoint = _file.position();
  _file.seek(offset);
  const uint32_t bytesRead = _file.read(out, readLen);
  _file.seek(origPoint);
  return bytesRead;
}

FlashTableState FlashTable::state(void) {
  return _state;
}

bool FlashTable::isReady(void) {
  return (_flash != nullptr) && static_cast<bool>(_file);
}

bool FlashTable::isBusy(void) {
  return _state != FLASHTABLE_STATE_IDLE;
}

bool FlashTable::commandInfo(Stream* stream) {
  if (stream == nullptr) {
    return false;
  }

  if (!isReady()) {
    stream->println("flash not ready");
    return false;
  }

  if (isBusy()) {
    stream->println("flash busy");
    return false;
  }

  uint8_t id[5] = {0U, 0U, 0U, 0U, 0U};
  SerialFlash.readID(id);
  const uint32_t flashCapacityBytes = SerialFlash.capacity(id);

  stream->print("jedec=");
  writeHexByte(stream, id[0]);
  stream->print(' ');
  writeHexByte(stream, id[1]);
  stream->print(' ');
  writeHexByte(stream, id[2]);
  stream->print(" cap=");
  stream->print(static_cast<unsigned long>(flashCapacityBytes));
  stream->print(" block=");
  stream->println(static_cast<unsigned long>(SerialFlash.blockSize()));

  stream->print("flash ");
  stream->print(getMaxSize());
  stream->print(",");
  stream->println(getCurSize());
  return true;
}

bool FlashTable::commandDump(Stream* stream, uint32_t maxBytes, uint32_t* usedBytes, uint32_t* dumpBytes) {
  if (usedBytes != nullptr) {
    *usedBytes = 0U;
  }
  if (dumpBytes != nullptr) {
    *dumpBytes = 0U;
  }

  if (stream == nullptr) {
    return false;
  }

  if (!isReady()) {
    stream->println("flash not ready");
    return false;
  }

  if (isBusy()) {
    stream->println("flash busy");
    return false;
  }

  uint32_t used = 0U;
  uint32_t dump = 0U;
  const bool dumpStarted = beginDump(maxBytes, &used, &dump);
  if (usedBytes != nullptr) {
    *usedBytes = used;
  }
  if (dumpBytes != nullptr) {
    *dumpBytes = dump;
  }

  if (!dumpStarted) {
    if (used == 0U) {
      stream->println("flash dump empty");
    } else {
      stream->println("flash dump failed");
    }
    return false;
  }

  stream->print("flash dump start bytes=");
  stream->print(static_cast<unsigned long>(dump));
  stream->print("/");
  stream->println(static_cast<unsigned long>(used));
  if (dump < used) {
    stream->println("flash dump truncated");
  }

  return true;
}

bool FlashTable::commandErase(Stream* stream) {
  if (stream == nullptr) {
    return false;
  }

  if (!isReady()) {
    stream->println("flash not ready");
    return false;
  }

  if (isBusy()) {
    stream->println("flash busy");
    return false;
  }

  if (!beginErase()) {
    stream->println("flash erase failed");
    return false;
  }

  stream->println("flash erase...");
  return true;
}

bool FlashTable::beginDump(uint32_t maxBytes, uint32_t* usedBytes, uint32_t* dumpBytes) {
  if (usedBytes != nullptr) {
    *usedBytes = 0U;
  }
  if (dumpBytes != nullptr) {
    *dumpBytes = 0U;
  }

  if (_state != FLASHTABLE_STATE_IDLE) {
    return false;
  }
  if (!isReady()) {
    return false;
  }

  uint32_t used = getCurSize();
  const uint32_t fileSize = getMaxSize();
  if (used > fileSize) {
    used = fileSize;
  }

  uint32_t dump = used;
  if ((maxBytes > 0U) && (dump > maxBytes)) {
    dump = maxBytes;
  }

  if (usedBytes != nullptr) {
    *usedBytes = used;
  }
  if (dumpBytes != nullptr) {
    *dumpBytes = dump;
  }

  if (dump == 0U) {
    return false;
  }

  _dumpLimitBytes = dump;
  _dumpOffsetBytes = 0U;
  _state = FLASHTABLE_STATE_DUMP;
  return true;
}

void FlashTable::cancelDump(void) {
  if (_state != FLASHTABLE_STATE_DUMP) {
    return;
  }

  _state = FLASHTABLE_STATE_IDLE;
  _dumpLimitBytes = 0U;
  _dumpOffsetBytes = 0U;
}

void FlashTable::writeHexByte(Stream* stream, uint8_t value) {
  static const char kHexDigits[] = "0123456789ABCDEF";
  stream->write(kHexDigits[(value >> 4) & 0x0FU]);
  stream->write(kHexDigits[value & 0x0FU]);
}

FlashTableServiceResult FlashTable::serviceDump(Stream* stream, uint16_t lineBytes) {
  if (_state != FLASHTABLE_STATE_DUMP) {
    return FLASHTABLE_SERVICE_IDLE;
  }

  if ((stream == nullptr) || (lineBytes == 0U)) {
    cancelDump();
    return FLASHTABLE_SERVICE_ERROR;
  }

  if (_dumpOffsetBytes >= _dumpLimitBytes) {
    _state = FLASHTABLE_STATE_IDLE;
    _dumpLimitBytes = 0U;
    _dumpOffsetBytes = 0U;
    return FLASHTABLE_SERVICE_DONE;
  }

  uint8_t byteBuff[kServiceDumpMaxLineBytes] = {0U};
  uint32_t remBytes = _dumpLimitBytes - _dumpOffsetBytes;
  uint32_t chunkBytes = remBytes;

  uint16_t maxLineBytes = lineBytes;
  if (maxLineBytes > kServiceDumpMaxLineBytes) {
    maxLineBytes = kServiceDumpMaxLineBytes;
  }
  if (chunkBytes > maxLineBytes) {
    chunkBytes = maxLineBytes;
  }

  const uint32_t readBytes = readAt(_dumpOffsetBytes, byteBuff, chunkBytes);
  if (readBytes == 0U) {
    cancelDump();
    return FLASHTABLE_SERVICE_ABORTED;
  }

  stream->print("@");
  stream->print(static_cast<unsigned long>(_dumpOffsetBytes));
  stream->print(": ");
  for (uint32_t i = 0U; i < readBytes; i++) {
    writeHexByte(stream, byteBuff[i]);
    if ((i + 1U) < readBytes) {
      stream->print(' ');
    }
  }
  stream->println();

  _dumpOffsetBytes += readBytes;
  if (_dumpOffsetBytes >= _dumpLimitBytes) {
    _state = FLASHTABLE_STATE_IDLE;
    _dumpLimitBytes = 0U;
    _dumpOffsetBytes = 0U;
    return FLASHTABLE_SERVICE_DONE;
  }

  return FLASHTABLE_SERVICE_ACTIVE;
}

bool FlashTable::beginErase(void) {
  if ((!isReady()) || (_state != FLASHTABLE_STATE_IDLE)) {
    return false;
  }

  _flash->eraseAll();
  _state = FLASHTABLE_STATE_ERASE;
  return true;
}

FlashTableServiceResult FlashTable::serviceErase(void) {
  if (_state != FLASHTABLE_STATE_ERASE) {
    return FLASHTABLE_SERVICE_IDLE;
  }

  if (_flash == nullptr) {
    _state = FLASHTABLE_STATE_IDLE;
    return FLASHTABLE_SERVICE_ERROR;
  }

  if (!_flash->ready()) {
    return FLASHTABLE_SERVICE_ACTIVE;
  }

  init(nullptr);
  _state = FLASHTABLE_STATE_IDLE;
  return FLASHTABLE_SERVICE_DONE;
}

void FlashTable::seekToEmpty(Stream* stream) {
  (void)stream;

  const uint32_t fileSize = _file.size();
  if (fileSize == 0U) {
    return;
  }

  uint8_t numEmpties = 0U;
  bool courseDone = false;

  _file.seek(0U);

  uint8_t serBuffer[1] = {0U};
  while ((_file.position() < fileSize) && !courseDone) {
    _file.read(serBuffer, 1);
    if (serBuffer[0] == kEmptyValue) {
      numEmpties++;

      while ((numEmpties < kNumEmptyTrigger) && (_file.position() < fileSize)) {
        _file.read(serBuffer, 1);
        if (serBuffer[0] == kEmptyValue) {
          numEmpties++;
        } else {
          numEmpties = 0U;
          break;
        }
      }

      if (numEmpties >= kNumEmptyTrigger) {
        _file.seek(_file.position() - kNumEmptyTrigger);
        if (_file.position() == 0U) {
          return;
        }

        while (_file.position() > 0U) {
          _file.read(serBuffer, 1);
          if (serBuffer[0] != kEmptyValue) {
            _file.seek(_file.position() + kNumEmptyTrigger - 1U);
            courseDone = true;
            break;
          }

          if (_file.position() == 0U) {
            return;
          }

          _file.seek(_file.position() - 2U);
        }
      }
    } else {
      numEmpties = 0U;
      uint32_t nextPos = _file.position() + kCoarseSeekStep;
      if (nextPos >= fileSize) {
        nextPos = fileSize - 1U;
      }
      _file.seek(nextPos);
    }
  }

  if (_file.position() > 0U) {
    _file.seek(_file.position() - 1U);
  } else {
    _file.seek(0U);
  }

  if (_file.position() >= fileSize) {
    _file.seek(fileSize - 1U);
  }
}

void FlashTable::beginDataDump(Stream* stream, uint32_t strtPos, uint32_t endPos) {
  if (stream == nullptr) {
    return;
  }

  flushPendingBuffer();

  const uint32_t fileSize = _file.size();
  if (strtPos > fileSize) {
    strtPos = fileSize;
  }
  if (endPos > fileSize) {
    endPos = fileSize;
  }
  if (endPos < strtPos) {
    const uint32_t temp = strtPos;
    strtPos = endPos;
    endPos = temp;
  }

  const uint32_t numBytes = endPos - strtPos;
  const uint32_t numBlocks = (numBytes / kDumpBlockSize) + ((numBytes % kDumpBlockSize) != 0U);

  uint32_t curBlock = 0U;
  const uint32_t origPoint = _file.position();
  _file.seek(strtPos);

  for (uint32_t i = 0U; i < kDumpStreamDrainMaxBytes; i++) {
    if (stream->available() <= 0) {
      break;
    }
    const int drainedByte = stream->read();
    if (drainedByte < 0) {
      break;
    }
  }

  // Handshake payload: [blank][blockSize][numBlocks][numBytes][blank]
  stream->write(static_cast<byte>(0x00));
  stream->write(kDumpBlockSize & 0xFFU);
  stream->write((kDumpBlockSize >> 8) & 0xFFU);
  stream->write(numBlocks & 0xFFU);
  stream->write((numBlocks >> 8) & 0xFFU);
  stream->write(numBytes & 0xFFU);
  stream->write((numBytes >> 8) & 0xFFU);
  stream->write((numBytes >> 16) & 0xFFU);
  stream->write((numBytes >> 24) & 0xFFU);
  stream->write(static_cast<byte>(0x00));

  bool handshakeReady = false;
  for (uint32_t i = 0U; i < kDumpStreamWaitMaxIters; i++) {
    if (stream->available() > 0) {
      handshakeReady = true;
      break;
    }
  }
  if (!handshakeReady) {
    _file.seek(origPoint);
    return;
  }

  for (uint32_t i = 0U; i < kDumpStreamDrainMaxBytes; i++) {
    if (stream->available() <= 0) {
      break;
    }
    const int drainedByte = stream->read();
    if (drainedByte < 0) {
      break;
    }
  }

  while (curBlock < numBlocks) {
    uint8_t byteBuff[kDumpBlockSize];

    uint32_t amnt = endPos - _file.position();
    if (amnt > kDumpBlockSize) {
      amnt = kDumpBlockSize;
    }
    _file.read(byteBuff, amnt);

    for (uint32_t i = 0U; i < amnt; i++) {
      stream->write(byteBuff[i]);
    }

    if (amnt < kDumpBlockSize) {
      for (uint32_t i = amnt; i < kDumpBlockSize; i++) {
        stream->write(static_cast<byte>(0x00));
      }
    }

    bool ackReady = false;
    for (uint32_t i = 0U; i < kDumpStreamWaitMaxIters; i++) {
      if (stream->available() > 0) {
        ackReady = true;
        break;
      }
    }
    if (!ackReady) {
      _file.seek(origPoint);
      return;
    }

    const int ackRead = stream->read();
    if (ackRead < 0) {
      _file.seek(origPoint);
      return;
    }
    const uint8_t ack = static_cast<uint8_t>(ackRead);
    for (uint32_t i = 0U; i < kDumpStreamDrainMaxBytes; i++) {
      if (stream->available() <= 0) {
        break;
      }
      const int drainedByte = stream->read();
      if (drainedByte < 0) {
        break;
      }
    }

    if (ack == 'N') {
      curBlock++;
    } else if (ack == 'L') {
      _file.seek(_file.position() - amnt);
    } else {
      _file.seek(origPoint);
      return;
    }
  }

  _file.seek(origPoint);
}

void FlashTable::beginDataDump(Stream* stream) {
  flushPendingBuffer();
  beginDataDump(stream, 0U, _file.position());
}

void FlashTable::beginDataDump() {
  flushPendingBuffer();
  beginDataDump(&Serial, 0U, _file.position());
}

bool FlashTable::writeByte(uint8_t in) {
  // Account for both on-flash position and pending buffered bytes.
  const uint32_t logicalPosition = _file.position() + _bufPos;
  if (logicalPosition >= _file.size()) {
    return false;
  }

  if ((_bufferPtr == nullptr) || (_buffSize == 0U)) {
    return _file.write(&in, 1U) == 1U;
  }

  _bufferPtr[_bufPos] = in;
  _bufPos++;

  if (_bufPos == _buffSize) {
    if (_file.write(_bufferPtr, _buffSize) != _buffSize) {
      return false;
    }
    _bufPos = 0U;
  }

  return true;
}

void FlashTable::flushPendingBuffer() {
  if ((_bufPos == 0U) || (_bufferPtr == nullptr)) {
    return;
  }

  _file.write(_bufferPtr, _bufPos);
  _bufPos = 0U;
}

bool FlashTable::writeUint32(uint32_t in) {
  const uint8_t byte1 = static_cast<uint8_t>(0b01111111U & (in >> 24));
  const uint8_t byte2 = static_cast<uint8_t>(in >> 16);
  const uint8_t byte3 = static_cast<uint8_t>(in >> 8);
  const uint8_t byte4 = static_cast<uint8_t>(in);

  bool ok = true;
  ok &= writeByte(byte1);
  ok &= writeByte(byte2);
  ok &= writeByte(byte3);
  ok &= writeByte(byte4); 
  return ok;
}

bool FlashTable::writeRow(uint32_t rowData[]) {
  if ((rowData == nullptr) || (_numCols == 0U) || (_lastValsPtr == nullptr)) {
    return false;
  }

  bool ok = true;

  const bool refreshOrigin = (_originRefreshInt > 0U) && (_rowsSinceRaw >= _originRefreshInt);
  if (!_initialized || refreshOrigin) {
    for (uint8_t col = 0U; col < _numCols; col++) {
      _lastValsPtr[col] = rowData[col];
      ok &= writeUint32(rowData[col]);
    }

    _rowsSinceRaw = 0U;
    _initialized = true;
    return ok;
  }

  for (uint8_t col = 0U; col < _numCols; col++) {
    const uint32_t lastVal = _lastValsPtr[col];
    const uint32_t curVal = rowData[col];

    const bool signAdd = (curVal >= lastVal);
    uint32_t offset = 0U;
    if (signAdd) {
      offset = curVal - lastVal;
    } else {
      offset = lastVal - curVal;
    }

    uint8_t lvl = 4U;
    if (offset <= LVL_2_MAX) {
      lvl = 2U;
    } else if (offset <= LVL_3_MAX) {
      lvl = 3U;
    }

    switch (lvl) {
      case 2U: {
        uint8_t byte1 = static_cast<uint8_t>(0b11100000U | (offset >> 8));
        const uint8_t byte2 = static_cast<uint8_t>(offset);

        if (!signAdd) {
          byte1 = static_cast<uint8_t>(byte1 & 0b10111111U);
        }
        byte1 = static_cast<uint8_t>(byte1 & 0b11011111U);

        ok &= writeByte(byte1);
        ok &= writeByte(byte2);
        break;
      }

      case 3U: {
        uint8_t byte1 = static_cast<uint8_t>(0b11100000U | (offset >> 16));
        const uint8_t byte2 = static_cast<uint8_t>(offset >> 8);
        const uint8_t byte3 = static_cast<uint8_t>(offset);

        if (!signAdd) {
          byte1 = static_cast<uint8_t>(byte1 & 0b10111111U);
        }

        ok &= writeByte(byte1);
        ok &= writeByte(byte2);
        ok &= writeByte(byte3);
        break;
      }

      default: {
        ok &= writeUint32(curVal);
        break;
      }
    }

    _lastValsPtr[col] = curVal;
  }

  _rowsSinceRaw++;
  return ok;
}