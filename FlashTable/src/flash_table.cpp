#include "flash_table.h"

FlashTable::FlashTable(uint8_t numCols,
                       uint16_t originRefreshInt,
                       uint32_t maxSize,
                       uint8_t tableNum,
                       uint16_t buffSize)
    : _maxSize(maxSize),
      _tableNum(tableNum),
      _buffSize(buffSize),
      _numCols(numCols),
      _originRefreshInt(originRefreshInt),
      _rowsSinceRaw(0U),
      _initialized(false),
      _lastValsPntr(nullptr),
      _bufferPntr(nullptr),
      _bufPos(0U) {
  if (_numCols > 0U) {
    _lastValsPntr = new uint32_t[_numCols];
  }
  if (_buffSize > 0U) {
    _bufferPntr = new uint8_t[_buffSize];
  }
}

FlashTable::~FlashTable() {
  flushPendingBuffer();
  delete[] _lastValsPntr;
  delete[] _bufferPntr;
}

void FlashTable::init(SerialFlashChip* serialFlash, Stream* stream) {
  (void)stream;
  if (serialFlash == nullptr) {
    return;
  }

  char fName[8];
  // Preserve legacy behavior: table number is rendered in base-8.
  itoa(_tableNum, fName, 8);

  bool findEmptySpot = true;
  if (!serialFlash->exists(fName)) {
    serialFlash->create(fName, _maxSize);
    findEmptySpot = false;
  }

  _file = serialFlash->open(fName);
  _bufPos = 0U;
  _rowsSinceRaw = 0U;
  _initialized = false;

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

uint32_t FlashTable::getMaxSize() {
  return _file.size();
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

  while (stream->available()) {
    stream->read();
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

  while (stream->available() == 0) {
  }
  while (stream->available() > 0) {
    stream->read();
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

    while (!stream->available()) {
    }

    const uint8_t ack = stream->read();
    while (stream->available()) {
      stream->read();
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

  if ((_bufferPntr == nullptr) || (_buffSize == 0U)) {
    return _file.write(&in, 1U) == 1U;
  }

  _bufferPntr[_bufPos] = in;
  _bufPos++;

  if (_bufPos == _buffSize) {
    if (_file.write(_bufferPntr, _buffSize) != _buffSize) {
      return false;
    }
    _bufPos = 0U;
  }

  return true;
}

void FlashTable::flushPendingBuffer() {
  if ((_bufPos == 0U) || (_bufferPntr == nullptr)) {
    return;
  }

  _file.write(_bufferPntr, _bufPos);
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
  if ((rowData == nullptr) || (_numCols == 0U) || (_lastValsPntr == nullptr)) {
    return false;
  }

  bool ok = true;

  const bool refreshOrigin = (_originRefreshInt > 0U) && (_rowsSinceRaw >= _originRefreshInt);
  if (!_initialized || refreshOrigin) {
    for (uint8_t col = 0U; col < _numCols; col++) {
      _lastValsPntr[col] = rowData[col];
      ok &= writeUint32(rowData[col]);
    }

    _rowsSinceRaw = 0U;
    _initialized = true;
    return ok;
  }

  for (uint8_t col = 0U; col < _numCols; col++) {
    const uint32_t lastVal = _lastValsPntr[col];
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

    _lastValsPntr[col] = curVal;
  }

  _rowsSinceRaw++;
  return ok;
}