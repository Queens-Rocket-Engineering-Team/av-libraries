#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

template <typename T, std::size_t Capacity = 16U>
class AimQueue {
  static_assert(Capacity > 0U && (Capacity & (Capacity - 1U)) == 0U,
                "Capacity must be a non-zero power of 2");

public:
  AimQueue() : _head(0U), _tail(0U) {}
  ~AimQueue() = default;

  bool push(const T& data) {
    const uint32_t head = _head.load(std::memory_order_relaxed);
    const uint32_t next = (head + 1U) & (static_cast<uint32_t>(Capacity) - 1U);
    if (next == _tail.load(std::memory_order_acquire)) {
      return false;
    }
    _buffer[head] = data;
    _head.store(next, std::memory_order_release);
    return true;
  }

  bool tryPop(T& data) {
    const uint32_t tail = _tail.load(std::memory_order_relaxed);
    if (tail == _head.load(std::memory_order_acquire)) {
      return false;
    }
    data = _buffer[tail];
    const uint32_t next = (tail + 1U) & (static_cast<uint32_t>(Capacity) - 1U);
    _tail.store(next, std::memory_order_release);
    return true;
  }

private:
  T _buffer[Capacity];
  std::atomic<uint32_t> _head;
  std::atomic<uint32_t> _tail;
};

