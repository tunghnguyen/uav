#pragma once

#include <cstdint>

template <typename T, uint32_t cap>
class Buffer {
private:
  T data[cap];
  uint32_t rear = 0;

public:
  Buffer() = default;

  T operator[](uint32_t k) const;

  T read() const;

  void write(T val);
};
