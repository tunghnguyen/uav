#include "buffer.hpp"

#include <cstdint>

Buffer::Buffer(int32_t data[], uint32_t cap) : data(data), cap(cap) {}

int32_t Buffer::operator[](uint32_t idx) const {
  return data[(rear + cap - idx) % cap];
}

void Buffer::push(int32_t val) {
  rear = (rear + 1) % cap;
  data[rear] = val;
}
