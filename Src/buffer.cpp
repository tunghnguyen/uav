#include "buffer.hpp"

#include <cstdint>

template <typename T, uint32_t cap>
T Buffer<T, cap>::operator[](uint32_t k) const {
  return data[(rear + cap + k) % cap];
}

template <typename T, uint32_t cap>
T Buffer<T, cap>::read() const {
  return data[rear];
}

template <typename T, uint32_t cap>
void Buffer<T, cap>::write(T val) {
  rear = (rear + 1) % cap;
  data[rear] = val;
}
