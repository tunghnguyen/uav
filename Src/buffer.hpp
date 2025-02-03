#pragma once

#include <cstdint>

class Buffer {
private:
  int32_t *data;
  uint32_t cap;
  uint32_t rear = 0;

public:
  Buffer(int32_t data[], uint32_t cap);

  int32_t operator[](uint32_t idx) const;

  void push(int32_t val);
};
