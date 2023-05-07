#pragma once

#include <array>
#define Byte unsigned char
#define Word unsigned short

struct Memory {
  static constexpr unsigned MEM_LIMIT = 1024 * 64;
  std::array<Byte, MEM_LIMIT> data;

  void initialise();

  // Read a byte
  Byte operator[](unsigned p_address) const { return data[p_address]; }

  // Write a byte
  Byte &operator[](unsigned p_address) { return data[p_address]; }
};