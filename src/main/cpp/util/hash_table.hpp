#pragma once
#include <cstdint>

namespace util {
std::size_t const HASH_TABLE_BITS = 21;
extern uint64_t HASH_TABLE[1 << HASH_TABLE_BITS];

void initialize_hash_table();

uint64_t get_hash(std::size_t index);
}  // namespace util
