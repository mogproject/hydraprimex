#include "hash_table.hpp"
#include "Random.hpp"

namespace util {
uint64_t HASH_TABLE[1 << HASH_TABLE_BITS];

static bool hash_table_initialized = false;

void initialize_hash_table() {
  if (hash_table_initialized) return;

  util::Random rand(12345);
  for (std::size_t i = 0; i < (1 << util::HASH_TABLE_BITS); ++i) {
    // create random nonzero number
    HASH_TABLE[i] = rand.randint<uint64_t>(1, static_cast<uint64_t>(-1));
  }
  hash_table_initialized = true;
}

uint64_t get_hash(std::size_t index) { return HASH_TABLE[index]; }
}  // namespace util
