#include "TriGraph.hpp"

namespace ds {
namespace graph {

//============================================================================
//    Hashing
//============================================================================
uint64_t TriGraph::vertex_hash(int i) const {
  auto mod = 1 << (util::HASH_TABLE_BITS - 1);
  return util::get_hash(mod + i % mod);
}

uint64_t TriGraph::red_edge_hash(int i, int j) const {
  auto mod = 1 << ((util::HASH_TABLE_BITS - 1) / 2);
  return util::get_hash((std::min(i, j) % mod) * mod + (std::max(i, j) % mod));
}

}  // namespace graph
}  // namespace ds
