#include "TriGraph.hpp"

namespace ds {
namespace graph {

//============================================================================
//    I/O
//============================================================================
/**
 * @brief String representation of the instance.
 *
 * @param os output stream
 * @param graph Graph instance
 * @return std::ostream& output stream
 */
std::ostream& operator<<(std::ostream& os, TriGraph const& graph) {
  return os << util::format(                          //
             "TriGraph(n=%lu, m=%lu, hash=%016llx)",  //
             graph.number_of_vertices(), graph.number_of_edges(), graph.hash_);
}

}  // namespace graph
}  // namespace ds
