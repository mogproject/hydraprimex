#include "TriGraph.hpp"

namespace ds {
namespace graph {

TriGraph::ColoredEdgeList TriGraph::to_colored_edges(TriGraph::EdgeList const& edges) {
  TriGraph::ColoredEdgeList ret;
  for (auto& e : edges) ret.push_back({e, Black});
  return ret;
}

}  // namespace graph
}  // namespace ds
