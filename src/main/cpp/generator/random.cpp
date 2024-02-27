#include "random.hpp"

namespace generator {
ds::graph::TriGraph erdos_renyi_graph(int n, double p, util::Random &rand) {
  ds::graph::TriGraph::VertexList vs = util::range_to_vec(n);
  ds::graph::TriGraph::ColoredEdgeList es;

  for (int i = 0; i < n - 1; ++i) {
    for (int j = i + 1; j < n; ++j) {
      if (rand.random() < p) es.push_back({{i, j}, ds::graph::TriGraph::Black});
    }
  }
  return ds::graph::TriGraph(vs, es);
}
}  // namespace generator
