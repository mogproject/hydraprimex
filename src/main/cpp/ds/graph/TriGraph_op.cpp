#include "TriGraph.hpp"

namespace ds {
namespace graph {

//============================================================================
//    Operations
//============================================================================
/**
 * @brief Takes the black-edge complement; converts black edges to non-edges
 * and non-edges to black edges.
 */
void TriGraph::black_complement() {
  int n = number_of_vertices();
  if (n <= 1) return;

  auto vs = vertices_.to_vector();
  auto vs_map = util::inverse_map(vs);

  std::vector<std::vector<int>> mat(n, std::vector<int>(n));
  for (auto e : black_edges()) {
    assert(e.first < e.second);
    mat[vs_map[e.first]][vs_map[e.second]] = 1;
  }
  for (auto e : red_edges()) {
    assert(e.first < e.second);
    mat[vs_map[e.first]][vs_map[e.second]] = 2;
  }

  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      if (mat[i][j] == 0) {
        add_black_edge(vs[i], vs[j]);
      } else if (mat[i][j] == 1) {
        remove_black_edge(vs[i], vs[j]);
      }
    }
  }

  // need to recompute properties
  compute_pairwise_properties();
}

/**
 * @brief Create an induced subgraph on the given vertices.
 *
 * @param vs vertices to keep
 * @param reindex compact indices if true
 * @return TriGraph subgraph
 */
TriGraph TriGraph::subgraph(std::vector<TriGraph::Vertex> vs, bool reindex) {
  static ds::set::FastSet fs;
  fs.initialize(number_of_original_vertices());
  for (auto v : vs) fs.set(v);

  if (reindex) {
    VertexList new_vs;
    ColoredEdgeList new_es;

    for (int i : vertices_) {
      if (!fs.get(i)) continue;
      new_vs.push_back(get_label(i));

      for (auto j : adj_black_[i]) {
        if (i < j && fs.get(j)) new_es.push_back({{get_label(i), get_label(j)}, TriGraph::Black});
      }
      for (auto j : adj_red_[i]) {
        if (i < j && fs.get(j)) new_es.push_back({{get_label(i), get_label(j)}, TriGraph::Red});
      }
    }
    return TriGraph(new_vs, new_es);
  } else {
    TriGraph ret(vertex_labels_, {});

    // add edges
    for (int i : vertices_) {
      if (!fs.get(i)) continue;
      for (auto j : adj_black_[i]) {
        if (i < j && fs.get(j)) ret.add_black_edge(i, j);
      }
      for (auto j : adj_red_[i]) {
        if (i < j && fs.get(j)) ret.add_red_edge(i, j);
      }
    }

    // compute pairwise properties
    ret.compute_pairwise_properties();
    return ret;
  }
}

}  // namespace graph
}  // namespace ds
