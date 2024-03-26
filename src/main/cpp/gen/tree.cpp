#include "tree.hpp"

namespace gen {
/**
 * @brief Generates a path TriGraph of `n` vertices.
 *
 * @param n number of vertices
 * @return TriGraph instance
 */
ds::graph::TriGraph path_graph(int n) { return full_rary_tree(1, n); }

/**
 * @brief Generates a full `r`-ary tree of `n` vertices.
 *
 * @param r branching factor
 * @param n number of vertices
 * @return TriGraph instance
 */
ds::graph::TriGraph full_rary_tree(int r, int n) {
  if (r <= 0) throw std::invalid_argument("r must be positive");
  ds::graph::TriGraph::ColoredEdgeList edges;
  for (int i = 1; i < n; ++i) edges.push_back({{(i - 1) / r, i}, ds::graph::TriGraph::Black});
  return ds::graph::TriGraph(util::range_to_vec(n), edges);
}

/**
 * @brief Generates a random tree of `n` vertices.
 *
 * @param rand util::Random instance
 * @param n number of vertices
 * @return TriGraph instance
 */
ds::graph::TriGraph random_tree(util::Random& rand, int n) {
  std::vector<int> labels;

  // randomize vertices
  for (int i = 0; i < n; ++i) labels.push_back(i);
  rand.shuffle(labels);

  // add random edges
  ds::graph::TriGraph::ColoredEdgeList edges;
  for (int i = 1; i < n; ++i) {
    int parent = rand.randint(0, i - 1);
    edges.push_back({{labels[parent], labels[i]}, ds::graph::TriGraph::Black});
  }

  return ds::graph::TriGraph(util::range_to_vec(n), edges);
}

}  // namespace gen
