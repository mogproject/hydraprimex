#pragma once

#include "ds/graph/TriGraph.hpp"
#include "util/Random.hpp"

namespace gen {
/**
 * @brief Generates a path graph of `n` vertices.
 *
 * @param n number of vertices
 * @return TriGraph instance
 */
ds::graph::TriGraph path_graph(int n);

/**
 * @brief Generates a full `r`-ary tree of `n` vertices.
 *
 * @param r branching factor
 * @param n number of vertices
 * @return TriGraph instance
 */
ds::graph::TriGraph full_rary_tree(int r, int n);

/**
 * @brief Generates a random tree of `n` vertices.
 *
 * @param rand util::Random instance
 * @param n number of vertices
 * @return TriGraph instance
 */
ds::graph::TriGraph random_tree(util::Random& rand, int n);
}  // namespace gen
