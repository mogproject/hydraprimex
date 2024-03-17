#pragma once

#include "ds/graph/TriGraph.hpp"
#include "sat/SATAdapter.hpp"

namespace algorithm {
namespace preprocess {
class VertexSeparator {
 private:
  typedef ds::graph::TriGraph::Vertex Vertex;

 public:
  mutable sat::SATAdapter solver_;

  VertexSeparator() {}

  /**
   * @brief Returns a list of the articulation points (cut vertices) of the graph.
   *
   * @return articulation points
   */
  std::vector<Vertex> find_articulation_points(ds::graph::TriGraph const& g) const;

  std::vector<ds::graph::TriGraph> decompose_into_covers(ds::graph::TriGraph const& graph, int max_separator_size);

  /**
   * @brief Finds a vertex separator of size at most `max_separator_size`.
   * Removing these vertices from the graph would make it disconnected.
   *
   * @param max_separator_size max size of a separator
   * @return list of vertices in the separator; empty list if solution was not found
   */
  std::vector<Vertex> find_vertex_separator(         //
      ds::graph::TriGraph const& graph,              //
      int max_separator_size,                        //
      std::vector<Vertex>* part_a = nullptr,         //
      std::vector<Vertex>* part_b = nullptr,         //
      std::vector<Vertex> const& head = {},          //
      std::vector<Vertex> const& tail = {},          //
      util::Random* rand = nullptr,                  //
      bool allow_overlap = true,                     //
      bool require_balance = false,                  //
      std::vector<Vertex> const& non_separator = {}  //
  ) const;

 private:
  inline int x(int i) const;
  inline int y(int i) const;
  inline int z(int i) const;
};
}  // namespace preprocess
}  // namespace algorithm
