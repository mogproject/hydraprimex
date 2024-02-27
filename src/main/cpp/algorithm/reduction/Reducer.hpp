#pragma once

#include "ds/graph/TriGraph.hpp"

namespace algorithm {
namespace reduction {
class Reducer {
 public:
  ds::graph::TriGraph::ContractSeq reduce(ds::graph::TriGraph &graph) {
    ds::graph::TriGraph::ContractSeq ret;

    bool updated = true;
    while (updated) {
      updated = false;

      //------------------------------------------------------------------------
      // (1) Take complement
      //------------------------------------------------------------------------
      auto n = graph.number_of_vertices();
      auto mb = graph.number_of_black_edges();
      auto mr = graph.number_of_red_edges();

      if (4 * mb > n * (n - 1) - 2 * mr) {
        graph.black_complement();
        log_debug("Reducer: took the complement: n=%lu, m=%lu, m_red=%lu", graph.number_of_vertices(),
                  graph.number_of_edges(), graph.number_of_red_edges());
      }

      //------------------------------------------------------------------------
      // (2) Free contraction
      //------------------------------------------------------------------------
      auto vs = graph.vertices();
      for (std::size_t i = 0; i < n && !updated; ++i) {
        auto u = vs[i];
        for (std::size_t j = i + 1; j < n && !updated; ++j) {
          auto v = vs[j];
          assert(u < v);

          if (graph.is_free_contraction(v, u)) {
            updated = true;
            graph.contract(v, u);
            ret.push_back({v, u});
            log_debug("Reducer: free contraction: (%d <- %d)", v, u);
            break;
          }
        }
        if (updated) break;
      }
    }

    return ret;
  }
};
}  // namespace reduction
}  // namespace algorithm
