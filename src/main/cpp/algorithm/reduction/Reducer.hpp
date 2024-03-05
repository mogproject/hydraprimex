#pragma once

#include "algorithm/base/BaseSolver.hpp"
#include "ds/graph/TriGraph.hpp"

namespace algorithm {
namespace reduction {
class Reducer : public base::BaseSolver {
 public:
  void run(base::SolverState &state, int graph_id, util::Random &rand) override {
    auto &g = state.get_graph(graph_id);

    ds::graph::TriGraph::ContractSeq seq;
    bool updated = true;
    while (updated) {
      updated = false;

      //------------------------------------------------------------------------
      // (1) Take complement
      //------------------------------------------------------------------------
      auto n = g.number_of_vertices();
      auto mb = g.number_of_black_edges();
      auto mr = g.number_of_red_edges();

      if (n >= 5 && 4 * mb > n * (n - 1) - 2 * mr) {
        g.black_complement();
        log_debug("%s Reducer: took the complement: n=%lu, m=%lu, m_red=%lu", state.label(graph_id).c_str(),
                  g.number_of_vertices(), g.number_of_edges(), g.number_of_red_edges());
      }

      //------------------------------------------------------------------------
      // (2) Free contraction
      //------------------------------------------------------------------------
      auto vs = g.vertices();
      for (std::size_t i = 0; i < n && !updated; ++i) {
        auto u = vs[i];
        for (std::size_t j = i + 1; j < n && !updated; ++j) {
          auto v = vs[j];
          assert(u < v);

          if (g.is_free_contraction(v, u)) {
            updated = true;
            g.contract(v, u);
            auto vv = g.get_label(v);
            auto uu = g.get_label(u);
            seq.push_back({vv, uu});
            log_debug("%s Reducer: free contraction: (%d <- %d)", state.label(graph_id).c_str(), vv, uu);
            break;
          }
        }
        if (updated) break;
      }
    }

    auto lb = state.get_lower_bound(graph_id);
    state.add_partial_solution(graph_id, lb, seq);

    if (state.get_graph(graph_id).number_of_vertices() <= 1) {
      // reducible instance
      state.update_exact(graph_id, lb, {});
      log_info("%s Reducer: reducible instance: red_deg=%d", state.label(graph_id).c_str(), lb);
    } else if (!seq.empty()) {
      log_info("%s Reducer: applied %lu contraction(s)", state.label(graph_id).c_str(), seq.size());
      state.refresh_trivial_upper_bound(graph_id);
    }
  }
};
}  // namespace reduction
}  // namespace algorithm
