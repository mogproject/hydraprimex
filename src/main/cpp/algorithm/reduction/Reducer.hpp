#pragma once

#include "algorithm/base/BaseSolver.hpp"
#include "ds/graph/TriGraph.hpp"
#include "ds/set/FastSet.hpp"

namespace algorithm {
namespace reduction {
class Reducer : public base::BaseSolver {
 private:
  typedef ds::graph::TriGraph::Vertex Vertex;

 public:
  static ds::graph::TriGraph::ContractSeq reduce(                    //
      ds::graph::TriGraph &g,                                        //
      std::string const &log_prefix = "",                            //
      std::vector<ds::graph::GraphLog> *graph_logs = nullptr,        //
      std::vector<std::pair<Vertex, Vertex>> *candidates = nullptr,  // candidate contraction sequences
      int upper_bound = -1,                                          //
      std::vector<Vertex> frozen_vertices = {}                       //
  ) {
    ds::graph::TriGraph::ContractSeq seq;

    static ds::set::FastSet frozen;
    frozen.initialize(g.number_of_original_vertices());
    for (auto x : frozen_vertices) frozen.set(x);

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
        if (graph_logs) graph_logs->push_back({ds::graph::GraphLogType::COMPLEMENT});
        log_trace("%s Reducer: took the complement: n=%lu, m=%lu, m_red=%lu", log_prefix.c_str(),
                  g.number_of_vertices(), g.number_of_edges(), g.number_of_red_edges());
      }

      //------------------------------------------------------------------------
      // (2) Free contraction
      //------------------------------------------------------------------------
      ds::graph::GraphLog log;
      if (candidates) {
        for (auto &p : *candidates) {
          if (g.is_free_contraction(p.first, p.second)) {
            seq.push_back(perform_contraction(p.first, p.second, g, log_prefix.c_str(), graph_logs, candidates,
                                              upper_bound, frozen_vertices));
            updated = true;
            break;
          }
        }
      } else {
        auto vs = g.vertices();
        for (std::size_t i = 0; i < n && !updated; ++i) {
          auto u = vs[i];
          if (frozen.get(u)) continue;

          for (std::size_t j = i + 1; j < n && !updated; ++j) {
            auto v = vs[j];
            assert(u < v);
            if (frozen.get(v)) continue;

            if (g.is_free_contraction(u, v)) {
              seq.push_back(perform_contraction(u, v, g, log_prefix.c_str(), graph_logs, candidates, upper_bound, frozen_vertices));
              updated = true;
              break;
            }
          }
          if (updated) break;
        }
      }
    }
    return seq;
  }

  void run(base::SolverState &state, int graph_id, util::Random &rand) override {
    auto seq = Reducer::reduce(state.get_graph(graph_id), state.label(graph_id));

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

 private:
  inline static std::pair<Vertex, Vertex> perform_contraction(  //
      int u,                                                    //
      int v,                                                    //
      ds::graph::TriGraph &g,                                   //
      std::string const &log_prefix,                            //
      std::vector<ds::graph::GraphLog> *graph_logs,             //
      std::vector<std::pair<Vertex, Vertex>> *candidates,       //
      int upper_bound,                                          //
      std::vector<Vertex> frozen_vertices                       //
  ) {
    ds::graph::GraphLog log;
    g.contract(v, u, &log);
    if (graph_logs) graph_logs->push_back(log);

    auto vv = g.get_label(v);
    auto uu = g.get_label(u);

    if (candidates && upper_bound >= 0) {
      // update candidates
      *candidates = g.update_candidates(*candidates, log, upper_bound, frozen_vertices);
    }
    log_trace("%s Reducer: free contraction: (%d <- %d)", log_prefix.c_str(), vv, uu);
    return {vv, uu};
  }
};
}  // namespace reduction
}  // namespace algorithm
