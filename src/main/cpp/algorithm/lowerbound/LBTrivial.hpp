#pragma once

#include "algorithm/base/BaseSolver.hpp"
#include "ds/graph/TriGraph.hpp"
#include "util/Timer.hpp"

namespace algorithm {
namespace lowerbound {
class LBTrivial : public base::BaseSolver {
 private:
 public:
  LBTrivial() {}

  void run(base::SolverState &state, int graph_id, util::Random &rand) override {
    util::Timer timer;
    auto &g = state.get_graph(graph_id);
    log_info("%s LBTrivial started: graph=%s", state.label(graph_id).c_str(), cstr(g));

    auto vs = g.vertices();
    int n = vs.size();
    int lb = n;
    for (int i = 0; i < n; ++i) {
      for (int j = i + 1; j < n; ++j) {
        //
        lb = std::min(lb, g.weak_red_potential(vs[i], vs[j]));
      }
    }

    if (state.update_lower_bound(graph_id, lb)) {
      // logging
      log_debug("%s LBTrivial found new LB: %d", state.label(graph_id).c_str(), lb);
    }
    log_info("%s LBTrivial finished: lb=%d, elapsed=%.2fs", state.label(graph_id).c_str(), lb, timer.stop());
  }
};
}  // namespace lowerbound
}  // namespace algorithm
