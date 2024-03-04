#pragma once

#include "ds/graph/TriGraph.hpp"
#include "util/Timer.hpp"

namespace algorithm {
namespace lowerbound {
class LBTrivial {
 private:
  ds::graph::TriGraph const& graph_;

 public:
  LBTrivial(ds::graph::TriGraph const& graph) : graph_(graph) {}

  int run() const {
    log_info("LBTrivial started: graph=%s", cstr(graph_));
    util::Timer timer;

    auto vs = graph_.vertices();
    int n = vs.size();
    int ret = n;
    for (int i = 0; i < n; ++i) {
      for (int j = i + 1; j < n; ++j) {
        //
        ret = std::min(ret, graph_.weak_red_potential(vs[i], vs[j]));
      }
    }

    log_info("LBTrivial finished: lb=%d, elapsed=%.2fs", ret, timer.stop());
    return ret;
  }
};
}  // namespace lowerbound
}  // namespace algorithm
