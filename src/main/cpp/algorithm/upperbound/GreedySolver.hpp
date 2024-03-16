#pragma once
#include <chrono>
#include <unistd.h>

#include "algorithm/base/BaseSolver.hpp"
#include "algorithm/upperbound/WeakRedPotential.hpp"
#include "util/Timer.hpp"

namespace algorithm {
namespace upperbound {

//==============================================================================
//  Signal Handling
//==============================================================================
namespace greedy {

extern bool volatile solver_ignore_alarm;
extern bool volatile solver_terminate_flag;

void solver_alarm_handler(int sig);
void set_timeout(int time_limit_sec);
void reset_timeout();
}  // namespace greedy

//==============================================================================
//  Main Class
//==============================================================================
class GreedySolver : public base::BaseSolver {
 private:
  typedef ds::graph::TriGraph::Vertex Vertex;

  int num_iterations_;
  int time_limit_sec_;
  util::Timer const* root_timer_;
  int volatility_rate_;
  int permissiveness_;
  std::vector<Vertex> frozen_vertices_;
  bool local_search_enabled_;

 public:
  GreedySolver(int num_iterations, int time_limit_sec = 0, util::Timer const* root_timer = nullptr, int volatility_rate = 5000,
               int permissiveness = 0, std::vector<Vertex> frozen_vertices = {}, bool local_search_enabled = true)
      : num_iterations_(num_iterations),
        time_limit_sec_(time_limit_sec),
        root_timer_(root_timer),
        volatility_rate_(volatility_rate),
        permissiveness_(permissiveness),
        frozen_vertices_(frozen_vertices),
        local_search_enabled_(local_search_enabled) {}

  void run(base::SolverState& state, int graph_id, util::Random& rand) override {
    if (state.resolved(graph_id)) return;  // already resolved

    util::Timer timer(time_limit_sec_, root_timer_);
    int time_limit = timer.get_effective_time_limit();
    if (time_limit == 0) return;  // already timed out
    time_limit = std::max(0, time_limit);

    log_info("%s GreedySolver started: n=%lu, num_iterations=%d, time_limit=%s, volat=%d, perm=%d",
             state.label(graph_id).c_str(), state.get_graph(graph_id).number_of_vertices(), num_iterations_,
             time_limit > 0 ? util::format("%ds", time_limit).c_str() : "N/A", volatility_rate_, permissiveness_);

    // set alarm
    if (time_limit > 0) greedy::set_timeout(time_limit);
    bool timed_out = false;

    // main loop
    int t = 0;
    for (; t < num_iterations_ && !state.resolved(graph_id); ++t) {
      if (greedy::solver_terminate_flag) {
        timed_out = true;
        break;
      }

      double volatility = 1.0 + (t / volatility_rate_);
      run_iteration(state, graph_id, rand, volatility, t);
    }

    // cancel alarm
    if (time_limit > 0) greedy::reset_timeout();
    if (timed_out) {
      log_warning("%s GreedySolver timed out: t=%d, elapsed=%.2fs", state.label(graph_id).c_str(), t, timer.stop());
    } else {
      log_info("%s GreedySolver finished: t=%d, runtime=%.2fs", state.label(graph_id).c_str(), t, timer.stop());
    }
  }

 private:
  void run_iteration(base::SolverState& state, int graph_id, util::Random& rand, double volatility = 1.0, int t = 0) {
    // create a WeakRedPotential instance
    auto& g = state.get_graph(graph_id);
    int wrp_ub = state.get_upper_bound(graph_id) + permissiveness_;
    auto criteria = WeakRedPotential(g, rand, wrp_ub, volatility, frozen_vertices_);

    // main loop
    int num_frozen = frozen_vertices_.size();
    int max_red_deg = 0;
    ds::graph::TriGraph::ContractSeq seq;

    while (criteria.number_of_vertices() > std::max(max_red_deg + 1, num_frozen + 1)) {
      // choose the best vertex pair
      auto p = criteria.dequeue();
      if (p.first < 0) {
        // could not improve the current solution
        log_trace("%s GreedySolver (t=%d): could not improve ub=%d", state.label(graph_id).c_str(), t, wrp_ub);
        return;
      }

      // make change and enqueue updated information
      log_trace("%s GreedySolver (t=%d) contracting: %d <- %d", state.label(graph_id).c_str(), t, p.second, p.first);
      auto d = criteria.contract(p.second, p.first);
      assert(d <= state.get_upper_bound(graph_id) + permissiveness_);

      max_red_deg = std::max(max_red_deg, d);
      seq.push_back({g.get_label(p.second), g.get_label(p.first)});
    }

    // rest of the contraction sequence;
    if (criteria.number_of_vertices() > 1 + num_frozen) {
      std::vector<int> vs = criteria.unfrozen_vertices();
      for (std::size_t i = 0; i < vs.size() - 1; ++i) {
        // create contraction sequence
        seq.push_back({g.get_label(vs[vs.size() - 1]), g.get_label(vs[i])});
      }
    }

    assert(static_cast<int>(seq.size()) == static_cast<int>(g.number_of_vertices()) - 1 - num_frozen);

    // store results
    if (state.update_upper_bound(graph_id, max_red_deg, seq)) {
      log_info("%s GreedySolver found new upper bound: t=%d, ub=%d", state.label(graph_id).c_str(), t, max_red_deg);
    }
    if (state.resolved(graph_id)) return;

    if (local_search_enabled_) {
      // local search
      // TODO: implement
    }
  }
};
}  // namespace upperbound
}  // namespace algorithm
