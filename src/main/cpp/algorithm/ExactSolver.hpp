#pragma once

#include "algorithm/exact/SATSolver.hpp"
#include "algorithm/lowerbound/LBTrivial.hpp"
#include "algorithm/reduction/Reducer.hpp"
#include "util/Timer.hpp"

namespace algorithm {
class ExactSolver {
 private:
  ds::graph::TriGraph const& graph_;
  util::Timer const* root_timer_;
  ds::graph::TriGraph::ContractSeq seq_;  // contraction sequence
  int lower_bound_;
  int upper_bound_;

 public:
  ExactSolver(ds::graph::TriGraph const& graph, util::Timer const* root_timer = nullptr)
      : graph_(graph), root_timer_(root_timer) {}

  int twin_width() const {
    assert(lower_bound_ == upper_bound_);
    return upper_bound_;
  }

  bool resolved() const {
    if (lower_bound_ != upper_bound_) return false;
    if (graph_.number_of_vertices() == 0) return true;

    // log_warning("n=%lu, sz=%lu, seq=%s", graph_.number_of_vertices(), seq_.size(), cstr(seq_));
    return graph_.number_of_vertices() == seq_.size() + 1;
  }

  ds::graph::TriGraph::ContractSeq contraction_sequence() const { return seq_; }

  void run(util::Random& rand, int lb = 0, int ub = -1) {
    lower_bound_ = lb;
    upper_bound_ = ub < 0 ? graph_.number_of_vertices() : ub;

    ds::graph::TriGraph g = graph_;  // create a copy

    //--------------------------------------------------------------------------
    //    Apply reduction rules
    //--------------------------------------------------------------------------
    reduction::Reducer reducer;

    auto reduced_seq = reducer.reduce(g);
    util::extend(seq_, reduced_seq);
    if (g.number_of_vertices() <= 1) {
      upper_bound_ = lower_bound_;
      log_debug("ExactSolver: reducible instance: tww=%d", upper_bound_);
      return;  // reducible instance
    }
    g.compute_pairwise_properties();  // FIXME: This shouldn't be needed.

    //--------------------------------------------------------------------------
    //    LBTrivial (tentative implementation)
    //--------------------------------------------------------------------------
    lowerbound::LBTrivial lb_trivial(g);
    int new_lb = lb_trivial.run();
    if (lower_bound_ < new_lb) {
      log_debug("LBTrivial: new LB: %d", new_lb);
      lower_bound_ = new_lb;
    }

    //--------------------------------------------------------------------------
    //    SAT solver
    //--------------------------------------------------------------------------
    algorithm::exact::SATSolver sat_solver(g);

    util::Timer lap_timer(0, root_timer_);
    bool solved = sat_solver.run(lb, ub, lap_timer.get_effective_time_limit());
    if (solved) {
      util::extend(seq_, sat_solver.contraction_sequence());
      lower_bound_ = sat_solver.lower_bound();
      upper_bound_ = sat_solver.upper_bound();
      return;
    }
  }
};
}  // namespace algorithm
