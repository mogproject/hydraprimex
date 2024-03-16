#pragma once

#include "algorithm/base/BaseSolver.hpp"
#include "algorithm/base/SolverState.hpp"
#include "algorithm/exact/BranchSolver.hpp"
#include "algorithm/exact/SATSolver.hpp"
#include "algorithm/lowerbound/LBGreedy.hpp"
#include "algorithm/reduction/Reducer.hpp"
#include "algorithm/upperbound/GreedySolver.hpp"
#include "util/Timer.hpp"

namespace algorithm {
class ExactSolver : public base::BaseSolver {
 private:
  util::Timer const* root_timer_;

 public:
  ExactSolver(util::Timer const* root_timer = nullptr) : root_timer_(root_timer) {}

  void run(base::SolverState& state, int graph_id, util::Random& rand) override {
    util::Timer timer;
    log_debug("%s ExactSolver started", state.label(0).c_str());

    //--------------------------------------------------------------------------
    //    Set up solvers
    //--------------------------------------------------------------------------
    std::vector<std::unique_ptr<base::BaseSolver>> solvers;

    solvers.push_back(std::make_unique<reduction::Reducer>());
    solvers.push_back(std::make_unique<lowerbound::LBGreedy>());
    solvers.push_back(std::make_unique<exact::BranchSolver>(5, 100000L));                              // quick branch
    solvers.push_back(std::make_unique<upperbound::GreedySolver>(1000, 2, root_timer_, 500, 0));       // quick greedy
    solvers.push_back(std::make_unique<exact::SATSolver>(root_timer_, 10));                            // quick SAT
    solvers.push_back(std::make_unique<upperbound::GreedySolver>(100000, 30, root_timer_, 10000, 0));  // greedy
    solvers.push_back(std::make_unique<exact::SATSolver>(root_timer_));

    //--------------------------------------------------------------------------
    //    Run solvers
    //--------------------------------------------------------------------------
    bool timed_out = false;
    while (!timed_out) {
      if ((graph_id = state.get_unresolved_graph_id()) < 0) {
        // all done
        break;
      }

      for (std::size_t i = 0; i < solvers.size(); ++i) {
        solvers[i]->run(state, graph_id, rand);
        if (state.resolved(graph_id)) {
          break;
        } else if (root_timer_ && root_timer_->is_time_over()) {
          timed_out = true;
          break;
        }
      }
    }
    log_debug("%s ExactSolver finished: elapsed=%.2fs", state.label(0).c_str(), timer.stop());
  }
};
}  // namespace algorithm
