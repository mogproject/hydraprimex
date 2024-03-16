#pragma once

#include "algorithm/base/BaseSolver.hpp"
#include "algorithm/reduction/Reducer.hpp"
#include "ds/set/FastSet.hpp"
#include "util/Timer.hpp"

namespace algorithm {
namespace exact {

//==============================================================================
//    Signal Handling
//==============================================================================
namespace branch {

extern bool volatile solver_ignore_alarm;
extern bool volatile solver_terminate_flag;

void solver_alarm_handler(int sig);
void set_timeout(int time_limit_sec);
void reset_timeout();
}  // namespace branch

//==============================================================================
//    Main Class
//==============================================================================
class BranchSolver : public base::BaseSolver {
 private:
  constexpr static std::size_t const CACHE_SIZE = 10000000;
  typedef ds::graph::TriGraph::Vertex Vertex;
  typedef ds::graph::TriGraph::VertexList VertexList;
  typedef ds::graph::TriGraph::EdgeList EdgeList;
  typedef ds::graph::TriGraph::ContractSeq ContractSeq;

  int time_limit_sec_;
  util::Timer const* root_timer_;

  long long counter_;
  long long counter_limit_;

  std::vector<Vertex> frozen_vertices_;
  ds::set::FastSet fs_frozen_;

  // temporary variables
  ds::graph::TriGraph graph_;
  int lb_;
  ContractSeq contractions_;
  std::vector<ds::graph::GraphLog> history_;

  // caches
  std::unordered_set<uint64_t> hash_seen_;
  std::vector<uint64_t> hash_seen_vec_;
  util::Random hash_seen_rand_;

 public:
  BranchSolver(int time_limit_sec = 0, long long counter_limit = 100000000L, util::Timer const* root_timer = nullptr,
               std::vector<Vertex> frozen_vertices = {})
      : time_limit_sec_(time_limit_sec),
        root_timer_(root_timer),
        counter_(0),
        counter_limit_(counter_limit),
        frozen_vertices_(frozen_vertices),
        hash_seen_rand_(0)  // Random instance used only for cache eviction
  {
    for (auto x : frozen_vertices) fs_frozen_.set(x);
  }

  void run(base::SolverState& state, int graph_id, util::Random& rand) override {
    if (state.resolved(graph_id)) return;  // already resolved

    util::Timer timer(time_limit_sec_, root_timer_);
    int time_limit = timer.get_effective_time_limit();
    if (time_limit == 0) return;  // already timed out
    time_limit = std::max(0, time_limit);

    auto& g = state.get_graph(graph_id);

    log_info("%s BranchSolver started: n=%lu, time_limit=%s, counter_limit=%lld", state.label(graph_id).c_str(),
             g.number_of_vertices(), time_limit > 0 ? util::format("%ds", time_limit).c_str() : "N/A", counter_limit_);

    // set alarm
    if (time_limit > 0) branch::set_timeout(time_limit);
    bool timed_out = false;

    // main loop
    int ret_code = 0;  // 0: in progress, 1: completed, 2: timed out, 3: counter limit exceeded
    while (!timed_out && !state.resolved(graph_id)) {
      auto lb = state.get_lower_bound(graph_id);
      log_debug("%s BranchSolver checking: lb=%d", state.label(graph_id).c_str(), lb);

      ret_code = search(state, graph_id, rand);
      if (ret_code != 0) break;

      if (state.update_lower_bound(graph_id, lb + 1)) {
        log_debug("%s BranchSolver found new LB: lb=%d, counter=%lld", state.label(graph_id).c_str(), lb + 1, counter_);
      }
    }

    // cancel alarm
    if (time_limit > 0) branch::reset_timeout();

    // show result
    switch (ret_code) {
      case 1: {  // found a solution
        state.update_upper_bound(graph_id, lb_, contractions_);
        log_info("%s BranchSolver found new UB: ub=%d, counter=%lld", state.label(graph_id).c_str(), lb_, counter_);
        break;
      }
      case 2: {  // timed out
        log_warning("%s BranchSolver timed out: elapsed=%.2fs, counter=%lld", state.label(graph_id).c_str(), timer.stop(), counter_);
        break;
      }
      case 3: {  // counter limit exceeded
        log_warning("%s BranchSolver limit exceeded: elapsed=%.2fs, counter=%lld", state.label(graph_id).c_str(),
                    timer.stop(), counter_);
        break;
      }
      default: {
        // do nothing (0: known solution is optimal, 3: counter limit exceeded)
      }
    }

    if (ret_code <= 1) {
      log_info("%s BranchSolver finished: runtime=%.2fs, ret_code=%d, counter=%lld", state.label(graph_id).c_str(),
               timer.stop(), ret_code, counter_);
    }
  }

 private:
  std::vector<Vertex> unfrozen_vertices() const {
    std::vector<Vertex> ret;
    for (auto i : graph_.vertices()) {
      if (!fs_frozen_.get(i)) ret.push_back(i);
    }
    return ret;
  }

  int search(base::SolverState& state, int graph_id, util::Random& rand) {
    // reset variables
    counter_ = 0;
    graph_ = state.get_graph(graph_id);
    lb_ = state.get_lower_bound();  // global lower bound
    contractions_.clear();
    history_.clear();

    // call recursive function
    EdgeList candidates = graph_.find_candidates(lb_);
    return search_recursive(candidates, 0);
  }

  int search_recursive(EdgeList& candidates, int depth) {
    // log_trace("lb=%d, candidates=%s, depth=%d, cs=%s", lb_, cstr(candidates), depth, cstr(contractions_));

    int num_frozen = frozen_vertices_.size();
    int n = graph_.number_of_vertices();
    if (n <= std::max(lb_, num_frozen) + 1) {
      complete_contractions();
      return 1;  // found optimal solution
    }

    //--------------------------------------------------------------------------
    //    Check Runtime Constraints
    //--------------------------------------------------------------------------
    // check counter
    if (counter_limit_ > 0) {
      counter_ += static_cast<long long>(candidates.size());
      if (counter_ > counter_limit_) {
        return 3;  // counter limit exceeded
      }
    }

    // check time
    if (branch::solver_terminate_flag) {
      return 2;  // timed out
    }

    //--------------------------------------------------------------------------
    //    Apply reduction rules
    //--------------------------------------------------------------------------
    auto history_size_before_reduction = history_.size();
    auto contraction_size_before_reduction = contractions_.size();
    int n_reduced = n;

    if (depth > 0) {
      auto reduced = algorithm::reduction::Reducer::reduce(graph_, "", &history_, &candidates, lb_, frozen_vertices_);
      n_reduced = graph_.number_of_vertices();
      util::extend(contractions_, reduced);

      if (!reduced.empty() && n_reduced <= std::max(lb_, num_frozen) + 1) {
        complete_contractions();
        return 1;  // found optimal solution by reductions
      }
    }
    // log_trace("after reduction: candidates=%s, cs=%s", cstr(candidates), cstr(contractions_));

    //--------------------------------------------------------------------------
    //    Branch on all candidates
    //--------------------------------------------------------------------------
    for (auto& p : candidates) {
      int i = p.first;
      int j = p.second;

      // log_trace("contracting: depth=%d, %d <- %d, g=%s, wrp=%d", depth, j, i, cstr(graph_), graph_.weak_red_potential(i, j));
      assert(graph_.weak_red_potential(i, j) <= lb_);

      // some neighbor will exceed the capacity; skip this contraction
      if (graph_.outer_red_potential(i, j) > lb_) {
        // log_trace("(%d <- %d): exceeded outer red potential", j, i);
        continue;
      }

      // do contraction
      history_.push_back({ds::graph::GraphLogType::CONTRACTION});
      graph_.contract(j, i, &history_.back());

      // check cache
      if (has_cache(graph_.hash())) {
        // cache hit
      } else {
        contractions_.push_back({graph_.get_label(j), graph_.get_label(i)});

        // update candidates
        auto next_candidates = graph_.update_candidates(candidates, history_.back(), lb_, frozen_vertices_);

        // recursive call
        auto ret = search_recursive(next_candidates, depth + 1);
        if (ret != 0) return ret;  // immediately return if solution was found

        // set cache
        add_cache(graph_.hash());

        // cancel contraction
        contractions_.pop_back();
      }

      // roll back contraction
      graph_.undo(history_.back());
      history_.pop_back();
      // log_trace("reverting: depth=%d, %d <- %d, g=%s", depth, j, i, cstr(graph_));
    }

    //--------------------------------------------------------------------------
    //    Roll back reductions
    //--------------------------------------------------------------------------
    while (history_.size() > history_size_before_reduction) {
      graph_.undo(history_.back());
      history_.pop_back();
    }
    contractions_.resize(contraction_size_before_reduction);

    return 0;  // no solution
  }

  void complete_contractions() {
    if (graph_.number_of_vertices() <= frozen_vertices_.size() + 1) return;

    auto vs = unfrozen_vertices();

    for (std::size_t i = 0; i < vs.size() - 1; ++i) {
      auto u = graph_.get_label(vs[vs.size() - 1]);
      auto v = graph_.get_label(vs[i]);
      contractions_.push_back({u, v});
    }
  }

  void add_cache(uint64_t hash) {
    if (hash_seen_vec_.size() >= CACHE_SIZE) {
      // random eviction
      int idx = hash_seen_rand_.randint(0, static_cast<int>(hash_seen_vec_.size()) - 1);
      auto hash_to_remove = hash_seen_vec_[idx];

      // override this index
      hash_seen_vec_[idx] = hash;
      hash_seen_.erase(hash_to_remove);
    } else {
      hash_seen_vec_.push_back(hash);
    }
    hash_seen_.insert(hash);
  }

  bool has_cache(uint64_t hash) const { return hash_seen_.find(hash) != hash_seen_.end(); }
};

}  // namespace exact
}  // namespace algorithm
