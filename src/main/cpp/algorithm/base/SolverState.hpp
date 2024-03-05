#pragma once

#include "ds/graph/TriGraph.hpp"

namespace algorithm {
namespace base {
/**
 * @brief Manages solver states and solutions.
 */
class SolverState {
 private:
  typedef int GraphID;

  /** Global lower bound. */
  int global_lower_bound_;

  /** Ordered list of graphs. */
  std::vector<ds::graph::TriGraph> graphs_;

  /** Local lower bound. */
  std::vector<int> lower_bounds_;

  /** Local upper bound. */
  std::vector<int> upper_bounds_;

  /** Contraction sequences. */
  std::vector<ds::graph::TriGraph::ContractSeq> contractions_;

  /** Partial contraction sequences (reduced solution). */
  std::vector<ds::graph::TriGraph::ContractSeq> contractions_part_;

 public:
  /**
   * @brief Summarize local and global lower bounds and local upper bound.
   *
   * @param graph_id graph id
   * @return std::string label representation
   */
  std::string label(int graph_id) const {
    return util::format("L%d[G%d:L%dU%d]", global_lower_bound_, graph_id, lower_bounds_[graph_id], upper_bounds_[graph_id]);
  }

  SolverState(ds::graph::TriGraph const& graph, int lower_bound, int upper_bound) : global_lower_bound_(lower_bound) {
    add_graph(graph, lower_bound, upper_bound);
    log_debug("%s Initialized SolverState.", label(0).c_str());
  }

  GraphID get_unresolved_graph_id() const {
    int k = graphs_.size();
    for (int i = 0; i < k; ++i) {
      // TODO: sort by graph size
      if (!resolved(i)) return i;
    }
    return -1;  // all done
  }

  inline ds::graph::TriGraph const& get_graph(GraphID graph_id) const { return graphs_[graph_id]; }
  inline ds::graph::TriGraph& get_graph(GraphID graph_id) { return graphs_[graph_id]; }

  bool resolved() const {
    for (auto ub : upper_bounds_) {
      if (global_lower_bound_ < ub) return false;
    }
    return true;
  }

  bool resolved(GraphID graph_id) const { return global_lower_bound_ >= upper_bounds_[graph_id]; }

  /**
   * @brief Returns the current contraction sequence of the entire graph in the original labels.
   *
   * @return ds::graph::TriGraph::ContractSeq contraction sequence
   */
  ds::graph::TriGraph::ContractSeq contraction_sequence() const {
    ds::graph::TriGraph::ContractSeq ret;
    int k = contractions_.size();
    for (int i = k - 1; i >= 0; --i) util::extend(ret, contractions_[i]);
    return ret;
  }

  /**
   * @brief Returns the current contraction sequence of the given graph in the original labels.
   *
   * @param graph_id graph id
   * @return ds::graph::TriGraph::ContractSeq contraction sequence
   */
  ds::graph::TriGraph::ContractSeq contraction_sequence(GraphID graph_id) const {
    ds::graph::TriGraph::ContractSeq ret;
    auto& g = graphs_[graph_id];
    for (auto& p : contractions_[graph_id]) ret.push_back({g.get_label(p.first), g.get_label(p.second)});
    return ret;
  }

  /**
   * @brief Updates local lower bound.
   *
   * @param graph_id graph id
   * @param lower_bound new lower bound
   *
   * @return true lower bound was updated
   * @return false lower bound was not updated
   */
  bool update_lower_bound(GraphID graph_id, int lower_bound) {
    if (lower_bound > lower_bounds_[graph_id]) {
      lower_bounds_[graph_id] = lower_bound;
      global_lower_bound_ = std::max(global_lower_bound_, lower_bound);  // update global lower bound
      return true;
    }
    assert(lower_bounds_[graph_id] <= global_lower_bound_);
    return false;
  }

  void add_partial_solution(int graph_id, int lower_bound, ds::graph::TriGraph::ContractSeq const& contractions) {
    update_lower_bound(graph_id, lower_bound);
    util::extend(contractions_part_[graph_id], contractions);
  }

  bool update_upper_bound(int graph_id, int upper_bound, ds::graph::TriGraph::ContractSeq const& contractions) {
    if (upper_bound >= 0 && upper_bound < upper_bounds_[graph_id]) {
      upper_bounds_[graph_id] = upper_bound;
      contractions_[graph_id] = contractions_part_[graph_id];
      util::extend(contractions_[graph_id], contractions);
      return true;
    }
    return false;
  }

  void update_exact(int graph_id, int upper_bound, ds::graph::TriGraph::ContractSeq const& contractions) {
    update_lower_bound(graph_id, upper_bound);
    update_upper_bound(graph_id, upper_bound, contractions);
  }

  int get_lower_bound() const { return global_lower_bound_; }
  int get_lower_bound(GraphID graph_id) const { return lower_bounds_[graph_id]; }
  int get_upper_bound() const {
    int ret = global_lower_bound_;
    for (auto ub : upper_bounds_) ret = std::max(ret, ub);
    return ret;
  }
  int get_upper_bound(GraphID graph_id) const { return upper_bounds_[graph_id]; }

  void refresh_trivial_upper_bound(GraphID graph_id) {
    auto trivial_cs = get_trivial_contraction_sequence(0);
    int trivial_ub = ds::graph::TriGraph::verify_contraction_sequence(graphs_[graph_id], trivial_cs);
    if (update_upper_bound(graph_id, trivial_ub, trivial_cs)) {
      log_info("%s Updated upper bound to trivial contractions: %d", label(graph_id).c_str(), get_upper_bound(graph_id));
    }
  }

 private:
  void add_graph(ds::graph::TriGraph const& graph, int lower_bound, int upper_bound) {
    int graph_id = graphs_.size();
    graphs_.push_back(graph);
    lower_bounds_.push_back(lower_bound);
    upper_bounds_.push_back(upper_bound < 0 ? graph.number_of_vertices() - 1 : upper_bound);
    contractions_.push_back({});
    contractions_part_.push_back({});

    // adjust lower bound
    if (update_lower_bound(graph_id, graph.max_red_degree())) {
      log_info("%s Updated lower bound to initial max red degree: %d", label(graph_id).c_str(), get_lower_bound(graph_id));
    }
    assert(lower_bounds_[graph_id] <= global_lower_bound_);

    // adjust upper bound
    refresh_trivial_upper_bound(graph_id);
  }

  ds::graph::TriGraph::ContractSeq get_trivial_contraction_sequence(int graph_id) {
    ds::graph::TriGraph::ContractSeq ret;
    auto& g = graphs_[graph_id];
    auto vs = g.vertices();
    int n = vs.size();
    for (int i = 0; i < n - 1; ++i) ret.push_back({g.get_label(vs[n - 1]), g.get_label(vs[i])});
    return ret;
  }
};
}  // namespace base
}  // namespace algorithm
