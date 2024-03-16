#pragma once

#include "ds/graph/GraphLog.hpp"
#include "ds/graph/TriGraph.hpp"
#include "ds/queue/AdaptivePriorityQueue.hpp"
#include "ds/set/FastSet.hpp"
#include "util/Random.hpp"

namespace algorithm {
namespace upperbound {
class WeakRedPotential {
 private:
  typedef ds::graph::TriGraph::Vertex Vertex;
  typedef ds::graph::TriGraph::EdgeList EdgeList;

  ds::graph::TriGraph graph_;
  int n_;
  util::Random &rand_;
  int ub_;
  int volatility_;
  double const FREE_CONTRACTION_BONUS = -1e5;

  // internal data structures
  std::vector<int> scores_;
  ds::queue::AdaptivePriorityQueue<double> q_;
  ds::set::FastSet fs_frozen_;

 public:
  WeakRedPotential(ds::graph::TriGraph const &graph, util::Random &rand, int ub, int volatility,
                   std::vector<Vertex> const &frozen_vertices = {})
      : graph_(graph),
        n_(graph.number_of_original_vertices()),
        rand_(rand),
        ub_(ub),
        volatility_(volatility),
        scores_(n_ * n_, -1),
        q_(n_ * n_),
        fs_frozen_(n_) {
    // initialize frozen vertices
    for (auto x : frozen_vertices) fs_frozen_.set(x);

    // initialize queue
    auto vs = graph.vertices();
    int n = vs.size();
    for (int i = 0; i < n; ++i) {
      for (int j = i + 1; j < n; ++j) update_element(vs[i], vs[j]);
    }
  }

  int number_of_vertices() const { return graph_.number_of_vertices(); }

  std::vector<Vertex> unfrozen_vertices() const {
    std::vector<Vertex> ret;
    for (auto i : graph_.vertices()) {
      if (!fs_frozen_.get(i)) ret.push_back(i);
    }
    return ret;
  }

  int contract(Vertex j, Vertex i) {
    ds::graph::GraphLog graph_log;
    int ret = graph_.contract(j, i, &graph_log);

    // enqueue vertex pairs
    for (auto &p : graph_log.potential_decreased) update_element(p.first, p.second);
    return ret;
  }

  std::pair<Vertex, Vertex> dequeue() {
    while (!q_.empty()) {
      auto x = q_.top().first;
      q_.pop();
      int i = x / n_;
      int j = x % n_;

      // skip a pair involving a removed vertex
      if (!graph_.has_vertex(i) || !graph_.has_vertex(j)) continue;

      // skip if the score is too high
      if (score(i, j) >= ub_) continue;

      // some neighbor will exceed the capacity; skip this contraction
      if (graph_.outer_red_potential(i, j) >= ub_) continue;

      return {i, j};
    }

    // could not improve the current upper-bound
    return {-1, -1};
  }

 private:
  inline int score(Vertex i, Vertex j) const { return graph_.weak_red_potential(i, j); }

  inline int get_id(Vertex i, Vertex j) const { return std::min(i, j) * n_ + std::max(i, j); }

  void update_element(Vertex i, Vertex j) {
    assert(graph_.has_vertex(i) && graph_.has_vertex(j));

    if (fs_frozen_.get(i) || fs_frozen_.get(j)) return;  // never contract frozen vertices

    int id = get_id(i, j);
    int s = score(i, j);
    if (scores_[id] == s) return;  // no change

    if (s >= ub_) return;  // potential too high

    auto rand_val = rand_.random() * volatility_;
    scores_[id] = s;

    double bonus = graph_.is_free_contraction(i, j) ? FREE_CONTRACTION_BONUS : 0;
    q_.push(id, s + rand_val + bonus);
  }
};
}  // namespace upperbound
}  // namespace algorithm
