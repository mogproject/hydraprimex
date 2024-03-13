#pragma once

#include "algorithm/base/BaseSolver.hpp"
#include "ds/queue/BucketQueue.hpp"
#include "ds/set/FastSet.hpp"
#include "util/Timer.hpp"

namespace algorithm {
namespace lowerbound {
class LBGreedy : public base::BaseSolver {
 private:
  int num_iterations_;

 public:
  LBGreedy(int num_iterations = 20) : num_iterations_(num_iterations) {}

  void run(base::SolverState &state, int graph_id, util::Random &rand) override {
    util::Timer timer;
    auto &g = state.get_graph(graph_id);
    log_info("%s LBGreedy started: n=%lu, num_iterations=%d", state.label(graph_id).c_str(), g.number_of_vertices(), num_iterations_);

    int t = 0;
    for (; t < num_iterations_ && !state.resolved(graph_id); ++t) {
      int lb = run_iteration(g, rand);

      if (state.update_lower_bound(graph_id, lb)) {
        log_debug("%s LBGreedy found new LB: t=%d, lb=%d", state.label(graph_id).c_str(), t, lb);
      }
    }
    log_info("%s LBGreedy finished: t=%d, elapsed=%.2fs", state.label(graph_id).c_str(), t, timer.stop());
  }

 private:
  inline int key(int n, int i, int j) const { return std::min(i, j) * n + std::max(i, j); }

  int run_iteration(ds::graph::TriGraph const &g, util::Random &rand) const {
    int n = g.number_of_vertices();
    int nn = g.number_of_original_vertices();
    auto vs = g.vertices();
    auto vs_inv = util::inverse_map(vs);

    // data structures
    ds::set::FastSet fs(nn), removed(nn);

    std::vector<int> initial_wrp(n * n, n);  // initial weak red potential (compressed indices)
    for (int i = 0; i < n; ++i) {
      for (int j = i + 1; j < n; ++j) {
        int wrp = g.weak_red_potential(vs[i], vs[j]);
        initial_wrp[key(n, i, j)] = wrp;
      }
    }

    ds::queue::BucketQueue q(initial_wrp);

    ds::set::SortedVectorSet active_vs(util::range_to_vec(n));
    int lb = 0;

    while (lb + 2 <= static_cast<int>(active_vs.size())) {
      int wrp = q.min_value();

      // update global status
      lb = std::max(lb, wrp);

      // randomly pick a vertex in the front bucket
      int x = q.get_random_key_with_min_value(rand);
      int v = rand.random() < 0.5 ? x / n : x % n;

      // remove vertex from the graph
      active_vs -= v;
      removed.set(vs[v]);

      // remove entries including v
      for (auto u : active_vs) q.remove(key(n, u, v));

      // decrease delta (V' := V \ {v})
      // (1) N_R(v) * V': v was a red neighbor
      fs.clear();
      for (auto uu : g.red_neighbors(vs[v])) {
        if (removed.get(uu)) continue;  // already removed
        fs.set(uu);                     // mark N_R(v)

        auto u = vs_inv[uu];
        for (auto w : active_vs) {
          if (u == w) continue;
          if (g.has_red_edge(vs[v], vs[w]) && u > w) continue;

          // decrement
          q.update(key(n, u, w), -1);
        }
      }

      // (2) N_B(v) * (V' \ N_R(v) \ N_B(v)): v was an unshared neighbor between u and w
      for (auto uu : g.black_neighbors(vs[v])) {
        if (removed.get(uu)) continue;
        fs.set(uu);
      }
      for (auto uu : g.black_neighbors(vs[v])) {
        if (removed.get(uu)) continue;
        auto u = vs_inv[uu];
        for (auto w : active_vs) {
          if (!fs.get(vs[w])) {
            // decrement
            q.update(key(n, u, w), -1);
          }
        }
      }
    }

    return lb;
  }
};
}  // namespace lowerbound
}  // namespace algorithm
