#pragma once
#include <cassert>

#include "algorithm/base/BaseSolver.hpp"
#include "sat/SATAdapter.hpp"
#include "util/Timer.hpp"
#include "util/logger.hpp"

namespace algorithm {
namespace exact {

class SATSolver : public base::BaseSolver {
 private:
  int time_limit_sec_;
  util::Timer const* root_timer_;
  int n_ = 0;
  sat::SATAdapter solver_;

 public:
  SATSolver(util::Timer const* root_timer = nullptr, int time_limit_sec = 0)
      : time_limit_sec_(time_limit_sec), root_timer_(root_timer) {}

  void run(base::SolverState& state, int graph_id, util::Random& rand) override {
    if (state.resolved(graph_id)) return;  // already resolved

    util::Timer timer(time_limit_sec_, root_timer_);
    auto time_limit_sec = timer.get_effective_time_limit();

    auto& g = state.get_graph(graph_id);
    n_ = g.number_of_vertices();
    auto lb = state.get_lower_bound();  // global lower bound
    auto ub = state.get_upper_bound(graph_id);

    log_info("%s SATSolver started: n=%ld, m=%ld, lb=%d, ub=%d, time_limit=%s", state.label(graph_id).c_str(),
             g.number_of_vertices(), g.number_of_edges(), lb, ub,
             time_limit_sec > 0 ? util::format("%ds", time_limit_sec).c_str() : "N/A");

    bool timed_out = false;
    for (int d = lb; !timed_out && (ub < 0 || d < ub); ++d) {
      log_trace("%s SATSolver checking d=%d", state.label(graph_id).c_str(), d);

      auto result = solve(state, graph_id, d, time_limit_sec);
      if (result == sat::status::SATISFIABLE) {
        break;
      } else if (result == sat::status::INCONSISTENT || result == sat::status::INCONSISTENT_AND_CORE_COMPUTED) {
        lb = d + 1;
        state.update_lower_bound(graph_id, lb);
        log_debug("%s SATSolver found new lower bound: lb=%d", state.label(graph_id).c_str(), lb);
      } else {
        timed_out = true;
        break;
      }
    }

    if (timed_out) {
      log_warning("%s SATSolver timed out: lb=%d, ub=%d, runtime=%.2fs", state.label(graph_id).c_str(), lb, ub, timer.stop());
    } else {
      log_info("%s SATSolver found exact solution: d=%d, runtime=%.2fs", state.label(graph_id).c_str(), lb, timer.stop());
    }
  }

 private:
  typedef std::vector<ds::graph::TriGraph::Vertex> VertexList;
  typedef std::unordered_map<ds::graph::TriGraph::Vertex, ds::graph::TriGraph::Vertex> VertexMap;

  int solve(base::SolverState& state, int graph_id, int d, int time_limit_sec = 0) {
    auto& g = state.get_graph(graph_id);

    // create vertex label maps
    auto vertices = g.vertices();
    auto vertices_inv = util::inverse_map(vertices);

    // construct clauses
    solver_.restart();
    encode_o();
    encode_p();
    encode_a(g, vertices_inv);
    encode_r(g, vertices, vertices_inv, d);
    encode_counter(d);

    // solve
    int ret = solver_.solve(time_limit_sec);
    if (ret == sat::status::SATISFIABLE) {
      // decode solution
      auto xs = decode_o();

      ds::graph::TriGraph::ContractSeq seq;
      for (int i = 0; i < n_ - 1; ++i) {
        int x = xs[i];
        for (int j = x + 1; j < n_; ++j) {
          if (solver_.get_witness(p(x, j))) {
            // `x` gets merged into `j`
            seq.push_back({g.get_label(vertices[j]), g.get_label(vertices[x])});
            break;
          }
        }
      }

      // register solution
      state.update_upper_bound(graph_id, d, seq);
    }
    return ret;
  }

  // SAT variables
  int o(int i, int j) {
    assert(i != j);
    return solver_.id(1000000000L + std::min(i, j) * n_ + std::max(i, j)) * (i < j ? 1 : -1);
  }
  int p(int i, int j) {
    assert(i < j);
    return solver_.id(2000000000L + i * n_ + j);
  }
  int r(int k, int i, int j) {
    assert(k != i && k != j && i != j);
    return solver_.id(3000000000L + (k * n_ + std::min(i, j)) * n_ + std::max(i, j));
  }
  int a(int i, int j) {
    assert(i != j);
    return solver_.id(4000000000L + std::min(i, j) * n_ + std::max(i, j));
  }

  // encoders
  void encode_o() {
    // (1) Transitivity: o(i,j) & o(j,k) => o(i,k)
    for (int i = 0; i < n_; ++i) {
      for (int j = 0; j < n_; ++j) {
        if (i == j) continue;
        for (int k = 0; k < n_; ++k) {
          if (i == k || j == k) continue;
          solver_.add_clause({-o(i, j), -o(j, k), o(i, k)});
        }
      }
    }
  }

  void encode_p() {
    // (2a + 2b) at least and at most one parent except the root
    for (int i = 0; i < n_ - 1; ++i) {
      std::vector<int> lits;
      for (int j = i + 1; j < n_; ++j) lits.push_back(p(i, j));
      solver_.add_equals_one(lits);
    }

    // (2c) ordering condition: p(i,j) => o(i,j)
    for (int i = 0; i < n_ - 1; ++i) {
      for (int j = i + 1; j < n_; ++j) solver_.add_clause({-p(i, j), o(i, j)});
    }
  }

  void encode_a(ds::graph::TriGraph const& graph, VertexMap const& vertices_inv) {
    // (3a) semantics of a
    for (int i = 0; i < n_ - 1; ++i) {
      for (int j = i + 1; j < n_; ++j) {
        for (int k = 0; k < n_; ++k) {
          if (i == k || j == k) continue;
          solver_.add_clause({-o(k, i), -o(k, j), -r(k, i, j), a(i, j)});
        }
      }
    }

    // (4a) initial red edges
    for (auto& e : graph.red_edges()) solver_.add_clause({a(vertices_inv.at(e.first), vertices_inv.at(e.second))});
  }

  void encode_r(ds::graph::TriGraph const& graph, VertexList const& vertices, VertexMap const& vertices_inv, int d) {
    int const max_diff = 3;

    // (3b) semantics of red edges
    for (int i = 0; i < n_ - 1; ++i) {
      auto ii = vertices[i];
      for (int j = i + 1; j < n_; ++j) {
        auto jj = vertices[j];
        for (auto kk : graph.get_black_symmetric_difference(ii, jj)) {
          auto k = vertices_inv.at(kk);
          assert(i != k && j != k);
          solver_.add_clause({-p(i, j), -o(i, k), r(i, j, k)});
        }

        // extra hints
        for (int diff = 1; diff <= max_diff; ++diff) {
          if (graph.weak_red_potential(ii, jj) >= d + diff) {
            // (j,i) cannot be int the first `diff` contraction pair
            std::vector<int> lits;
            for (int k = 0; k < n_; ++k) {
              if (i != k && j != k) lits.push_back(o(k, i));
            }
            solver_.add_atleast(lits, diff, -1, {-p(i, j)});
          }
        }
      }
    }

    // (3c) transfer red edges
    for (int i = 0; i < n_ - 1; ++i) {
      for (int j = i + 1; j < n_; ++j) {
        for (int k = 0; k < n_; ++k) {
          if (i == k || j == k) continue;
          solver_.add_clause({-p(i, j), -o(i, k), -a(i, k), r(i, j, k)});
        }
      }
    }

    // (3d) maintain red edges
    for (int i = 0; i < n_; ++i) {
      for (int j = 0; j < n_; ++j) {
        if (i == j) continue;
        for (int k = 0; k < n_ - 1; ++k) {
          if (i == k || j == k) continue;
          for (int m = k + 1; m < n_; ++m) {
            if (i == m || j == m) continue;
            solver_.add_clause({-o(i, j), -o(j, k), -o(j, m), -r(i, k, m), r(j, k, m)});
          }
        }
      }
    }

    // (4b) present red edges
    // for every initial red edge xy
    //   for every i
    //     o(i, x) and o(i, y) -> r(i, x, y)
    for (auto& e : graph.red_edges()) {
      auto x = vertices_inv.at(e.first);
      auto y = vertices_inv.at(e.second);
      for (int i = 0; i < n_; ++i) {
        if (i == x || i == y) continue;
        solver_.add_clause({-o(i, x), -o(i, y), r(i, x, y)});
      }
    }
  }

  void encode_counter(int ubound) {
    for (int i = 0; i < n_ - 1; ++i) {
      for (int x = 0; x < n_; ++x) {
        if (i == x) continue;
        std::vector<int> lits;
        for (int y = 0; y < n_; ++y) {
          if (x != y && i != y) lits.push_back(r(i, x, y));
        }
        solver_.add_atmost(lits, ubound);
      }
    }
  }

  // decoder
  std::vector<int> decode_o() {
    std::vector<int> xs;
    for (int i = 0; i < n_; ++i) xs.push_back(i);
    auto compare = [&](int i, int j) { return solver_.get_witness(o(i, j)); };
    std::sort(xs.begin(), xs.end(), compare);
    return xs;
  }
};
}  // namespace exact
}  // namespace algorithm
