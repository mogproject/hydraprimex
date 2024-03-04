#pragma once
#include <cassert>

#include "ds/graph/TriGraph.hpp"
#include "ds/set/FastSet.hpp"
#include "sat/SATAdapter.hpp"
#include "util/Random.hpp"
#include "util/Timer.hpp"
#include "util/logger.hpp"

namespace algorithm {
namespace exact {

class SATSolver {
 private:
  ds::graph::TriGraph const& graph_;
  std::vector<ds::graph::TriGraph::Vertex> vertices_;
  std::unordered_map<ds::graph::TriGraph::Vertex, ds::graph::TriGraph::Vertex> vertices_inv_;
  int n_;
  sat::SATAdapter solver_;
  int lower_bound_;
  int upper_bound_;
  ds::graph::TriGraph::ContractSeq seq_;

 public:
  SATSolver(ds::graph::TriGraph const& graph)
      : graph_(graph),
        vertices_(graph.vertices()),
        vertices_inv_(util::inverse_map(vertices_)),
        n_(graph.number_of_vertices()),
        lower_bound_(0),
        upper_bound_(-1) {}

  int lower_bound() const { return lower_bound_; }

  int upper_bound() const { return upper_bound_; }

  int twin_width() const {
    assert(lower_bound_ == upper_bound_);
    return upper_bound_;
  }

  ds::graph::TriGraph::ContractSeq const& contraction_sequence() { return seq_; }

  bool run(int lower_bound = 0, int upper_bound = -1, int time_limit_sec = 0) {
    assert(lower_bound <= static_cast<int>(graph_.number_of_vertices()) - 1);
    assert(static_cast<int>(vertices_.size()) == n_);

    log_info("SATSolver started: n=%ld, m=%ld, lb=%d, ub=%d, time_limit=%s", graph_.number_of_vertices(), graph_.number_of_edges(),
             lower_bound, upper_bound, time_limit_sec > 0 ? util::format("%ds", time_limit_sec).c_str() : "N/A");

    util::Timer timer;
    lower_bound_ = lower_bound;
    upper_bound_ = upper_bound;

    bool timed_out = false;
    for (int d = lower_bound_; !timed_out && (upper_bound_ < 0 || d < upper_bound_); ++d) {
      log_trace("SATSolver checking d=%d", d);

      auto result = solve(d, time_limit_sec);
      if (result == sat::status::SATISFIABLE) {
        break;
      } else if (result == sat::status::INCONSISTENT || result == sat::status::INCONSISTENT_AND_CORE_COMPUTED) {
        lower_bound_ = d + 1;
        log_debug("SATSolver found new LB: lb=%d", lower_bound_);
      } else {
        timed_out = true;
        break;
      }
    }

    if (timed_out) {
      log_warning("SATSolver timed out: lb=%d, ub=%d, runtime=%.2fs", lower_bound_, upper_bound_, timer.stop());
    } else {
      log_info("SATSolver found exact solution: d=%d, runtime=%.2fs", upper_bound_, timer.stop());
    }
    return !timed_out;
  }

 private:
  int solve(int d, int time_limit_sec = 0) {
    // construct clauses
    solver_.restart();
    encode_o();
    encode_p();
    encode_a();
    encode_r(d);
    encode_counter(d);

    // solve
    int ret = solver_.solve(time_limit_sec);
    if (ret == sat::status::SATISFIABLE) {
      upper_bound_ = d;

      // decode solution
      auto xs = decode_o();

      seq_.clear();
      for (int i = 0; i < n_ - 1; ++i) {
        int x = xs[i];
        for (int j = x + 1; j < n_; ++j) {
          if (solver_.get_witness(p(x, j))) {
            // seq_.push_back({graph_.get_label(j), graph_.get_label(x)});  // `x` gets merged into `j`
            seq_.push_back({vertices_[j], vertices_[x]});  // `x` gets merged into `j`
            break;
          }
        }
      }
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

  void encode_a() {
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
    for (auto& e : graph_.red_edges()) solver_.add_clause({a(vertices_inv_[e.first], vertices_inv_[e.second])});
  }

  void encode_r(int d) {
    int const max_diff = 3;

    // (3b) semantics of red edges
    for (int i = 0; i < n_ - 1; ++i) {
      auto ii = vertices_[i];
      for (int j = i + 1; j < n_; ++j) {
        auto jj = vertices_[j];
        for (auto kk : graph_.get_black_symmetric_difference(ii, jj)) {
          auto k = vertices_inv_[kk];
          assert(i != k && j != k);
          solver_.add_clause({-p(i, j), -o(i, k), r(i, j, k)});
        }

        // extra hints
        for (int diff = 1; diff <= max_diff; ++diff) {
          if (graph_.weak_red_potential(ii, jj) >= d + diff) {
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
    for (auto& e : graph_.red_edges()) {
      auto x = vertices_inv_[e.first];
      auto y = vertices_inv_[e.second];
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
