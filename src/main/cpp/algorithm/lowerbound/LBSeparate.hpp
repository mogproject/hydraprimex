#pragma once

#include "algorithm/base/BaseSolver.hpp"
#include "algorithm/exact/BranchSolver.hpp"
#include "algorithm/exact/SATSolver.hpp"
#include "algorithm/lowerbound/LBGreedy.hpp"
#include "algorithm/preprocess/VertexSeparator.hpp"
#include "algorithm/upperbound/GreedySolver.hpp"

namespace algorithm {
namespace lowerbound {

class LBSeparate : public base::BaseSolver {
 private:
  typedef ds::graph::TriGraph::Vertex Vertex;
  int num_iterations_;
  int search_level_;

 public:
  LBSeparate(int num_iterations = 20, int search_level = 0)
      : num_iterations_(num_iterations), search_level_(search_level) {}

  void run(base::SolverState &state, int graph_id, util::Random &rand) override {
    util::Timer timer;
    auto &g = state.get_graph(graph_id);
    log_info("%s LBSeparate started: n=%lu, num_iterations=%d, search_level=%d", state.label(graph_id).c_str(),
             g.number_of_vertices(), num_iterations_, search_level_);

    auto h = g.subgraph(find_largest_component(g));
    assert(h.is_connected());

    int t = 0;
    for (; t < num_iterations_ && !state.resolved(graph_id); ++t) {
      int lb = run_iteration(h, rand, t, state.get_lower_bound());

      if (state.update_lower_bound(graph_id, lb)) {
        log_info("%s LBSeparate found new LB: t=%d, lb=%d", state.label(graph_id).c_str(), t, lb);
      }
    }
    log_info("%s LBSeparate finished: t=%d, elapsed=%.2fs", state.label(graph_id).c_str(), t, timer.stop());
  }

  int run_iteration(ds::graph::TriGraph const &graph, util::Random &rand, int iteration_id, int known_lb) {
    auto g = graph;
    std::vector<Vertex> vertices = g.vertices();
    std::vector<Vertex> frozen_vertices;

    while (true) {
      bool last_loop = false;

      // separate and induce on the largest component
      bool separated = separate(g, rand, vertices, frozen_vertices);
      if (!separated) {
        // sample
        vertices = sample_degree_weighted(g, get_number_of_samples(known_lb), rand);
        last_loop = true;
      }

      // induce on chosen vertices and convert indices of frozen vertices
      std::vector<ds::graph::TriGraph::VertexLabel> frozen_labels;

      if (separated) {
        for (auto v : frozen_vertices) frozen_labels.push_back(g.get_label(v));
      }

      // log_trace("before induce: edges: %s", cstr(g.edges()));
      // log_trace("all vertices: %s", cstr(util::sorted(g.vertices())));
      // log_trace("chosen vertices: %s", cstr(util::sorted(vertices)));

      g = g.subgraph(vertices, true);
      assert(g.is_connected());

      frozen_vertices.clear();
      if (separated) {
        for (auto v : frozen_labels) frozen_vertices.push_back(g.get_index(v));
      }

      // disable logging
      auto prev_log_level = util::logging::log_level;
      util::set_log_level(util::logging::LogLevel::CRITICAL);

      base::SolverState state({g, known_lb, -1});
      int graph_id = 0;
      compute_lower_bound(state, rand, search_level_);

      // enable logging
      util::set_log_level(prev_log_level);

      if (last_loop || state.resolved(graph_id)) return state.get_lower_bound();
    }
    throw std::runtime_error("never happens");
  }

  int get_number_of_samples(int known_lb) const {
    int ret = search_level_ >= 2 ? 60 : search_level_ >= 1 ? 30 : 23;
    return std::max(known_lb * 2 + 4, ret);
  }

  bool separate(ds::graph::TriGraph const &g,      //
                util::Random &rand,                //
                std::vector<int> &vertices,        // may be updated
                std::vector<int> &frozen_vertices  // may be updated
  ) {
    int n = g.number_of_vertices();
    std::vector<Vertex> s;
    algorithm::preprocess::VertexSeparator sep;

    assert(g.is_connected());

    for (                                                                                      //
        int sep_size = 1, not_found_count = 0;                                                 //
        s.empty() && sep_size < n - 1 && not_found_count < 5;                                  //
        sep_size = std::min(sep_size * 2, sep_size + std::max(1, n / 100)), ++not_found_count  //
    ) {
      s = sep.find_vertex_separator(g, sep_size, nullptr, nullptr, {}, {}, &rand, false, false, frozen_vertices);
      log_trace("LBSeparate: sep_size=%d, not_found_count=%d s=%s", sep_size, not_found_count, cstr(s));
    }
    if (s.empty()) return false;  // separator not found

    auto h = g;
    for (auto x : s) h.remove_vertex(x);  // H = G - S
    vertices = find_largest_component(h);
    for (auto x : s) vertices.push_back(x);

    static ds::set::FastSet fs;
    fs.initialize(g.number_of_original_vertices());
    for (auto v : vertices) fs.set(v);

    auto spike = find_spike(g, s, fs);
    assert(spike >= 0);
    vertices.push_back(spike);
    fs.set(spike);

    // update frozen vertices
    std::vector<Vertex> next_frozen;
    for (auto x : frozen_vertices) {
      if (fs.get(x)) next_frozen.push_back(x);
    }

    // add spike and separator to the frozen vertices
    if (fs.get(spike)) next_frozen.push_back(spike);
    for (auto x : s) {
      if (fs.get(x)) next_frozen.push_back(x);
    }
    frozen_vertices = next_frozen;
    return true;
  }

  /**
   * @brief Picks one vertex (spike) adjacent to the separator from outside if one exists
   *
   * @param g graph
   * @param s separator
   * @param fs FastSet instance storing active vertices
   * @return Vertex spike vertex or not found (-1)
   */
  Vertex find_spike(ds::graph::TriGraph const &g, std::vector<Vertex> const &s, ds::set::FastSet const &fs) const {
    for (auto u : s) {
      for (auto v : g.neighbors(u)) {
        if (!fs.get(v)) return v;
      }
    }
    return -1;
  }

  std::vector<ds::graph::TriGraph::Vertex> find_largest_component(ds::graph::TriGraph const &g) {
    auto ccs = g.connected_components();
    int ret = 0;
    for (std::size_t i = 1; i < ccs.size(); ++i) {
      if (ccs[i].size() > ccs[ret].size()) ret = i;
    }
    return ccs[ret];
  }

  void compute_lower_bound(base::SolverState &state, util::Random &rand, int level) {
    int graph_id = 0;
    auto &g = state.get_graph(0);
    auto n = g.number_of_vertices();

    // trivial cases
    int known_lb = state.get_lower_bound();
    if (known_lb >= 4 && n <= 10) return;
    if (known_lb >= 3 && n <= 8) return;
    if (known_lb >= 2 && n <= 7) return;

    std::vector<std::unique_ptr<base::BaseSolver>> solvers;
    solvers.push_back(std::make_unique<upperbound::GreedySolver>(10, 10, nullptr, 6, 0));
    solvers.push_back(std::make_unique<lowerbound::LBGreedy>(10));

    if (search_level_ == 0) {
      solvers.push_back(std::make_unique<exact::BranchSolver>(1, 1000000L, 27));
      // solvers.push_back(std::make_unique<exact::BranchSolver>(1, 1000000L, 27));
      solvers.push_back(std::make_unique<exact::BranchSolver>(2, 1000000L, 30));
      solvers.push_back(std::make_unique<exact::SATSolver>(nullptr, 1, 22));
    } else if (search_level_ == 1) {
      solvers.push_back(std::make_unique<exact::BranchSolver>(3, 10000000L, 36));
      // solvers.push_back(std::make_unique<exact::BranchSolver>(3, 10000000L, 36));
      // solvers.push_back(std::make_unique<exact::BranchSolver>(6, 10000000L, 36));
      solvers.push_back(std::make_unique<exact::SATSolver>(nullptr, 10, 27));
    } else {
      solvers.push_back(std::make_unique<exact::BranchSolver>(120, 1000000000L, 123));
    }

    // run solvers
    for (std::size_t i = 0; i < solvers.size() && !state.resolved(graph_id); ++i) {
      solvers[i]->run(state, graph_id, rand);
    }
    return;
  }

  std::vector<Vertex> sample_degree_weighted(ds::graph::TriGraph const &graph, int sample_size, util::Random &rand) {
    int n = graph.number_of_vertices();
    auto vertices = graph.vertices();
    if (n <= sample_size) return vertices;

    std::vector<Vertex> vs, q;
    int nn = graph.number_of_original_vertices();
    std::vector<int> degs, all_degs(nn, 0);
    static ds::set::FastSet visited;
    visited.initialize(nn);

    for (auto v : vertices) all_degs[v] = graph.degree(v);

    // pick the first vertex
    int start_v = vertices[rand.weighted_choice(all_degs)];
    vs.push_back(start_v);
    degs.push_back(all_degs[start_v]);
    q.push_back(start_v);
    visited.set(start_v);

    for (int cursor = 0; cursor < sample_size && cursor < static_cast<int>(vs.size()); ++cursor) {
      // pick next destination
      int i = rand.weighted_choice(degs.begin() + cursor, degs.end()) + cursor;
      std::swap(vs[cursor], vs[i]);
      std::swap(degs[cursor], degs[i]);
      int u = vs[cursor];

      // enqueue u's unseen neighbors
      for (auto v : graph.neighbors(u)) {
        if (!visited.get(v)) {
          visited.set(v);
          vs.push_back(v);
          degs.push_back(all_degs[v]);
        }
      }
    }

    // trim vertices
    assert(sample_size <= static_cast<int>(vs.size()));
    vs.resize(sample_size);

    return vs;
  }
};
}  // namespace lowerbound
}  // namespace algorithm
