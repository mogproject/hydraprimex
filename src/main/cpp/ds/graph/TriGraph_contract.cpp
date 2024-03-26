#include "TriGraph.hpp"

namespace ds {
namespace graph {

//============================================================================
//    Contractions
//============================================================================

/**
 * @brief Contracts vertex i into j.
 *
 * @param j vertex to merge
 * @param i vertex to be merged
 * @return int maximum red degree in the closed neighborhood of j after contraction
 */
int TriGraph::contract(Vertex j, Vertex i, GraphLog* graph_log) {
  assert(has_vertex(i));
  assert(has_vertex(j));
  assert(i != j);

  // initialize update list
  EdgeList updated;

  // categorize the neighborhood of i and j
  bool edge_ij = has_edge(i, j);
  int edge_ij_red = has_red_edge(i, j) ? 1 : 0;

  auto i_adj_r = adj_red_[i] - j;            // a2 + c3 + c4
  auto i_adj_b = adj_black_[i] - j;          // a1 + c1 + c2
  auto j_adj_r = adj_red_[j] - i;            // c2 + c4 + b2
  auto j_adj_b = adj_black_[j] - i;          // c1 + c3 + b1
  auto i_adj = adj_black_[i] | adj_red_[i];  // a1 + a2 + c1 + c2 + c3 + c4
  auto j_adj = adj_black_[j] | adj_red_[j];  // c1 + c2 + c3 + c4 + b1 + b2
  auto common_nbrs = i_adj & j_adj;          // c1 + c2 + c3 +c4

  auto a1 = i_adj_b - j_adj;
  auto a2 = i_adj_r - j_adj;
  auto b1 = j_adj_b - i_adj;
  auto b2 = j_adj_r - i_adj;
  auto c1 = i_adj_b & j_adj_b;
  auto c2 = i_adj_b & j_adj_r;
  auto c3 = i_adj_r & j_adj_b;
  auto c4 = i_adj_r & j_adj_r;

  //--------------------------------------------------------------------------
  // I. Update mu values
  //--------------------------------------------------------------------------
  std::vector<std::pair<Edge, int>> mu_delta;

  // clang-format off
    // (1) A1 x (A1 | C1 | C2): -1
    for (auto x : a1) for (auto y : a1) if (x < y) mu_delta.push_back({{x, y}, -1});
    for (auto x : a1) for (auto y : c1 | c2) mu_delta.push_back({{x, y}, -1});

    // (2) B1 x (C1 | C3 | B1): -1
    for (auto x : b1) for (auto y : b1) if (x < y) mu_delta.push_back({{x, y}, -1});
    for (auto x : b1) for (auto y : c1 | c3) mu_delta.push_back({{x, y}, -1});

    // (3) C2 x C3: -1
    for (auto x : c2) for (auto y : c3) mu_delta.push_back({{x, y}, -1});

    // (4) C4 x (C1 | C2 | C3 | C4): -1
    for (auto x : c4) for (auto y : c4) if (x < y) mu_delta.push_back({{x, y}, -1});
    for (auto x : c4) for (auto y : c1 | c2 | c3) mu_delta.push_back({{x, y}, -1});

    // (5) C2 x C2: -1
    for (auto x : c2) for (auto y : c2) if (x < y) mu_delta.push_back({{x, y}, -2});

    // (6) C1 x (C1 | C2 | C3): -2
    for (auto x : c1) for (auto y : c1) if (x < y) mu_delta.push_back({{x, y}, -2});
    for (auto x : c1) for (auto y : c2 | c3) mu_delta.push_back({{x, y}, -2});

    // (7) C3 x C3: -2
    for (auto x : c3) for (auto y : c3) if (x < y) mu_delta.push_back({{x, y}, -2});

    // (8) (A1 | A2) x (B1 | B2): +1
    for (auto x : a1 | a2) for (auto y : b1 | b2) mu_delta.push_back({{x, y}, 1});
  // clang-format on

  std::unordered_map<Vertex, int> mu_delta_j;
  if (has_edge(i, j)) {
    // (9) j x (A1 | C1 | C2): -1 or -2
    for (auto x : a1 | c1 | c2) mu_delta_j[x] -= has_red_edge(i, j) ? 1 : 2;

    // (10) j x (A2 | C3 | C4): -1
    for (auto x : a2 | c3 | c4) mu_delta_j[x] -= 1;
  }

  // (11) j x N(A1 | A2): +1    new red edge between j and x <- (A1 | A2)
  for (auto x : a1 | a2) {
    for (auto y : adj_black_[x] | adj_red_[x]) {
      if (y != i) mu_delta_j[y] += 1;
    }
  }

  // (12) j x N_B(C3 | B1): -1    new red edge between j and x <- (C3 | B1)
  for (auto x : c3 | b1) {
    for (auto y : adj_black_[x]) {
      if (j != y) {
        assert(y != i);
        mu_delta_j[y] -= 1;
      }
    }
  }

  for (auto& p : mu_delta_j) {
    if (p.second != 0) mu_delta.push_back({{j, p.first}, p.second});
  }

  //--------------------------------------------------------------------------
  // II. Save graph log
  //--------------------------------------------------------------------------
  if (graph_log) {
    // track changes in weak red potential
    std::unordered_map<int, int> wrp_delta;
    ds::set::FastSet common_nbrs_set(n_orig_);
    for (auto x : common_nbrs) common_nbrs_set.set(x);

    for (auto& p : mu_delta) wrp_delta[key(n_orig_, p.first.first, p.first.second)] = -p.second;

    // apply changes in degrees
    int j_deg_delta = a1.size() + a2.size() - edge_ij;
    for (auto x : vertices_) {  // j x V
      if (x != i && x != j) wrp_delta[key(n_orig_, j, x)] += j_deg_delta;
    }

    for (auto x : common_nbrs) {
      for (auto y : vertices_) {
        if (y == i) continue;
        if (common_nbrs_set.get(y)) {  // C x C
          if (x < y) wrp_delta[key(n_orig_, x, y)] -= 2;
        } else {  // C x (V-C)  [including C x j]
          wrp_delta[key(n_orig_, x, y)] -= 1;
        }
      }
    }

    // apply changes in new edges
    for (auto x : a1 | a2) wrp_delta[key(n_orig_, j, x)] -= 2;

    // collect the set of vertices whose weak red potential has decreased
    std::vector<std::pair<int, int>> potential_decreased;
    for (auto& p : wrp_delta) {
      if (p.second < 0) potential_decreased.push_back({p.first / n_orig_, p.first % n_orig_});
    }

    // save values
    graph_log->log_type = GraphLogType::CONTRACTION;
    graph_log->merge = j;
    graph_log->merged = i;
    graph_log->removed_black = adj_black_[i].to_vector();
    graph_log->removed_red = adj_red_[i].to_vector();
    graph_log->new_neighbors = (a1 | a2).to_vector();
    graph_log->recolored = (b1 | c3).to_vector();
    graph_log->mu_delta = mu_delta;
    graph_log->potential_decreased = potential_decreased;
  }

  //--------------------------------------------------------------------------
  // III. Make changes
  //--------------------------------------------------------------------------
  // remove edge ij if exists
  if (edge_ij) {
    if (edge_ij_red) {
      remove_red_edge(i, j);
    } else {
      remove_black_edge(i, j);
    }
  }

  // recolor {j, w | w <- N_B(j) - N_B(i)}
  for (int w : b1 | c3) {
    remove_black_edge(j, w);
    add_red_edge(j, w);
  }

  // add red edges {j, w| w <- N(i) - N(j)}
  for (int w : a1 | a2) add_red_edge(j, w);

  // remove vertex i
  remove_vertex(i);

  // update mu values
  for (auto& p : mu_delta) {
    auto u = p.first.first;
    auto v = p.first.second;
    mu_[u][v] += p.second;
    mu_[v][u] += p.second;
  }

  //--------------------------------------------------------------------------
  // IV. Return max red degree in the neighborhood
  //--------------------------------------------------------------------------
  int ret = red_degree(j);
  for (auto w : adj_black_[j] | adj_red_[j]) ret = std::max(ret, red_degree(w));
  return ret;
}

void TriGraph::undo(GraphLog const& graph_log) {
  switch (graph_log.log_type) {
    case GraphLogType::COMPLEMENT: {
      black_complement();
      break;
    }
    case GraphLogType::CONTRACTION: {
      int i = graph_log.merged;
      int j = graph_log.merge;

      add_vertex(i);

      // add edges
      for (auto w : graph_log.removed_black) add_black_edge(i, w);
      for (auto w : graph_log.removed_red) add_red_edge(i, w);

      // remove edges
      for (auto w : graph_log.new_neighbors) remove_edge(j, w);

      // recolor edges
      for (auto w : graph_log.recolored) {
        remove_red_edge(j, w);
        add_black_edge(j, w);
      }

      // revert mu values
      for (auto& p : graph_log.mu_delta) {
        auto u = p.first.first;
        auto v = p.first.second;
        mu_[u][v] -= p.second;
        mu_[v][u] -= p.second;
      }
      break;
    }
    default: {
      throw std::invalid_argument("never happens");
    }
  }
}

/**
 * @brief Finds all vertex pairs whose weak red potential is at most the given threshold.
 *
 * @param upper_bound threshold value
 * @param frozen_vertices vertices excluded from candidates
 * @return std::vector<std::pair<Vertex, Vertex>> contraction candidates
 */
std::vector<std::pair<TriGraph::Vertex, TriGraph::Vertex>> TriGraph::find_candidates(  //
    int upper_bound,                                                                   //
    TriGraph::VertexList const& frozen_vertices                                        //
) const {
  static ds::set::FastSet frozen;
  frozen.initialize(n_orig_);
  for (auto x : frozen_vertices) frozen.set(x);

  VertexList vs;
  for (auto x : vertices_) {
    if (!frozen.get(x)) vs.push_back(x);
  }

  std::vector<std::pair<Vertex, Vertex>> ret;

  int n = vs.size();
  for (int i = 0; i < n; ++i) {
    auto u = vs[i];
    for (int j = i + 1; j < n; ++j) {
      auto v = vs[j];
      if (weak_red_potential(u, v) <= upper_bound) ret.push_back({u, v});
    }
  }
  return ret;
}

/**
 * @brief Updates contraction candidates after a contraction.
 *
 * @param previous_candidates previous candidates
 * @param graph_log GraphLog instance of the previous contraction
 * @param upper_bound threshould value
 * @param frozen_vertices vertices excluded from candidates
 * @return std::vector<std::pair<Vertex, Vertex>> contraction candidates
 */
std::vector<std::pair<TriGraph::Vertex, TriGraph::Vertex>> TriGraph::update_candidates(     //
    std::vector<std::pair<TriGraph::Vertex, TriGraph::Vertex>> const& previous_candidates,  //
    GraphLog const& graph_log,                                                              //
    int upper_bound,                                                                        //
    TriGraph::VertexList const& frozen_vertices                                             //
) const {
  static ds::set::FastSet frozen;
  static ds::set::FastSet seen;

  frozen.initialize(n_orig_);
  seen.initialize(n_orig_ * n_orig_);

  for (auto x : frozen_vertices) frozen.set(x);

  std::vector<std::pair<Vertex, Vertex>> ret;
  for (auto& p : graph_log.potential_decreased) {
    auto u = p.first;
    auto v = p.second;
    if (frozen.get(u) || frozen.get(v)) continue;
    if (weak_red_potential(u, v) > upper_bound) continue;
    ret.push_back({u, v});
    seen.set(key(n_orig_, u, v));
  }

  for (auto& p : previous_candidates) {
    auto u = p.first;
    auto v = p.second;
    assert(!frozen.get(u) && !frozen.get(v));
    if (u == graph_log.merged || v == graph_log.merged) continue;
    if (seen.get(key(n_orig_, u, v))) continue;
    if (weak_red_potential(u, v) > upper_bound) continue;
    ret.push_back({u, v});
  }

  return ret;
}

//============================================================================
//    Contraction sequence verification
//============================================================================
int TriGraph::verify_contraction_sequence(TriGraph const& graph, ContractSeq const& seq) {
  auto n = graph.number_of_vertices();
  if (n == 0) {
    if (!seq.empty()) throw std::runtime_error("seq must be empty for empty graphs");
  }
  if (seq.size() + 1 != n) throw std::runtime_error("seq size mismatch");

  auto g = graph;  // create a copy
  int red_deg = g.max_red_degree();

  for (auto& p : seq) {
    auto j = g.get_index(p.first);
    auto i = g.get_index(p.second);
    if (!g.has_vertex(i) || !g.has_vertex(j)) throw std::runtime_error("vertex does not exist");
    red_deg = std::max(red_deg, g.contract(j, i));
  }
  return red_deg;
}

//============================================================================
//    Reduction Rules
//============================================================================
/**
 * @brief Checks if a contraction is safe.
 *
 * @param i one of the contraction pair
 * @param j one of the contraction pair
 *
 * @return true safe to contract
 * @return false not safe to contract
 */
bool TriGraph::is_free_contraction(Vertex i, Vertex j) const {
  assert(has_vertex(i) && has_vertex((j)));

  // (1) They must be black twins.
  if ((adj_black_[i] - j) != (adj_black_[j] - i)) return false;

  // (2) Red neighbors have subset relation.
  if (adj_red_[i].size() > adj_red_[j].size()) std::swap(i, j);
  return (adj_red_[i] - j - adj_red_[j]).empty();
}

}  // namespace graph
}  // namespace ds
