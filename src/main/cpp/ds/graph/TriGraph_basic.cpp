#include "TriGraph.hpp"

namespace ds {
namespace graph {
//============================================================================
//    Vertex and Edge Enumeration
//============================================================================
/**
 * @brief Returns a sorted list of all vertices.
 *
 * @return VertexList all vertices
 */
TriGraph::VertexList TriGraph::vertices() const { return vertices_.to_vector(); }

/**
 * @brief Returns a list of all edges.
 *
 * @param sorted sort edges if true
 * @return ColoredEdgeList all edges
 */
TriGraph::ColoredEdgeList TriGraph::edges(bool sorted) const {
  ColoredEdgeList ret;
  for (int i : vertices_) {
    for (auto j : adj_black_[i]) {
      if (i < j) ret.push_back(std::make_pair(std::make_pair(i, j), Black));
    }
    for (auto j : adj_red_[i]) {
      if (i < j) ret.push_back(std::make_pair(std::make_pair(i, j), Red));
    }
  }

  if (sorted) std::sort(ret.begin(), ret.end());
  assert(static_cast<int>(ret.size()) == num_black_edges_ + num_red_edges_);
  return ret;
}

TriGraph::EdgeList TriGraph::get_edges(Color color) const {
  auto& adj = color == Black ? adj_black_ : adj_red_;
  std::vector<Edge> ret;
  for (auto i : vertices_) {
    for (auto j : adj[i]) {
      if (i < j) ret.push_back(std::make_pair(i, j));
    }
  }

  assert(static_cast<int>(ret.size()) == (color == Black ? num_black_edges_ : num_red_edges_));
  return ret;
}

/**
 * @brief Returns a list of black edges.
 *
 * @return EdgeList black edge list
 * @note result will be sorted
 */
TriGraph::EdgeList TriGraph::black_edges() const { return get_edges(Black); }

/**
 * @brief Returns a list of red edges.
 *
 * @return EdgeList red edge list
 * @note result will be sorted
 */
TriGraph::EdgeList TriGraph::red_edges() const { return get_edges(Red); }

TriGraph::Vertex TriGraph::get_label(Vertex i) const {
  assert(0 <= i && i < n_orig_);
  return vertex_labels_[i];
}

std::unordered_map<TriGraph::Vertex, TriGraph::Vertex> TriGraph::get_label_map() const {
  std::unordered_map<Vertex, Vertex> ret;
  for (int i = 0; i < n_orig_; ++i) ret[vertex_labels_[i]] = i;
  return ret;
}

//============================================================================
//    Vertex and Edge Modifications
//============================================================================
/**
 * @brief Checks if the graph has the given vertex.
 *
 * @param i vertex label
 *
 * @return true vertex i exists
 * @return false vertex i does not exist
 */
bool TriGraph::has_vertex(Vertex i) const {
  assert(0 <= i && i < n_orig_);  // vertex out of range
  return vertices_.get(i);
}

/**
 * @brief Adds a new vertex to the graph.
 *
 * @param i vertex label
 */
void TriGraph::add_vertex(Vertex i) {
  assert(0 <= i && i < n_orig_);  // vertex out of range
  assert(!vertices_.get(i));      // vertex already exists

  vertices_.set(i);
  hash_ ^= vertex_hash(i);
}

void TriGraph::remove_vertex(Vertex i) {
  assert(0 <= i && i < n_orig_);  // vertex out of range
  assert(vertices_.get(i));       // vertex does not exist

  for (int w : adj_black_[i].to_vector()) remove_black_edge(i, w);
  for (int w : adj_red_[i].to_vector()) remove_red_edge(i, w);
  vertices_.reset(i);
  hash_ ^= vertex_hash(i);
}

bool TriGraph::has_black_edge(Vertex i, Vertex j) const {
  assert(0 <= i && i < n_orig_);  // vertex out of range
  assert(0 <= j && j < n_orig_);  // vertex out of range

  return adj_black_[i].get(j);
}

bool TriGraph::has_red_edge(Vertex i, Vertex j) const {
  assert(0 <= i && i < n_orig_);  // vertex out of range
  assert(0 <= j && j < n_orig_);  // vertex out of range

  return adj_red_[i].get(j);
}

bool TriGraph::has_edge(Vertex i, Vertex j) const { return has_black_edge(i, j) || has_red_edge(i, j); }

/**
 * @brief Adds a new black  edge to the graph.
 *
 * @param i endpoint i
 * @param j endpoint j
 */
void TriGraph::add_black_edge(Vertex i, Vertex j) {
  assert(!has_edge(i, j));

  adj_black_[i].set(j);
  adj_black_[j].set(i);
  ++num_black_edges_;
}

void TriGraph::add_red_edge(Vertex i, Vertex j) {
  assert(!has_edge(i, j));

  adj_red_[i].set(j);
  adj_red_[j].set(i);
  ++num_red_edges_;
  hash_ ^= red_edge_hash(i, j);
}

void TriGraph::remove_edge(Vertex i, Vertex j) {
  assert(has_edge(i, j));

  if (has_black_edge(i, j)) {
    remove_black_edge(i, j);
  } else {
    remove_red_edge(i, j);
  }
}

void TriGraph::remove_black_edge(Vertex i, Vertex j) {
  assert(has_black_edge(i, j));

  adj_black_[i].reset(j);
  adj_black_[j].reset(i);
  --num_black_edges_;
}

void TriGraph::remove_red_edge(Vertex i, Vertex j) {
  assert(has_red_edge(i, j));

  adj_red_[i].reset(j);
  adj_red_[j].reset(i);
  --num_red_edges_;
  hash_ ^= red_edge_hash(i, j);
}

//============================================================================
//    Neighbors
//============================================================================
std::vector<TriGraph::Vertex> TriGraph::neighbors(Vertex i) const {
  assert(0 <= i && i < n_orig_);  // vertex out of range

  auto ret = adj_black_[i].to_vector();
  util::extend(ret, adj_red_[i].to_vector());
  return ret;
}

TriGraph::AdjSet const& TriGraph::red_neighbors(Vertex i) const { return adj_red_[i]; }
TriGraph::AdjSet const& TriGraph::black_neighbors(Vertex i) const { return adj_black_[i]; }

int TriGraph::degree(Vertex i) const {
  assert(0 <= i && i < n_orig_);  // vertex out of range

  return adj_black_[i].size() + adj_red_[i].size();
}

int TriGraph::black_degree(Vertex i) const {
  assert(0 <= i && i < n_orig_);  // vertex out of range

  return adj_black_[i].size();
}

/**
 * @brief Returns the red degree at the given vertex.
 *
 * @param i vertex
 * @return int red degree
 */
int TriGraph::red_degree(Vertex i) const {
  assert(0 <= i && i < n_orig_);  // vertex out of range

  return adj_red_[i].size();
}

/**
 * @brief Returns the maximum red degree of this trigraph.
 *
 * @return int max red degree
 */
int TriGraph::max_red_degree() const {
  int ret = 0;
  for (int i = 0; i < n_orig_; ++i) ret = std::max(ret, red_degree(i));
  return ret;
}

//============================================================================
//    Pairwise Properties
//============================================================================
std::vector<TriGraph::Vertex> TriGraph::get_black_symmetric_difference(Vertex i, Vertex j) const {
  assert(0 <= i && i < n_orig_);  // vertex out of range
  assert(0 <= j && j < n_orig_);  // vertex out of range

  return ((adj_black_[i] ^ adj_black_[j]) - i - j).to_vector();
}

/**
 * @brief Computes the red degree of vertex j after vertex i contracts to j.
 *
 * @param i vertex 1
 * @param j vertex 2
 * @return int weak red potential
 */
int TriGraph::weak_red_potential(Vertex i, Vertex j) const {
  assert(i != j);
  auto ret = degree(i) + degree(j) - mu_[i][j] - (has_edge(i, j) ? 2 : 0);
  assert(ret >= 0);
  return ret;
}

int TriGraph::outer_red_potential(Vertex i, Vertex j) const {
  int ret = -1;
  auto a1b1 = (adj_black_[i] ^ adj_black_[j]) - adj_red_[i] - adj_red_[j] - i - j;
  for (auto v : a1b1) ret = std::max(ret, red_degree(v));
  return ret + 1;
}

void TriGraph::compute_pairwise_properties() {
  // We may not change any entries for removed vertices.
  int n = number_of_vertices();
  auto vs = vertices();
  for (int i = 0; i < n; ++i) {
    auto u = vs[i];
    for (int j = i + 1; j < n; ++j) {
      auto v = vs[j];
      auto common_neighbors = (adj_black_[u] | adj_red_[u]) & (adj_black_[v] | adj_red_[v]);
      mu_[u][v] = mu_[v][u] = 2 * common_neighbors.size() - (common_neighbors & (adj_red_[u] | adj_red_[v])).size();
    }
  }
}

}  // namespace graph
}  // namespace ds
