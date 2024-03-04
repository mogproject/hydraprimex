#pragma once

#include "ds/set/SortedVectorSet.hpp"
#include "util/hash_table.hpp"
#include "util/logger.hpp"
#include "util/util.hpp"

namespace ds {
namespace graph {
class TriGraph {
 public:
  //============================================================================
  //    Type Aliases
  //============================================================================
  typedef int Vertex;                                // vertex label
  typedef std::pair<Vertex, Vertex> Edge;            // edge
  typedef int Color;                                 // edge color: 0=black, 1=red
  typedef ds::set::SortedVectorSet AdjSet;           // adjacency set
  typedef std::vector<Vertex> VertexList;            // vertex list
  typedef std::pair<Edge, Color> ColoredEdge;        // edge with color
  typedef std::vector<ColoredEdge> ColoredEdgeList;  // colored edge list
  typedef std::vector<Edge> EdgeList;                // edge list
  typedef EdgeList ContractSeq;                      // contraction sequence

  //============================================================================
  //    Constants
  //============================================================================
  static int const Black = 0;
  static int const Red = 1;

 private:
  //============================================================================
  //    Fields
  //============================================================================
  /** Labels for original vertices. */
  VertexList vertex_labels_;

  /** Original number of vertices. */
  int n_orig_;

  /** Set of active vertices. */
  AdjSet vertices_;

  /** Adjacency sets for black edges. */
  std::vector<AdjSet> adj_black_;

  /** Adjacency sets for red edges. */
  std::vector<AdjSet> adj_red_;

  /** Number of black edges. */
  int num_black_edges_ = 0;

  /** Number of black edges. */
  int num_red_edges_ = 0;

  /** Hash value for this trigraph. */
  uint64_t hash_ = 0ULL;

  //============================================================================
  //    Pairwise Properties
  //============================================================================
  std::vector<std::vector<int>> ncn_;  // number of common neighbors
  std::vector<std::vector<int>> ncr_;  // number of common red neighbors

 public:
  TriGraph() : TriGraph({}, {}) {}

  TriGraph(VertexList const& vertices, ColoredEdgeList const& edges)
      : vertex_labels_(vertices),
        n_orig_(vertices.size()),
        adj_black_(n_orig_),
        adj_red_(n_orig_),
        num_black_edges_(0),
        num_red_edges_(0),
        hash_(0ULL),
        ncn_(n_orig_, std::vector<int>(n_orig_)),
        ncr_(n_orig_, std::vector<int>(n_orig_)) {
    // initialize hash table
    util::initialize_hash_table();

    // compress vertex labels
    std::unordered_map<int, int> label_inv;
    for (int i = 0; i < n_orig_; ++i) label_inv[vertices[i]] = i;

    // initialize vertex hash
    for (int i = 0; i < n_orig_; ++i) hash_ ^= vertex_hash(i);

    // initialize vertices
    for (int i = 0; i < n_orig_; ++i) vertices_.set(i);

    // create colored edges
    for (auto& e : edges) {
      auto i = label_inv[e.first.first];
      auto j = label_inv[e.first.second];
      switch (e.second) {
        case Black: {
          add_black_edge(i, j);
          break;
        }
        case Red: {
          add_red_edge(i, j);
          break;
        }
      }
    }

    // compute pairwise properties
    compute_pairwise_properties();
  }

  /**
   * equality
   */
  friend bool operator==(TriGraph const& lhs, TriGraph const& rhs) {
    if (lhs.hash_ != rhs.hash_) return false;
    if (lhs.vertex_labels_ != rhs.vertex_labels_) return false;
    if (lhs.vertices_ != rhs.vertices_) return false;
    if (lhs.adj_black_ != rhs.adj_black_) return false;
    if (lhs.adj_red_ != rhs.adj_red_) return false;
    return true;
  }

  /**
   * assignment
   */
  void operator=(TriGraph const& other) {
    vertex_labels_ = other.vertex_labels_;
    n_orig_ = other.n_orig_;
    vertices_ = other.vertices_;
    adj_black_ = other.adj_black_;
    adj_red_ = other.adj_red_;
    num_black_edges_ = other.num_black_edges_;
    num_red_edges_ = other.num_red_edges_;
    hash_ = other.hash_;
    ncn_ = other.ncn_;
    ncr_ = other.ncr_;
  }

  /**
   * copy constructor
   */
  TriGraph(TriGraph const& other) { *this = other; }

  //============================================================================
  //    Global Properties
  //============================================================================
  /**
   * @brief Returns the number of vertices in the graph.
   *
   * @return std::size_t number of vertices
   */
  std::size_t number_of_vertices() const { return vertices_.size(); }

  /**
   * @brief Returns the number of edges in the graph.
   *
   * @return std::size_t number of edges
   */
  std::size_t number_of_edges() const { return num_black_edges_ + num_red_edges_; }

  /**
   * @brief Returns the number of black edges in the graph.
   *
   * @return std::size_t number of black edges
   */
  std::size_t number_of_black_edges() const { return num_black_edges_; }

  /**
   * @brief Returns the number of red edges in the graph.
   *
   * @return std::size_t number of red edges
   */
  std::size_t number_of_red_edges() const { return num_red_edges_; }

 private:
  //============================================================================
  //    Hashing
  //============================================================================
  inline uint64_t vertex_hash(int i) const {
    auto mod = 1 << (util::HASH_TABLE_BITS - 1);
    return util::get_hash(mod + i % mod);
  }

  inline uint64_t red_edge_hash(int i, int j) const {
    auto mod = 1 << ((util::HASH_TABLE_BITS - 1) / 2);
    return util::get_hash((std::min(i, j) % mod) * mod + (std::max(i, j) % mod));
  }

 public:
  //============================================================================
  //    Vertex and Edge Enumeration
  //============================================================================
  /**
   * @brief Returns a sorted list of all vertices.
   *
   * @return VertexList all vertices
   */
  VertexList vertices() const { return vertices_.to_vector(); }

  /**
   * @brief Returns a list of all edges.
   *
   * @param sorted sort edges if true
   * @return ColoredEdgeList all edges
   */
  ColoredEdgeList edges(bool sorted = false) const {
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

  /**
   * @brief Returns a list of black edges.
   *
   * @return EdgeList black edge list
   * @note result will be sorted
   */
  EdgeList black_edges() const { return get_edges(Black); }

  /**
   * @brief Returns a list of red edges.
   *
   * @return EdgeList red edge list
   * @note result will be sorted
   */
  EdgeList red_edges() const { return get_edges(Red); }

  Vertex get_label(Vertex i) const {
    assert(0 <= i && i < n_orig_);
    return vertex_labels_[i];
  }

  std::unordered_map<Vertex, Vertex> get_label_map() const {
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
  bool has_vertex(Vertex i) const {
    assert(0 <= i && i < n_orig_);  // vertex out of range
    return vertices_.get(i);
  }

  /**
   * @brief Adds a new vertex to the graph.
   *
   * @param i vertex label
   */
  void add_vertex(Vertex i) {
    assert(0 <= i && i < n_orig_);  // vertex out of range
    assert(!vertices_.get(i));      // vertex already exists

    vertices_.set(i);
    hash_ ^= vertex_hash(i);
  }

  void remove_vertex(Vertex i) {
    assert(0 <= i && i < n_orig_);  // vertex out of range
    assert(vertices_.get(i));       // vertex does not exist

    for (int w : adj_black_[i].to_vector()) remove_black_edge(i, w);
    for (int w : adj_red_[i].to_vector()) remove_red_edge(i, w);
    vertices_.reset(i);
    hash_ ^= vertex_hash(i);
  }

  bool has_black_edge(Vertex i, Vertex j) const {
    assert(0 <= i && i < n_orig_);  // vertex out of range
    assert(0 <= j && j < n_orig_);  // vertex out of range

    return adj_black_[i].get(j);
  }

  bool has_red_edge(Vertex i, Vertex j) const {
    assert(0 <= i && i < n_orig_);  // vertex out of range
    assert(0 <= j && j < n_orig_);  // vertex out of range

    return adj_red_[i].get(j);
  }

  bool has_edge(Vertex i, Vertex j) const { return has_black_edge(i, j) || has_red_edge(i, j); }

  /**
   * @brief Adds a new black  edge to the graph.
   *
   * @param i endpoint i
   * @param j endpoint j
   */
  void add_black_edge(Vertex i, Vertex j) {
    assert(!has_edge(i, j));

    adj_black_[i].set(j);
    adj_black_[j].set(i);
    ++num_black_edges_;
  }

  void add_red_edge(Vertex i, Vertex j) {
    assert(!has_edge(i, j));

    adj_red_[i].set(j);
    adj_red_[j].set(i);
    ++num_red_edges_;
    hash_ ^= red_edge_hash(i, j);
  }

  void remove_black_edge(Vertex i, Vertex j) {
    assert(has_black_edge(i, j));

    adj_black_[i].reset(j);
    adj_black_[j].reset(i);
    --num_black_edges_;
  }

  void remove_red_edge(Vertex i, Vertex j) {
    assert(has_red_edge(i, j));

    adj_red_[i].reset(j);
    adj_red_[j].reset(i);
    --num_red_edges_;
    hash_ ^= red_edge_hash(i, j);
  }

  //============================================================================
  //    Neighbors
  //============================================================================
  std::vector<Vertex> neighbors(Vertex i) const {
    assert(0 <= i && i < n_orig_);  // vertex out of range

    auto ret = adj_black_[i].to_vector();
    util::extend(ret, adj_red_[i].to_vector());
    return ret;
  }

  int degree(Vertex i) const {
    assert(0 <= i && i < n_orig_);  // vertex out of range

    return adj_black_[i].size() + adj_red_[i].size();
  }

  int black_degree(Vertex i) const {
    assert(0 <= i && i < n_orig_);  // vertex out of range

    return adj_black_[i].size();
  }

  /**
   * @brief Returns the red degree at the given vertex.
   *
   * @param i vertex
   * @return int red degree
   */
  int red_degree(Vertex i) const {
    assert(0 <= i && i < n_orig_);  // vertex out of range

    return adj_red_[i].size();
  }

  /**
   * @brief Returns the maximum red degree of this trigraph.
   *
   * @return int max red degree
   */
  int max_red_degree() const {
    int ret = 0;
    for (int i = 0; i < n_orig_; ++i) ret = std::max(ret, red_degree(i));
    return ret;
  }

  //============================================================================
  //    Pairwise Properties
  //============================================================================
  std::vector<Vertex> get_black_symmetric_difference(Vertex i, Vertex j) const {
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
  int weak_red_potential(Vertex i, Vertex j) const {
    assert(i != j);
    auto ret = degree(i) + degree(j) - 2 * ncn_[i][j] + ncr_[i][j] - (has_edge(i, j) ? 2 : 0);
    assert(ret >= 0);
    return ret;
  }

  void compute_pairwise_properties() {
    for (int i = 0; i < n_orig_; ++i) {
      for (int j = i + 1; j < n_orig_; ++j) {
        auto common_neighbors = (adj_black_[i] | adj_red_[i]) & (adj_black_[j] | adj_red_[j]);
        ncn_[i][j] = ncn_[j][i] = common_neighbors.size();
        ncr_[i][j] = ncr_[j][i] = (common_neighbors & (adj_red_[i] | adj_red_[j])).size();
      }
    }
  }

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
  int contract(Vertex j, Vertex i) {
    assert(has_vertex(i));
    assert(has_vertex(j));
    assert(i != j);

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

    auto a12 = a1 | a2;

    //--------------------------------------------------------------------------
    // (1) Update the number of common neighbors
    //--------------------------------------------------------------------------
    for (auto a : a12) {
      for (auto b : b1 | b2) {
        ++ncn_[a][b];  // j becomes a new common neighbor of a and b
        ++ncn_[b][a];
      }
      for (auto v : adj_black_[a] | adj_red_[a]) {
        if (v != i) {
          ++ncn_[j][v];  // a becomes a new common neighbor of j and v
          ++ncn_[v][j];
        }
      }
      if (edge_ij) {
        --ncn_[j][a];  // i was a common neighbor
        --ncn_[a][j];
      }
    }

    for (auto x : common_nbrs) {
      for (auto y : common_nbrs) {
        if (x == y) continue;
        --ncn_[x][y];  // i was a common neighbor of x and y
      }
    }

    if (edge_ij) {
      for (auto x : common_nbrs) {
        --ncn_[j][x];  // i was a common neighbor of j and x
        --ncn_[x][j];
      }
    }

    //--------------------------------------------------------------------------
    // (2) Update the number of common red neighbors
    //--------------------------------------------------------------------------
    // (2a) red edges incident to i (including edge ij)
    for (auto x : adj_red_[i]) {
      for (auto y : adj_red_[i]) {
        if (x == y) continue;
        --ncr_[x][y];
      }
      for (auto y : adj_black_[i]) {
        --ncr_[x][y];
        --ncr_[y][x];
      }
    }

    // (2b) for new red edge xj, j becomes a new common red neighbor of x and y for y <- N({i, j})
    auto new_nbrs = a12 | c3 | b1;
    for (auto x : new_nbrs) {
      for (auto y : new_nbrs) {
        if (x == y) continue;
        ++ncr_[x][y];
      }
    }

    for (auto x : a12) {
      for (auto y : c1 | c2 | c4 | b2) {
        ++ncr_[x][y];
        ++ncr_[y][x];
      }
    }

    for (auto x : c3 | b1) {
      for (auto y : c1) {
        ++ncr_[x][y];
        ++ncr_[y][x];
      }
    }

    // (2c) for new red edge xj, x becomes a new common red neighbor of y and j for y <- N_B(x)
    for (auto x : a12) {
      for (auto y : adj_black_[x] | adj_red_[x]) {
        if (y == i) continue;
        ++ncr_[y][j];
        ++ncr_[j][y];
      }
    }
    for (auto x : c3 | b1) {
      for (auto y : adj_black_[x]) {
        if (y == j) continue;
        ++ncr_[y][j];
        ++ncr_[j][y];
      }
    }

    //--------------------------------------------------------------------------
    // (3) Make changes
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
    for (int w : a12) {
      //
      add_red_edge(j, w);
    }

    // remove vertex i
    remove_vertex(i);

    //--------------------------------------------------------------------------
    // (4) Return max red degree in the neighborhood
    //--------------------------------------------------------------------------
    int ret = red_degree(j);
    for (auto w : adj_black_[j] | adj_red_[j]) ret = std::max(ret, red_degree(w));
    return ret;
  }

  void undo_contract() {}

  //============================================================================
  //    Contraction sequence verification
  //============================================================================
  static int verify_contraction_sequence(TriGraph const& graph, ContractSeq const& seq) {
    auto n = graph.number_of_vertices();
    if (n == 0) {
      if (!seq.empty()) throw std::runtime_error("seq must be empty for empty graphs");
    }
    if (seq.size() + 1 != n) throw std::runtime_error("seq size mismatch");

    auto g = graph;  // create a copy
    int red_deg = g.max_red_degree();

    for (auto& p : seq) {
      auto j = p.first;
      auto i = p.second;
      if (!g.has_vertex(i) || !g.has_vertex(j)) throw std::runtime_error("vertex does not exist");
      red_deg = std::max(red_deg, g.contract(j, i));
    }
    return red_deg;
  }

  //============================================================================
  //    Debugging
  //============================================================================
  /**
   * @brief Assert that the properties `ncn_` and `ncr_` are consistent
   * with the current graph.
   */
  void check_consistency() const {
    auto h = *this;
    h.compute_pairwise_properties();

    auto ncn = ncn_;
    auto ncr = ncr_;
    for (int i = 0; i < n_orig_; ++i) {
      if (has_vertex(i)) continue;
      for (auto j = 0; j < n_orig_; ++j) {
        ncn[i][j] = ncn[j][i] = 0;
        ncr[i][j] = ncr[j][i] = 0;
      }
    }
    if (h.ncn_ != ncn) {
      log_debug("Found inconsistency in ncn: expected=%s, actual=%s", cstr(h.ncn_), cstr(ncn_));
      throw std::runtime_error("error in TriGraph#check_consistency()");
    }

    if (h.ncr_ != ncr) {
      log_debug("Found inconsistency in ncr: expected=%s, actual=%s", cstr(h.ncr_), cstr(ncr_));
      throw std::runtime_error("error in TriGraph#check_consistency()");
    }
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
  bool is_free_contraction(Vertex i, Vertex j) const {
    assert(has_vertex(i) && has_vertex((j)));

    // (1) They must be black twins.
    if ((adj_black_[i] - j) != (adj_black_[j] - i)) return false;

    // (2) Red neighbors have subset relation.
    if (adj_red_[i].size() > adj_red_[j].size()) std::swap(i, j);
    return (adj_red_[i] - j - adj_red_[j]).empty();
  }

  //============================================================================
  //    Complement
  //============================================================================

  /**
   * @brief Takes the black-edge complement; converts black edges to non-edges
   * and non-edges to black edges.
   */
  void black_complement() {
    int n = number_of_vertices();
    if (n <= 1) return;

    auto vs = vertices_.to_vector();
    auto vs_map = util::inverse_map(vs);

    std::vector<std::vector<int>> mat(n, std::vector<int>(n));
    for (auto e : black_edges()) {
      assert(e.first < e.second);
      mat[vs_map[e.first]][vs_map[e.second]] = 1;
    }
    for (auto e : red_edges()) {
      assert(e.first < e.second);
      mat[vs_map[e.first]][vs_map[e.second]] = 2;
    }

    for (int i = 0; i < n; ++i) {
      for (int j = i + 1; j < n; ++j) {
        if (mat[i][j] == 0) {
          add_black_edge(vs[i], vs[j]);
        } else if (mat[i][j] == 1) {
          remove_black_edge(vs[i], vs[j]);
        }
      }
    }

    // need to recompute properties
    compute_pairwise_properties();
  }

  //============================================================================
  //    I/O
  //============================================================================
  /**
   * @brief String representation of the instance.
   *
   * @param os output stream
   * @param graph Graph instance
   * @return std::ostream& output stream
   */
  friend std::ostream& operator<<(std::ostream& os, TriGraph const& graph) {
    return os << util::format(                          //
               "TriGraph(n=%lu, m=%lu, hash=%016llx)",  //
               graph.number_of_vertices(), graph.number_of_edges(), graph.hash_);
  }

 private:
  EdgeList get_edges(Color color) const {
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
};
}  // namespace graph
}  // namespace ds
