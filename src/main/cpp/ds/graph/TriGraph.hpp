#pragma once

#include "ds/graph/GraphLog.hpp"
#include "ds/set/FastSet.hpp"
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
  std::vector<std::vector<int>> mu_;  // the mu value: mu(i,j) := 2 * (# of common neighbors of i and j) - (# of common red neighbors of i and j)

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
        mu_(n_orig_, std::vector<int>(n_orig_)) {
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
    mu_ = other.mu_;
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
   * @brief Returns the number of original vertices of the graph.
   *
   * @return std::size_t number of original vertices
   */
  std::size_t number_of_original_vertices() const { return n_orig_; }

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

  //============================================================================
  //    Hashing
  //============================================================================
  uint64_t hash() const { return hash_; }

 private:
  uint64_t vertex_hash(int i) const;
  uint64_t red_edge_hash(int i, int j) const;

  //============================================================================
  //    Vertex and Edge Enumeration
  //============================================================================
 private:
  EdgeList get_edges(Color color) const;

 public:
  /**
   * @brief Returns a sorted list of all vertices.
   *
   * @return VertexList all vertices
   */
  VertexList vertices() const;

  /**
   * @brief Returns a list of all edges.
   *
   * @param sorted sort edges if true
   * @return ColoredEdgeList all edges
   */
  ColoredEdgeList edges(bool sorted = false) const;

  /**
   * @brief Returns a list of black edges.
   *
   * @return EdgeList black edge list
   * @note result will be sorted
   */
  EdgeList black_edges() const;

  /**
   * @brief Returns a list of red edges.
   *
   * @return EdgeList red edge list
   * @note result will be sorted
   */
  EdgeList red_edges() const;

  Vertex get_label(Vertex i) const;

  std::unordered_map<Vertex, Vertex> get_label_map() const;

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
  bool has_vertex(Vertex i) const;

  /**
   * @brief Adds a new vertex to the graph.
   *
   * @param i vertex label
   */
  void add_vertex(Vertex i);
  void remove_vertex(Vertex i);
  bool has_black_edge(Vertex i, Vertex j) const;
  bool has_red_edge(Vertex i, Vertex j) const;
  bool has_edge(Vertex i, Vertex j) const;

  /**
   * @brief Adds a new black  edge to the graph.
   *
   * @param i endpoint i
   * @param j endpoint j
   */
  void add_black_edge(Vertex i, Vertex j);
  void add_red_edge(Vertex i, Vertex j);
  void remove_edge(Vertex i, Vertex j);
  void remove_black_edge(Vertex i, Vertex j);
  void remove_red_edge(Vertex i, Vertex j);

  //============================================================================
  //    Neighbors
  //============================================================================
  std::vector<Vertex> neighbors(Vertex i) const;

  AdjSet const& red_neighbors(Vertex i) const;
  AdjSet const& black_neighbors(Vertex i) const;

  int degree(Vertex i) const;

  int black_degree(Vertex i) const;

  /**
   * @brief Returns the red degree at the given vertex.
   *
   * @param i vertex
   * @return int red degree
   */
  int red_degree(Vertex i) const;

  /**
   * @brief Returns the maximum red degree of this trigraph.
   *
   * @return int max red degree
   */
  int max_red_degree() const;

  //============================================================================
  //    Pairwise Properties
  //============================================================================
  std::vector<Vertex> get_black_symmetric_difference(Vertex i, Vertex j) const;

  /**
   * @brief Computes the red degree of vertex j after vertex i contracts to j.
   *
   * @param i vertex 1
   * @param j vertex 2
   * @return int weak red potential
   */
  int weak_red_potential(Vertex i, Vertex j) const;

  int outer_red_potential(Vertex i, Vertex j) const;

  void compute_pairwise_properties();

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
  int contract(Vertex j, Vertex i, GraphLog* graph_log = nullptr);

  void undo(GraphLog const& graph_log);

  /**
   * @brief Finds all vertex pairs whose weak red potential is at most the given threshold.
   *
   * @param upper_bound threshold value
   * @param frozen_vertices vertices excluded from candidates
   * @return std::vector<std::pair<Vertex, Vertex>> contraction candidates
   */
  std::vector<std::pair<Vertex, Vertex>> find_candidates(int upper_bound, VertexList const& frozen_vertices = {}) const;

  /**
   * @brief Updates contraction candidates after a contraction.
   *
   * @param previous_candidates previous candidates
   * @param graph_log GraphLog instance of the previous contraction
   * @param upper_bound threshould value
   * @param frozen_vertices vertices excluded from candidates
   * @return std::vector<std::pair<Vertex, Vertex>> contraction candidates
   */
  std::vector<std::pair<Vertex, Vertex>> update_candidates(               //
      std::vector<std::pair<Vertex, Vertex>> const& previous_candidates,  //
      GraphLog const& graph_log,                                          //
      int upper_bound,                                                    //
      VertexList const& frozen_vertices = {}                              //
  ) const;

  //============================================================================
  //    Contraction sequence verification
  //============================================================================
  static int verify_contraction_sequence(TriGraph const& graph, ContractSeq const& seq);

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
  bool is_free_contraction(Vertex i, Vertex j) const;

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
  //    Debugging
  //============================================================================
  /**
   * @brief Assert that the property `mu_` is consistent with the current graph.
   */
  void check_consistency() const;

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
  friend std::ostream& operator<<(std::ostream& os, TriGraph const& graph);

 private:
  inline int key(int n, int i, int j) const { return n * std::min(i, j) + std::max(i, j); }
};
}  // namespace graph
}  // namespace ds
