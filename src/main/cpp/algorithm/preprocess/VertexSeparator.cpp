#include "VertexSeparator.hpp"
#include "ds/map/Bimap.hpp"
#include "ds/set/FastSet.hpp"

typedef ds::graph::TriGraph::Vertex Vertex;

namespace algorithm {
namespace preprocess {
std::vector<Vertex> VertexSeparator::find_articulation_points(ds::graph::TriGraph const& g) const {
  int n = g.number_of_vertices();

  std::vector<Vertex> ret;
  std::vector<int> dfsnum(n, -1), dfslow(n, 0), parent(n, -1);
  static ds::set::FastSet fs;
  fs.initialize(n);
  int dfscnt, root, num_root_children;

  // inner function
  auto f = [&](int u, auto& self) -> void {
    dfsnum[u] = dfslow[u] = dfscnt++;

    for (auto v : g.neighbors(u)) {
      if (dfsnum[v] < 0) {  // unvisited
        parent[v] = u;
        if (u == root) ++num_root_children;  // special case: u is a root
        self(v, self);                       // recurse

        if (u != root && dfslow[v] >= dfsnum[u]) {
          if (!fs.get(u)) {
            ret.push_back(u);  // u is an articulation point
            fs.set(u);
          }
        }
        dfslow[u] = std::min(dfslow[u], dfslow[v]);
      } else if (v != parent[u]) {  // back edge and not direct cycle
        dfslow[u] = std::min(dfslow[u], dfsnum[v]);
      }
    }
  };

  for (int i = 0; i < n; ++i) {
    if (dfsnum[i] >= 0) continue;
    root = i;
    dfscnt = 0;
    num_root_children = 0;
    f(root, f);
    if (!fs.get(root) && num_root_children > 1) ret.push_back(root);  // special case
  }

  return ret;
}

std::vector<ds::graph::TriGraph> VertexSeparator::decompose_into_covers(ds::graph::TriGraph const& graph, int max_separator_size) {
  std::vector<Vertex> part_a, part_b;

  std::vector<ds::graph::TriGraph> ret = {graph};
  std::queue<int> q;
  q.push(0);

  while (!q.empty()) {
    auto i = q.front();
    q.pop();

    // find the smallest separator (this will keep us from generating duplicated covers)
    bool found = false;
    for (int sz = 1; sz <= max_separator_size; ++sz) {
      auto sep = find_vertex_separator(ret[i], sz, &part_a, &part_b);
      if (!sep.empty()) {
        found = true;
        // printf("a=%lu, b=%lu, z=%lu\n", part_a.size(), part_b.size(), sep.size());
        break;
      }
    }

    // if separator not found; keep the graph as is
    if (found) {
      // separator found; decompose into two parts
      q.push(ret.size());  // add last index to queue
      q.push(i);           // add current index to queue
      ret.push_back(ret[i].subgraph(part_b, true));
      ret[i] = ret[i].subgraph(part_a, true);  // replace graph
    }
  }

  return ret;
}

std::vector<int> VertexSeparator::find_vertex_separator(  //
    ds::graph::TriGraph const& graph,                     //
    int max_separator_size,                               //
    std::vector<Vertex>* part_a,                          //
    std::vector<Vertex>* part_b,                          //
    std::vector<Vertex> const& head,                      //
    std::vector<Vertex> const& tail,                      //
    util::Random* rand,                                   //
    bool allow_overlap,                                   //
    bool require_balance,                                 //
    std::vector<Vertex> const& non_separator              //
) const {
  log_trace("vertices: %s", cstr(graph.vertices()));
  log_trace("edges: %s", cstr(graph.edges()));
  log_trace("head=%s, tail=%s, non_separator=%s", cstr(head), cstr(tail), cstr(non_separator));

  int n = graph.number_of_vertices();
  std::vector<Vertex> xs, ys, zs, aa, bb;

  // vertex label remapping for randomness
  ds::map::Bimap<int> labels(graph.vertices());
  if (rand) labels.shuffle(*rand);

  //==========================================================================
  // Encode SAT instance
  //==========================================================================

  // solver_.restart(rand);  // @note Never use this. This will lead to memory corruption.
  solver_.restart();

  // color encoding: every vertex must be x (piece 1), y (piece 2), or z (separator)
  static ds::set::FastSet fs1, fs2;
  fs1.initialize(n);
  fs2.initialize(n);

  for (auto x : head) {
    auto a = labels.g(x);
    fs1.set(a);
    aa.push_back(-z(a));
  }
  for (auto x : tail) {
    auto b = labels.g(x);
    fs2.set(b);
    bb.push_back(-z(b));
  }

  if (allow_overlap) {
    if (!head.empty()) solver_.add_clause(aa);  // at least one from head must be a non-separator
    if (!tail.empty()) solver_.add_clause(bb);  // at least one from tail must be a non-separator
  }

  for (int i = 0; i < n; ++i) {
    solver_.add_equals_one({x(i), y(i), z(i)});

    xs.push_back(x(i));
    ys.push_back(y(i));
    zs.push_back(z(i));

    if (fs1.get(i)) {
      solver_.add_clause({-y(i)});  // left divider: x or z
    }
    if (fs2.get(i)) {
      solver_.add_clause({-x(i)});  // right divider: y or z
    }
    if (!allow_overlap && (fs1.get(i) || fs2.get(i))) solver_.add_clause({-z(i)});  // divider cannot be separator
  }

  // reachability
  for (int i = 0; i < n; ++i) {
    auto u = labels.f(i);
    for (auto v : graph.neighbors(u)) {
      auto j = labels.g(v);
      // x(i) -> x(j) or z(j)
      solver_.add_clause({-x(i), x(j), z(j)});
      // y(i) -> y(j) or z(j)
      solver_.add_clause({-y(i), y(j), z(j)});
    }
  }

  // non-emptyness
  solver_.add_atleast(xs, 1);
  solver_.add_atleast(ys, 1);

  if (require_balance) {
    auto ubound = 9 * n / 10;
    solver_.add_atmost(xs, ubound);
    solver_.add_atmost(ys, ubound);
  }

  // non-separator
  for (auto v : non_separator) solver_.add_clause({-z(labels.g(v))});

  // constraints on the solution size
  solver_.add_atmost(zs, max_separator_size);

  //==========================================================================
  // Solve with a SAT solver
  //==========================================================================

  int ret = solver_.solve(0);
  if (ret == sat::status::SATISFIABLE) {
    // deocde solution
    std::vector<int> solution, result_a, result_b;

    for (int i = 0; i < n; ++i) {
      auto v = labels.f(i);
      if (solver_.get_witness(z(i))) {
        solution.push_back(v);
        result_a.push_back(v);
        result_b.push_back(v);
      } else if (solver_.get_witness(x(i))) {
        result_a.push_back(v);
      } else {
        assert(solver_.get_witness(y(i)));
        result_b.push_back(v);
      }
    }

    log_trace("solution: %s", cstr(solution));
    log_trace("A: %s", cstr(result_a));
    log_trace("B: %s", cstr(result_b));

    if (part_a) *part_a = result_a;
    if (part_b) *part_b = result_b;
    return solution;
  }
  return {};  // solution not found
}

inline int VertexSeparator::x(int i) const { return solver_.id(1000000000L + i); }
inline int VertexSeparator::y(int i) const { return solver_.id(2000000000L + i); }
inline int VertexSeparator::z(int i) const { return solver_.id(3000000000L + i); }
}  // namespace preprocess
}  // namespace algorithm
