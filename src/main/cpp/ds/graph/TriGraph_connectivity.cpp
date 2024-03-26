#include "TriGraph.hpp"

namespace ds {
namespace graph {

/**
 * @brief Finds a component in the graph including the specific vertex.
 *
 * @param g graph
 * @param v vertex
 * @return std::vector<TriGraph::Vertex> vertices in the component
 */
static std::vector<TriGraph::Vertex> get_component(TriGraph const& g, TriGraph::Vertex v) {
  static ds::set::FastSet visited;
  visited.initialize(g.number_of_original_vertices());

  std::vector<int> component = {v};
  std::stack<int> st;

  st.push(v);
  visited.set(v);

  while (!st.empty()) {
    auto x = st.top();
    st.pop();
    for (auto y : g.neighbors(x)) {
      if (!visited.get(y)) {
        component.push_back(y);
        st.push(y);
        visited.set(y);
      }
    }
  }
  return component;
}

/**
 * @brief Finds all connected components in the graph, using both black and red edges.
 *
 * @return std::vector<std::vector<Vertex>> list of components
 */
std::vector<std::vector<TriGraph::Vertex>> TriGraph::connected_components() const {
  std::vector<std::vector<TriGraph::Vertex>> ret;
  static ds::set::FastSet visited;
  visited.initialize(number_of_original_vertices());

  for (auto v : vertices()) {
    if (!visited.get(v)) {
      auto component = get_component(*this, v);
      for (auto x : component) visited.set(x);
      ret.push_back(component);
    }
  }
  return ret;
}

/**
 * @brief Returns true if the graph is connected using both black and red edges.
 *
 * @return true graph is connected
 * @return false graph is disconnected
 */
bool TriGraph::is_connected() const {
  if (number_of_vertices() <= 1) return true;
  auto vs = vertices();
  auto cc = get_component(*this, vs[0]);
  return cc.size() == number_of_vertices();
}

}  // namespace graph
}  // namespace ds
