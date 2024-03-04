#include <fstream>

#include "pace_extended.hpp"
#include "util/logger.hpp"

namespace readwrite {
ds::ProblemInstance read_pace_extended(std::istream &is) {
  typedef ds::graph::TriGraph::VertexList VertexList;
  typedef ds::graph::TriGraph::ColoredEdgeList ColoredEdgeList;

  VertexList vertices;
  ColoredEdgeList edges;

  int n = -1;
  int m = -1;
  int lb = 0;
  int ub = -1;
  int i = 0;

  for (std::string line; std::getline(is, line);) {
    if (line.empty()) continue;

    while (i < static_cast<int>(line.size()) && !std::isprint(line[i])) ++i;
    if (line[i] == 'c') continue;  // ignore comments

    auto tokens = util::split(line);
    if (tokens[0] == "p") {  // p-line
      if (tokens.size() < 4) throw std::invalid_argument(util::format("unexpected p-line: %s", line.c_str()));
      if (tokens[1] != "twwe") throw std::invalid_argument(util::format("unexpected p-line: %s", line.c_str()));

      n = std::stoi(tokens[2]);
      m = std::stoi(tokens[3]);
      if (tokens.size() >= 5) ub = std::stoi(tokens[4]);
      if (tokens.size() >= 6) lb = std::stoi(tokens[5]);

      if (lb >= n)
        if (ub >= 0 && ub < lb) {
          throw std::invalid_argument(util::format("upper bound too small: n=%d, lb=%d, ub=%d", n, lb, ub));
        }
    } else if (tokens[0] == "v") {  // v-line
      if (tokens.size() != 2) throw std::invalid_argument("unexpected v-line");

      auto v = std::stoi(tokens[1]);
      vertices.push_back(v);
    } else if (tokens[0] == "e") {  // e-line
      if (tokens.size() != 4) throw std::invalid_argument("unexpected e-line");

      auto u = std::stoi(tokens[1]);
      auto v = std::stoi(tokens[2]);
      auto color = std::stoi(tokens[3]);
      if (color != ds::graph::TriGraph::Black && color != ds::graph::TriGraph::Red) {
        throw std::invalid_argument(util::format("unexpected color: %d", color));
      }
      edges.push_back({{u, v}, color});
    }
  }

  ds::graph::TriGraph G(vertices, edges);

  // verification
  if (n != static_cast<int>(G.number_of_vertices())) throw std::invalid_argument("inconsistent n");
  if (m != static_cast<int>(G.number_of_edges())) throw std::invalid_argument("inconsistent m");
  if (ub >= 0 && lb > ub) {
    throw std::invalid_argument(util::format("lower bound larger than upper bound: n=%d, lb=%d, ub=%d", n, lb, ub));
  }

  ds::ProblemInstance ret = {G, lb, ub};
  return ret;
}

ds::ProblemInstance load_pace_extended(char const *path) {
  std::ifstream f(path);
  if (f.fail()) throw std::invalid_argument(util::format("Failed to open file: %s", path));
  return read_pace_extended(f);
}
}  // namespace readwrite
