#pragma once
#include <unordered_map>
#include <vector>

#include "ds/set/FastSet.hpp"

namespace ds {
namespace graph {

enum class GraphLogType {
  CONTRACTION,
  COMPLEMENT,
};

class GraphLog {
 public:
  GraphLogType log_type = GraphLogType::CONTRACTION;

  int merge = -1;                  // vertex to keep (j)
  int merged = -1;                 // vertex to remove (i)
  std::vector<int> removed_black;  // set of i's old neighbors k such that edge ik was black
  std::vector<int> removed_red;    // set of i's old neighbors k such that edge ik was red
  std::vector<int> new_neighbors;  // set of j's new neighbors
  std::vector<int> recolored;      // set of j's neighbors k such that edge jk turns red from black
  std::vector<std::pair<std::pair<int, int>, int>> mu_delta;  // changes in mu values
  std::vector<std::pair<int, int>> potential_decreased;  // set of vertex pairs whose weak red potential has decreased
};
}  // namespace graph
}  // namespace ds
