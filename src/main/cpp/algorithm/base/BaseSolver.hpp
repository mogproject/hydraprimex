#pragma once

#include "algorithm/base/SolverState.hpp"
#include "ds/graph/TriGraph.hpp"
#include "util/Random.hpp"

namespace algorithm {
namespace base {
class BaseSolver {
 public:
  virtual void run(SolverState &state, int graph_id, util::Random &rand) = 0;
};
}  // namespace base
}  // namespace algorithm
