#pragma once

#include "ds/graph/TriGraph.hpp"

namespace ds {
/**
 * @brief Represents one instance for Annotated Twin-width.
 */
struct ProblemInstance {
  /** Input trigraph. */
  graph::TriGraph graph;

  /** Known lower bound. */
  int lower_bound_tww = 0;

  /** Known upper bound. */
  int upper_bound_tww = -1;  // -1: infinity
};
}  // namespace ds
