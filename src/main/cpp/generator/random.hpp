#pragma once

#include "ds/graph/TriGraph.hpp"
#include "util/Random.hpp"

namespace generator {
ds::graph::TriGraph erdos_renyi_graph(int n, double p, util::Random &rand);
}
