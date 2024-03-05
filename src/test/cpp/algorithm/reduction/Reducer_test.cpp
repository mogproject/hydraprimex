#include <gtest/gtest.h>

#include "algorithm/reduction/Reducer.hpp"

using namespace std;
using namespace ds::graph;
using namespace algorithm::reduction;

typedef TriGraph::ContractSeq Seq;

TEST(ReducerTest, Reduce) {
  util::Random rand(12345);
  TriGraph g = {{2, 6, 7, 8}, {{{2, 6}, 0}, {{2, 8}, 0}, {{6, 7}, 1}}};
  Reducer reducer;

  util::set_log_level(util::logging::LogLevel::NONE);
  algorithm::base::SolverState state({g, 0, 4});
  reducer.run(state, 0, rand);
  util::set_log_level(util::logging::LogLevel::TRACE);

  EXPECT_EQ(state.contraction_sequence(), Seq({{8, 6}, {8, 2}, {8, 7}}));
}
