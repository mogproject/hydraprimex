#include <gtest/gtest.h>

#include "algorithm/reduction/Reducer.hpp"

using namespace std;
using namespace ds::graph;
using namespace algorithm::reduction;

typedef TriGraph::ContractSeq Seq;

TEST(ReducerTest, Reduce) {
  TriGraph g = {{2, 6, 7, 8}, {{{2, 6}, 0}, {{2, 8}, 0}, {{6, 7}, 1}}};

  Reducer reducer;

  util::set_log_level(util::logging::LogLevel::NONE);
  auto ret = reducer.reduce(g);
  util::set_log_level(util::logging::LogLevel::TRACE);

  EXPECT_EQ(ret, Seq({{3, 1}, {3, 0}, {3, 2}}));
}
