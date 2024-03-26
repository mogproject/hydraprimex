#include <gtest/gtest.h>

#include "algorithm/reduction/Reducer.hpp"
#include "algorithm/upperbound/GreedySolver.hpp"
#include "readwrite/pace_extended.hpp"

using namespace std;
using namespace ds::graph;

class GreedySolverTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    // Comment out here if you want to debug with TRACE logs.
    util::set_log_level(util::logging::LogLevel::NONE);
  }
  virtual void TearDown() { util::set_log_level(util::logging::LogLevel::TRACE); }
};

typedef TriGraph::ContractSeq Seq;

static void verify_instance_greedy(std::string const& path, int tww, int num_iterations = 3, int lb = 0,
                                   bool reduce = false, int seed = 12345) {
  util::Random rand(seed);

  auto inst = readwrite::load_pace_extended(path.c_str());
  inst.lower_bound_tww = lb;
  algorithm::base::SolverState state(inst);
  algorithm::upperbound::GreedySolver solver(num_iterations);

  if (reduce) {
    algorithm::reduction::Reducer reducer;
    reducer.run(state, 0, rand);
  }

  solver.run(state, 0, rand);

  int claimed_tww = state.get_upper_bound();
  EXPECT_LE(state.get_lower_bound(), claimed_tww);
  auto actual_tww = ds::graph::TriGraph::verify_contraction_sequence(inst.graph, state.contraction_sequence());
  EXPECT_EQ(claimed_tww, actual_tww);
  EXPECT_GE(claimed_tww, tww);
}

static std::string p(int index, int steps) {
  return util::format("src/test/resources/tiny-set/%03d/tiny%03d-s%d.gr", index, index, steps);
}

TEST_F(GreedySolverTest, RunWithTinyInstances) {
  util::set_log_level(util::logging::LogLevel::NONE);
  for (int s = 0; s <= 9; ++s) verify_instance_greedy(p(1, s), s < 9 ? 1 : 0);
  for (int s = 0; s <= 9; ++s) verify_instance_greedy(p(2, s), s < 7 ? 2 : s < 9 ? 1 : 0);
  for (int s = 0; s <= 20; ++s) verify_instance_greedy(p(5, s), 2);
  for (int s = 0; s <= 20; ++s) verify_instance_greedy(p(7, s), 2);
  for (int s = 0; s <= 9; ++s) verify_instance_greedy(p(8, s), s < 4 ? 4 : s < 6 ? 3 : s < 8 ? 2 : s < 9 ? 1 : 0);
  for (int s = 0; s <= 8; ++s) verify_instance_greedy(p(9, s), s < 8 ? 1 : 0);
  for (int s = 0; s <= 19; ++s) verify_instance_greedy(p(10, s), s < 18 ? 2 : s < 19 ? 1 : 0);
  util::set_log_level(util::logging::LogLevel::TRACE);
}

TEST_F(GreedySolverTest, RunWithPaceInstances) {
  verify_instance_greedy("data/pace2023/exact_045.gr", 2, 10000, 2, true, 12345);
}
