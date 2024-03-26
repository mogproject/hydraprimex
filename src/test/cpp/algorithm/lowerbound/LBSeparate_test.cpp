#include <gtest/gtest.h>

#include "algorithm/lowerbound/LBSeparate.hpp"
#include "algorithm/reduction/Reducer.hpp"
#include "readwrite/pace_extended.hpp"

using namespace std;
using namespace ds::graph;

class LBSeparateTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    // Comment out here if you want to debug with TRACE logs.
    util::set_log_level(util::logging::LogLevel::NONE);
  }
  virtual void TearDown() { util::set_log_level(util::logging::LogLevel::TRACE); }
};

typedef TriGraph::ContractSeq Seq;

static void verify_instance(std::string const& path, int lb, bool reduce = false) {
  util::Random rand(12345);

  auto inst = readwrite::load_pace_extended(path.c_str());
  algorithm::base::SolverState state(inst);
  algorithm::lowerbound::LBSeparate solver;

  if (reduce) {
    algorithm::reduction::Reducer reducer;
    reducer.run(state, 0, rand);
  }

  solver.run(state, 0, rand);
  EXPECT_GE(state.get_lower_bound(), lb);
}

static std::string p(int index, int steps) {
  return util::format("src/test/resources/tiny-set/%03d/tiny%03d-s%d.gr", index, index, steps);
}

TEST_F(LBSeparateTest, RunWithTinyInstances) {
  for (int s = 0; s <= 9; ++s) verify_instance(p(1, s), s < 9 ? 1 : 0);
  for (int s = 0; s <= 9; ++s) verify_instance(p(2, s), s < 7 ? 2 : s < 9 ? 1 : 0);
  for (int s = 0; s <= 9; ++s) verify_instance(p(3, s), 0);
  for (int s = 0; s <= 9; ++s) verify_instance(p(4, s), 0);
  for (int s = 0; s <= 20; ++s) verify_instance(p(5, s), s == 0 ? 2 : 3);
  for (int s = 0; s <= 9; ++s) verify_instance(p(6, s), 0);
  for (int s = 0; s <= 20; ++s) verify_instance(p(7, s), s == 15 || s == 18 || s == 20 ? 2 : 1);
  for (int s = 0; s <= 9; ++s) verify_instance(p(8, s), s < 4 ? 4 : s < 6 ? 3 : s < 8 ? 2 : s < 9 ? 1 : 0);
  for (int s = 0; s <= 8; ++s) verify_instance(p(9, s), s < 8 ? 1 : 0);
  for (int s = 0; s <= 19; ++s) verify_instance(p(10, s), s < 18 ? 2 : s < 19 ? 1 : 0);
}

TEST_F(LBSeparateTest, RunWithReducedInstances) {
  // verify_instance("src/test/resources/small/008/small008.gr", 2, true);
  verify_instance("data/pace2023/exact_021.gr", 3, true);
}
