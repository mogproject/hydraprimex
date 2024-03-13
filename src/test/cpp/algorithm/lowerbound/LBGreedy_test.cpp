#include <gtest/gtest.h>

#include "algorithm/lowerbound/LBGreedy.hpp"
#include "algorithm/reduction/Reducer.hpp"
#include "readwrite/pace_extended.hpp"

using namespace std;
using namespace ds::graph;

typedef TriGraph::ContractSeq Seq;

static void verify_instance(std::string const& path, int lb) {
  util::Random rand(12345);

  auto inst = readwrite::load_pace_extended(path.c_str());
  algorithm::base::SolverState state(inst);
  algorithm::lowerbound::LBGreedy solver;

  solver.run(state, 0, rand);
  EXPECT_EQ(state.get_lower_bound(), lb);
}

static std::string p(int index, int steps) {
  return util::format("src/test/resources/tiny-set/%03d/tiny%03d-s%d.gr", index, index, steps);
}

TEST(LBGreedyTest, RunWithTinyInstances) {
  util::set_log_level(util::logging::LogLevel::NONE);
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
  util::set_log_level(util::logging::LogLevel::TRACE);
}

TEST(LBGreedyTest, RunWithReducedInstances) {
  util::Random rand(12345);

  auto inst = readwrite::load_pace_extended("src/test/resources/pace2023/exact_056_twwe.gr");

  algorithm::lowerbound::LBGreedy solver;
  algorithm::reduction::Reducer reducer;

  util::set_log_level(util::logging::LogLevel::NONE);
  algorithm::base::SolverState state(inst);
  reducer.run(state, 0, rand);
  solver.run(state, 0, rand);
  util::set_log_level(util::logging::LogLevel::TRACE);

  EXPECT_EQ(state.get_lower_bound(), 2);
}
