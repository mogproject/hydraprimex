#include <gtest/gtest.h>

#include "algorithm/exact/SATSolver.hpp"
#include "readwrite/pace_extended.hpp"

using namespace std;

static void verify_instance(char const* pattern, int index, int n, int tww) {
  util::Random rand(12345);
  for (int s = 0; s < n; ++s) {
    auto path = util::format(pattern, index, index, s);
    auto inst = readwrite::load_pace_extended(path.c_str());

    log_info("---------- %s", path.c_str());
    algorithm::base::SolverState state(inst);
    algorithm::exact::SATSolver solver;
    solver.run(state, 0, rand);

    EXPECT_TRUE(state.resolved());
    int claimed_tww = state.get_upper_bound();
    EXPECT_EQ(claimed_tww, state.get_lower_bound());
    auto actual_tww = ds::graph::TriGraph::verify_contraction_sequence(inst.graph, state.contraction_sequence());
    EXPECT_EQ(claimed_tww, actual_tww);
    EXPECT_LE(claimed_tww, tww);
  }
}

TEST(SATSolverTest, RunWithTinyInstances) {
  char const* pattern = "src/test/resources/tiny-set/%03d/tiny%03d-s%d.gr";

  util::set_log_level(util::logging::LogLevel::NONE);
  verify_instance(pattern, 1, 10, 1);
  verify_instance(pattern, 2, 10, 2);
  verify_instance(pattern, 3, 10, 0);
  verify_instance(pattern, 4, 10, 0);
  // verify_instance(pattern, 5, 25, 3);
  verify_instance(pattern, 6, 10, 0);
  // verify_instance(pattern, 7, 25, 2);
  verify_instance(pattern, 8, 10, 4);
  verify_instance(pattern, 9, 9, 1);
  verify_instance(pattern, 10, 20, 2);
  util::set_log_level(util::logging::LogLevel::TRACE);
}

TEST(SATSolverTest, RunWithSmallInstances) {
  char const* pattern = "src/test/resources/small/%03d/small%03d-s%d.gr";

  util::set_log_level(util::logging::LogLevel::NONE);
  verify_instance(pattern, 1, 8, 1);
  verify_instance(pattern, 2, 18, 2);
  util::set_log_level(util::logging::LogLevel::TRACE);
}

TEST(SATSolverTest, RunWithMissingVertices) {
  util::Random rand(12345);

  auto inst = readwrite::load_pace_extended("src/test/resources/small/002/small002-s8.gr");
  inst.graph.contract(5, 1);
  algorithm::exact::SATSolver solver;

  util::set_log_level(util::logging::LogLevel::NONE);
  algorithm::base::SolverState state({inst.graph, inst.lower_bound_tww, inst.upper_bound_tww});
  solver.run(state, 0, rand);
  util::set_log_level(util::logging::LogLevel::TRACE);

  EXPECT_TRUE(state.resolved());
  int claimed_tww = state.get_upper_bound();
  EXPECT_EQ(claimed_tww, state.get_lower_bound());
  auto actual_tww = ds::graph::TriGraph::verify_contraction_sequence(inst.graph, state.contraction_sequence());
  EXPECT_EQ(claimed_tww, actual_tww);
  EXPECT_EQ(claimed_tww, 2);
}
