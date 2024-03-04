#include <gtest/gtest.h>

#include "algorithm/ExactSolver.hpp"
#include "readwrite/pace_extended.hpp"

using namespace std;

void verify_instance(std::string const& path, int tww, util::Random& rand) {
  auto inst = readwrite::load_pace_extended(path.c_str());
  algorithm::ExactSolver solver(inst.graph);
  solver.run(rand, inst.lower_bound_tww, inst.upper_bound_tww);

  EXPECT_TRUE(solver.resolved());
  EXPECT_LE(solver.twin_width(), tww);
  auto red_deg = ds::graph::TriGraph::verify_contraction_sequence(inst.graph, solver.contraction_sequence());
  EXPECT_EQ(solver.twin_width(), red_deg);
}

TEST(ExactSolverTest, RunWithSmallInstances) {
  char const* prefix = "src/test/resources/small";

  util::Random rand(12345);

  util::set_log_level(util::logging::LogLevel::NONE);
  for (int s = 1; s < 7; ++s) {
    verify_instance(util::format("%s/001/small001-s%d.gr", prefix, s), 1, rand);
    verify_instance(util::format("%s/002/small002-s%d.gr", prefix, s), 2, rand);
    verify_instance(util::format("%s/003/small003-s%d.gr", prefix, s), 6, rand);
  }

  verify_instance(util::format("%s/007/small007.gr", prefix), 3, rand);
  util::set_log_level(util::logging::LogLevel::TRACE);
}
