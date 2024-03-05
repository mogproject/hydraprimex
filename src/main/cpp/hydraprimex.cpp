#include <memory>

#include "algorithm/ExactSolver.hpp"
#include "algorithm/base/SolverState.hpp"
#include "app/AppConfiguration.hpp"
#include "readwrite/pace_extended.hpp"
#include "util/Random.hpp"
#include "util/Timer.hpp"
#include "util/logger.hpp"
#include "util/util.hpp"

using namespace std;

/**
 * @brief Entry point of the program.
 *
 * @param argc argument count
 * @param argv argument strings
 * @return int status code
 */
int main(int argc, char* argv[]) {
  util::Timer root_timer;

  // parse args
  app::AppConfiguration conf(argc, argv);
  if (conf.exit) return conf.status_code;

  // configure logging
  util::set_log_level(conf.log_level);  // log level
  util::set_colored_log(!conf.disable_colored_log);

  // set time limit
  root_timer.set_time_limit(conf.time_limit);

  // pseudorandom number generator
  util::Random rand(conf.seed);
  log_info("Solver started: seed=%u, time-limit=%s", conf.seed,
           conf.time_limit > 0 ? util::format("%ds", conf.time_limit).c_str() : "N/A");

  // load input file(s)
  ds::ProblemInstance instance;
  try {
    instance = readwrite::load_pace_extended(conf.input_path.c_str());
  } catch (std::invalid_argument const& e) {
    log_error("File format error: %s", e.what());
    return 2;
  }

  log_info("Loaded graph (n=%ld, m=%ld, m_red=%ld, lb=%d, ub=%s): %s", instance.graph.number_of_vertices(),
           instance.graph.number_of_edges(), instance.graph.number_of_red_edges(), instance.lower_bound_tww,
           instance.upper_bound_tww < 0 ? "INF" : std::to_string(instance.upper_bound_tww).c_str(), conf.input_path.c_str());

  // create the solver
  algorithm::base::SolverState sstate(instance);
  algorithm::ExactSolver solver(&root_timer);

  int status_code = 0;
  try {
    solver.run(sstate, -1, rand);

    if (sstate.resolved()) {
      auto cs = sstate.contraction_sequence();
      int tww = sstate.get_upper_bound();

      if (cs.size() + 1 != instance.graph.number_of_vertices()) {
        if (tww == instance.upper_bound_tww) {
          log_success("Given upper bound is optimal (no output): tww=%d, elapsed=%.3fs", tww, root_timer.stop());
        } else {
          // something is wrong
          log_critical("Possible bug: elapsed=%.3fs, cs=%s", root_timer.stop(), cstr(cs));
          status_code = 3;
        }
      } else {
        // verification
        int actual_tww = ds::graph::TriGraph::verify_contraction_sequence(instance.graph, cs);

        if (tww == actual_tww) {
          // ok
          if (tww < instance.lower_bound_tww) {
            // given lower bound might be wrong
            log_warning("Contraction width is below the given lower bound: tww=%d, lb=%d", tww, instance.lower_bound_tww);
          }

          log_success("Found a solution: tww=%d, elapsed=%.3fs", tww, root_timer.stop());

          // output results
          if (conf.tww_mode) {
            // twin-width
            printf("%d\n", tww);
          } else {
            // contraction sequence
            for (auto p : cs) { printf("%d %d\n", p.first, p.second); }
          }
        } else {
          log_critical("Twin-width does not match: claimed=%d, actual=%d", tww, actual_tww);
          status_code = 2;
        }
      }
    } else {
      // not resolved
      log_error("Could not find a solution: elapsed=%.3fs", root_timer.stop());
      status_code = 2;
    }
  } catch (std::invalid_argument const& e) {
    log_critical("Solver error: invalid_argument: %s", e.what());
    status_code = 3;
  } catch (std::exception const& e) {  //
    log_critical("Solver error: %s", e.what());
    status_code = 3;
  }

  return status_code;
}
