#include <memory>

#include "algorithm/ExactSolver.hpp"
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
  log_info("Configuration: seed=%u, time-limit=%s", conf.seed,
           conf.time_limit > 0 ? util::format("%ds", conf.time_limit).c_str() : "N/A");

  // load input file(s)
  ds::ProblemInstance instance;
  try {
    instance = readwrite::load_pace_extended(conf.input_path.c_str());
  } catch (std::invalid_argument const& e) {
    log_critical("File format error: %s", e.what());
    return 2;
  }

  log_info("Loaded graph (n=%ld, m=%ld, m_red=%ld): %s", instance.graph.number_of_vertices(),
           instance.graph.number_of_edges(), instance.graph.number_of_red_edges(), conf.input_path.c_str());
  log_info("Parameters: lb=%d, ub=%d", instance.lower_bound_tww, instance.upper_bound_tww);

  // create the solver
  algorithm::ExactSolver solver(instance.graph, &root_timer);

  int status_code = 0;
  try {
    solver.run(rand, instance.lower_bound_tww, instance.upper_bound_tww);
    if (solver.resolved()) {
      // verification
      // log_debug("constr_seq: %s", cstr(solver.contraction_sequence()));
      int tww = ds::graph::TriGraph::verify_contraction_sequence(instance.graph, solver.contraction_sequence());

      if (tww == solver.twin_width()) {
        // ok
        log_success("Found a solution: tww=%d, elapsed=%.3fs", solver.twin_width(), root_timer.stop());

        // output results
        if (conf.tww_mode) {
          // twin-width
          printf("%d\n", solver.twin_width());
        } else {
          // contraction sequence
          for (auto p : solver.contraction_sequence()) {
            printf("%d %d\n", instance.graph.get_label(p.first), instance.graph.get_label(p.second));
          }
        }
      } else {
        log_critical("Twin-width does not match: claimed=%d, actual=%d", solver.twin_width(), tww);
        status_code = 2;
      }
    } else {
      log_critical("Failed to find a solution: elapsed=%.3fs", root_timer.stop());
      status_code = 2;
    }
  } catch (std::invalid_argument const& e) {
    log_critical("Solver error: invalid_argument: %s", e.what());
    status_code = 2;
  } catch (std::exception const& e) {  //
    log_critical("Solver error: %s", e.what());
    status_code = 2;
  }

  return status_code;
}
