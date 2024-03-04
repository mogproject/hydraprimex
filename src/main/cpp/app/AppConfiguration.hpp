#pragma once

#include <CLI11/CLI11.hpp>

#include "util/logger.hpp"
#include "util/util.hpp"

namespace app {

char const* PROGRAM_NAME = "hydraprimex";
char const* PROGRAM_VERSION = "0.1.0-SNAPSHOT";

/**
 * @brief Handles command-line arguments and configurations.
 */
class AppConfiguration {
 public:
  bool exit = false;  // Immediately exit after parsing.
  int status_code = 0;

  // parameters and flags
  bool print_version = false;

  util::logging::LogLevel log_level = util::logging::LogLevel::INFO;
  bool disable_colored_log = false;

  std::string input_path;
  uint32_t seed = 12345;
  double scale = 1.0;
  int time_limit = 0;
  bool tww_mode = false;

  AppConfiguration(int argc, char* argv[]) {
    // parse arguments
    CLI::App app{"Solver for Annotated Twin-Width"};

    app.add_option("path", input_path, "Input file path")->option_text("INPUT_PATH");
    app.add_flag("-v,--version", print_version, "Print program's version number and exit");
    app.add_option("-l,--log-level", log_level, util::format("Set log level to one of [%s]", log_level_options))
        ->option_text("LEVEL")
        ->transform(CLI::CheckedTransformer(util::logging::log_level_map, CLI::ignore_case));
    app.add_flag("--no-color", disable_colored_log, "Disable colored logging");
    app.add_option("--seed", seed, "Random seed")->option_text("SEED");
    app.add_option("--time-limit", time_limit, "Time limit in seconds")->option_text("TIME_LIMIT_SEC");
    app.add_flag("--tww", tww_mode, "Print twin-width instead of contraction sequence");

    try {
      app.parse(argc, argv);

      if (print_version) {  // show version
#ifdef NDEBUG
        printf("%s %s\n", PROGRAM_NAME, PROGRAM_VERSION);
#else
        printf("%s %s-DEBUG\n", PROGRAM_NAME, PROGRAM_VERSION);
#endif
        exit = true;
      } else if (input_path.empty()) {  // path is required
        printf("path is required\n");
        printf("Run with --help for more information.\n");
        exit = true;
        status_code = 106;  // This code is consistent with CLI11.
      }
    } catch (CLI::ParseError const& e) {
      exit = true;
      status_code = app.exit(e);
    }
  }

 private:
#ifdef NDEBUG
  char const* log_level_options = "none, success, critical, error, warning, info, debug";
#else
  char const* log_level_options = "none, success, critical, error, warning, info, debug, trace";
#endif
};
}  // namespace app
