#include <z3++.h>

#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "cxxopts.hpp"
#include "date.h"
#include "experiments.hh"
#include "input.hh"
#include "nlohmann/json.hpp"
#include "output.hh"
#include "spdlog/spdlog.h"

#define FAIL(msg)                               \
  {                                             \
    spdlog::set_pattern("%^[tmit-star]:%$ %v"); \
    log->error(msg);                            \
    return EXIT_FAILURE;                        \
  }

// Evaluate an expression that returns an optional-like value (coerces to
// boolean, uses *to access a held value if it exists), then either print an
// error message and quit the program or return the held value
#define TRY(try_expr, msg)  \
  ({                        \
    auto result = try_expr; \
    if (!result) {          \
      FAIL(msg);            \
    }                       \
    *result;                \
  })

std::function<void(int)> sigint_handler_impl;
void sigint_handler(int signum) { sigint_handler_impl(signum); }

int main(int argc, char** argv) {
  // Set up argument parsing and logging
  cxxopts::Options options("tmit-star", "ASAO TAMP");
  // clang-format off
  options.add_options()
    ("p,problem_spec", "Problem specification file path", cxxopts::value<std::string>())
    ("e,experiment_spec", "Experiment specification file path", cxxopts::value<std::string>())
    ("o,optimize", "Perform optimizing planning")
    ("i,interpolate", "Interpolate solutions before output")
    ("s,save-plans", "Save intermediate plans while optimizing")
    ("v,verbose", "Set verbosity level")
    ("h,help", "Display this help message");
  // clang-format on
  options.parse_positional({"problem_spec"});
  options.positional_help("[path to problem specification file]");
  auto cmdline_args = options.parse(argc, argv);
  if (cmdline_args.count("help") > 0) {
    std::cout << options.help() << std::endl;
    return 0;
  }

  spdlog::set_pattern("[%H:%M:%S] %^[%l @ %n]%$ %v");
  auto log = utils::get_logger("tmit-star");

  const auto v_count = cmdline_args.count("verbose");
  if (v_count == 0) {
    spdlog::set_level(spdlog::level::info);
  } else if (v_count == 1) {
    spdlog::set_level(spdlog::level::debug);
  } else if (v_count > 1) {
    spdlog::set_level(spdlog::level::trace);
  }

  z3::set_param("tactic.default_tactic", "sat");
  z3::set_param("sat.euf", false);

  // Load the problem
  if (cmdline_args.count("problem_spec") != 1) {
    FAIL("Please provide exactly one problem specification file path!");
  }

  const auto maybe_problem =
  input::load_problem(std::filesystem::path(cmdline_args["problem_spec"].as<std::string>()));
  if (!maybe_problem) {
    return EXIT_FAILURE;
  }

  const auto& problem = *maybe_problem;
  log->info("Loaded problem {} in domain {} with {} predicates and {} actions",
            problem.name,
            problem.domain.name,
            problem.domain.predicates.size(),
            problem.domain.actions.size());

  log->info("Loaded scene with {} movable objects, {} static objects, {} surfaces, and {} robot "
            "joints ({} of which are controllable)",
            problem.initial_scene.scene_info->num_movable_objects,
            problem.initial_scene.scene_info->num_static_objects,
            problem.initial_scene.scene_info->num_surfaces,
            problem.initial_scene.scene_info->num_robot_joints,
            problem.initial_scene.scene_info->num_controllable_joints);

  // Load the experiment parameters
  input::ExperimentParameters experiment;
  if (cmdline_args.count("experiment_spec") == 1) {
    const auto maybe_experiment =
    input::load_experiment(std::filesystem::path(cmdline_args["experiment_spec"].as<std::string>()),
                           problem);
    if (maybe_experiment) {
      experiment = *maybe_experiment;
    } else {
      return EXIT_FAILURE;
    }
  } else if (cmdline_args.count("experiment_spec") == 0) {
    experiment = input::make_default_experiment(problem);
  } else {
    FAIL(
    "Please provide either no experiment parameter file path (to use defaults) or exactly one!");
  }

  const auto should_optimize    = cmdline_args.count("optimize") > 0;
  const auto should_interpolate = cmdline_args.count("interpolate") > 0;
  const auto save_plans         = cmdline_args.count("save-plans") > 0;

  // Run the desired number of iterations of the experiment
  std::vector<std::vector<output::PlannerSolution>> results;
  results.reserve(experiment.num_trials);
  bool caught_sigint        = false;
  const auto output_results = [&]() {
    nlohmann::json results_json;
    results_json["problem_name"] = problem.name;
    results_json["trial_data"]   = results;
    std::filesystem::path output_filepath(fmt::format(
    "{}_trials_{}.json", problem.name, date::format("%F_%T", std::chrono::system_clock::now())));
    std::ofstream output_file(output_filepath);
    if (!output_file.is_open()) {
      FAIL("Couldn't open output file!");
    }

    output_file << std::setw(4) << results_json << std::endl;
    output_file.close();
    return 0;
  };

  // Set up the signal handler for SIGINT
  sigint_handler_impl = [&](int signum) {
    if (!caught_sigint) {
      caught_sigint = true;
      output_results();
      exit(signum);
    }
  };

  signal(SIGINT, sigint_handler);
  for (unsigned int experiment_iter = 1; experiment_iter <= experiment.num_trials;
       ++experiment_iter) {
    log->info("Running trial {} of {}...", experiment_iter, experiment.num_trials);
    results.emplace_back(experiments::run_experiment(
    problem, experiment, should_optimize, experiment_iter, should_interpolate, save_plans));
  }

  output_results();
  return 0;
}
