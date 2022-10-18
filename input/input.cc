#include "input.hh"

#include <filesystem>
#include <fstream>
#include <optional>

#include "nlohmann/json.hpp"
#include "pddl.hh"
#include "scene.hh"
#include "state.hh"
#include "utils.hh"

namespace input {
std::optional<PlanningProblem> load_problem(const std::filesystem::path& problem_spec_path) {
  using json = nlohmann::json;
  auto log   = utils::get_logger("tmit-star::input::problem");
  log->debug("Loading problem specification from {}", problem_spec_path.c_str());
  if (!std::filesystem::exists(problem_spec_path)) {
    log->error("No problem specification file at given path!");
    return std::nullopt;
  }

  std::ifstream problem_spec_file(problem_spec_path);
  if (!problem_spec_file.is_open()) {
    log->error("Could not open problem specification file!");
    return std::nullopt;
  }

  json problem_spec;
  problem_spec_file >> problem_spec;
  problem_spec_file.close();
  ENSURE_SPEC_FIELD(problem_spec,
                    "domain",
                    "Problem specification does not specify a symbolic domain!");
  ENSURE_SPEC_FIELD(problem_spec,
                    "problem",
                    "Problem specification does not specify a symbolic problem!");
  ENSURE_SPEC_FIELD(problem_spec,
                    "initial_scene",
                    "Problem specification does not specify an initial scene!");
  ENSURE_SPEC_FIELD(problem_spec,
                    "predicates",
                    "Problem specification does not specify a predicate implementation file!");
  ENSURE_SPEC_FIELD(problem_spec,
                    "collision_blocklist",
                    "Problem specification does not specify a collision blocklist file!");
  const std::filesystem::path predicate_file_path(problem_spec["predicates"].get<std::string>());
  const std::filesystem::path blocklist_file_path(
  problem_spec["collision_blocklist"].get<std::string>());
  auto symbolic_problem_result =
  symbolic::load(std::filesystem::path(problem_spec["domain"].get<std::string>()),
                 std::filesystem::path(problem_spec["problem"].get<std::string>()));
  if (!symbolic_problem_result) {
    return std::nullopt;
  }

  const auto initial_scene_result =
  geometric::load(std::filesystem::path(problem_spec["initial_scene"].get<std::string>()));
  if (!initial_scene_result) {
    return std::nullopt;
  }

  auto& [domain, problem] = *symbolic_problem_result;
  GET_FIELD_WITH_DEFAULT(unsigned int, min_steps, 0, problem_spec, is_number);
  GET_FIELD_WITH_DEFAULT(unsigned int, max_steps, 100, problem_spec, is_number);
  problem.max_steps = max_steps;
  problem.min_steps = min_steps;

  const auto& [workspace_bounds, initial_scenegraph, initial_configuration] = *initial_scene_result;
  return PlanningProblem{problem.name,
                         predicate_file_path,
                         blocklist_file_path,
                         domain,
                         problem,
                         initial_scenegraph,
                         workspace_bounds,
                         initial_configuration,
                         problem.goal};
}

constexpr char default_plan_output_path_format[]         = "{}_trial_{{}}_plan.json";
constexpr char default_initial_plan_output_path_format[] = "{}_trial_{{}}_initial_plan.json";
constexpr char default_statistics_output_path_format[]   = "{}_stats.json";
constexpr unsigned int NUM_TRIALS_DEFAULT                = 1;
constexpr unsigned int BATCH_SIZE_DEFAULT                = 50;
constexpr unsigned int BATCH_BUDGET_DEFAULT              = 5;
constexpr unsigned int MAX_DURATION_DEFAULT              = 100;

std::optional<ExperimentParameters>
load_experiment(const std::filesystem::path& experiment_spec_path, const PlanningProblem& problem) {
  using json = nlohmann::json;
  auto log   = utils::get_logger("tmit-star::input::experiment");
  if (!std::filesystem::exists(experiment_spec_path)) {
    log->error("No experiment specification file at given path!");
    return std::nullopt;
  }

  std::ifstream experiment_spec_file(experiment_spec_path);
  if (!experiment_spec_file.is_open()) {
    log->error("Could not open experiment specification file!");
    return std::nullopt;
  }

  json experiment_spec;
  experiment_spec_file >> experiment_spec;
  experiment_spec_file.close();
  GET_FIELD_WITH_DEFAULT(unsigned int, num_trials, NUM_TRIALS_DEFAULT, experiment_spec, is_number);
  GET_FIELD_WITH_DEFAULT(unsigned int, batch_size, BATCH_SIZE_DEFAULT, experiment_spec, is_number);
  GET_FIELD_WITH_DEFAULT(
  unsigned int, batch_budget, BATCH_BUDGET_DEFAULT, experiment_spec, is_number);
  GET_FIELD_WITH_DEFAULT(
  unsigned int, max_duration, MAX_DURATION_DEFAULT, experiment_spec, is_number);
  GET_FIELD_WITH_DEFAULT(std::string,
                         plan_output_path_format,
                         fmt::format(default_plan_output_path_format, problem.name),
                         experiment_spec,
                         is_string);
  GET_FIELD_WITH_DEFAULT(std::string,
                         initial_plan_output_path_format,
                         fmt::format(default_initial_plan_output_path_format, problem.name),
                         experiment_spec,
                         is_string);
  GET_FIELD_WITH_DEFAULT(std::string,
                         statistics_output_path_format,
                         fmt::format(default_statistics_output_path_format, problem.name),
                         experiment_spec,
                         is_string);

  return ExperimentParameters{num_trials,
                              batch_size,
                              batch_budget,
                              max_duration,
                              plan_output_path_format,
                              initial_plan_output_path_format,
                              statistics_output_path_format};
}

ExperimentParameters make_default_experiment(const PlanningProblem& problem) {
  return ExperimentParameters{NUM_TRIALS_DEFAULT,
                              BATCH_SIZE_DEFAULT,
                              BATCH_BUDGET_DEFAULT,
                              MAX_DURATION_DEFAULT,
                              fmt::format(default_plan_output_path_format, problem.name),
                              fmt::format(default_initial_plan_output_path_format, problem.name),
                              fmt::format(default_statistics_output_path_format, problem.name)};
}
}  // namespace input
