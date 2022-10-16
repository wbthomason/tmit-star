#ifndef INPUT_HH
#define INPUT_HH
#include <array>
#include <filesystem>
#include <optional>

#include "bounds.hh"
#include "pddl.hh"
#include "scene.hh"
#include "scenegraph.hh"
#include "state.hh"
#include "symbolic.hh"

namespace input {
struct PlanningProblem {
  std::string name;
  std::filesystem::path predicate_file_path;
  std::filesystem::path blocklist_file_path;
  planner::symbolic::Domain domain;
  planner::symbolic::Problem symbolic_problem;
  planner::geometric::SceneGraph initial_scene;
  std::array<planner::geometric::Bounds, 3> workspace_bounds;
  planner::geometric::State<> initial_configuration;
  planner::symbolic::Constraint goal_constraint;
};

struct ExperimentParameters {
  unsigned int num_trials;
  unsigned int batch_size;
  unsigned int batch_budget;
  unsigned int max_duration;
  std::string plan_output_path_format;
  std::string initial_plan_output_path_format;
  std::string statistics_output_path;
};

std::optional<PlanningProblem> load_problem(const std::filesystem::path& problem_spec_path);
std::optional<ExperimentParameters>
load_experiment(const std::filesystem::path& experiment_spec_path, const PlanningProblem& problem);
ExperimentParameters make_default_experiment(const PlanningProblem& problem);
}  // namespace input
#endif
