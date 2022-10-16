#include "experiments.hh"

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "collisions.hh"
#include "cspace.hh"
#include "fmt/format.h"
#include "goal.hh"
#include "input.hh"
#include "itstar/AITstar.h"
#include "lua_env.hh"
#include "mode_atlas.hh"
#include "motion_validation.hh"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/ScopedState.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/terminationconditions/CostConvergenceTerminationCondition.h"
#include "ompl/geometric/PathGeometric.h"
#include "optimization_objective.hh"
#include "output.hh"
#include "robin_hood.h"
#include "sampler.hh"
#include "scenegraph.hh"
#include "solver.hh"
#include "utils.hh"

namespace experiments {
auto run_experiment(const input::PlanningProblem& problem,
                    const input::ExperimentParameters& experiment,
                    bool should_optimize,
                    unsigned int trial_number,
                    bool should_interpolate,
                    bool save_plans) -> std::vector<output::PlannerSolution> {
  namespace geom = planner::geometric;
  namespace sym  = planner::symbolic;

  auto log = utils::get_logger("exoplanet::experiment");

  // Setup the planning problem:
  // Copy the initial scenegraph
  geom::SceneGraph initial_sg(problem.initial_scene);
  log->debug("Set up initial scenegraph");

  // Construct the mode atlas
  planner::ModeAtlas mode_atlas(problem.domain, problem.symbolic_problem, std::move(initial_sg));
  log->debug("Set up initial mode atlas");

  // Make the symbol index
  robin_hood::unordered_map<std::string, size_t> symbol_indices;
  for (const auto& predicate : problem.domain.predicates) {
    if (predicate->discrete) {
      symbol_indices[predicate->name] = predicate->mode_idx;
    }
  }

  log->debug("Built symbol index");

  // Make OMPL-style workspace bounds
  ompl::base::RealVectorBounds workspace_bounds(2);
  for (size_t i = 0; i < 2; ++i) {
    workspace_bounds.setLow(i, problem.workspace_bounds[i].lower);
    workspace_bounds.setHigh(i, problem.workspace_bounds[i].upper);
  }

  log->debug("Set up workspace bounds");

  // Set up bounds for the predicate formula solver
  std::vector<geom::JointBounds> joint_bounds;
  for (const auto& joint_info : initial_sg.scene_info->joint_info) {
    switch (initial_sg.joints[joint_info.joint_idx].type) {
      case geom::Joint::Type::Continuous:
        joint_bounds.emplace_back(geom::JointBounds{-std::numbers::pi, std::numbers::pi});
        break;
      case geom::Joint::Type::Prismatic:
      case geom::Joint::Type::Revolute:
        joint_bounds.emplace_back(joint_info.bounds ? *joint_info.bounds :
                                                      geom::JointBounds::infinity());
      default:
        break;
    };
  }

  assert(joint_bounds.size() == initial_sg.scene_info->num_controllable_joints);

  planner::SolverBounds solver_bounds{problem.workspace_bounds, joint_bounds};
  log->debug("Set up solver bounds");

  // Construct the Lua evaluation environment
  sym::LuaEnv lua_env(problem.predicate_file_path,
                      initial_sg.scene_info->num_controllable_joints + 3,
                      initial_sg.scene_info->num_movable_objects +
                      initial_sg.scene_info->num_static_objects,
                      initial_sg.scene_info->num_surfaces);
  // Set up predicate functions
  robin_hood::unordered_map<std::string, sym::LuaRef> predicate_function_indices;
  for (const auto& predicate : problem.domain.predicates) {
    if (!predicate->discrete) {
      auto [argspec_idx, argument_types] =
      sym::generate_predicate_argspec(lua_env, predicate->arguments, *initial_sg.scene_info);
      if (!predicate_function_indices.contains(predicate->template_name)) {
        sym::load_predicate(
        lua_env, predicate->template_name, argument_types, *initial_sg.scene_info);
        predicate_function_indices.emplace(predicate->template_name, argspec_idx);
      }

      predicate->fn_idx      = predicate_function_indices[predicate->template_name];
      predicate->argspec_idx = argspec_idx;
    }
  }

  log->debug("Set up Lua environment");

  // Construct and setup the configuration space
  auto cspace = std::make_shared<planner::CompositeStateSpace>(
  &mode_atlas, symbol_indices, &initial_sg, workspace_bounds);
  cspace->setStateSamplerAllocator([&](const ompl::base::StateSpace* state_space) {
    return std::make_shared<planner::CompositeSampler>(
    state_space->as<planner::CompositeStateSpace>(), lua_env, solver_bounds);
  });
  log->debug("Set up configuration space");

  // Construct and setup the space information structure, motion validator, and state validator
  auto space_info = std::make_shared<ompl::base::SpaceInformation>(cspace);
  auto motion_validator =
  std::make_shared<planner::MultimodalMotionValidator>(space_info.get(), &mode_atlas);
  space_info->setMotionValidator(motion_validator);
  log->debug("Set up motion validator");
  auto state_validator = std::make_shared<planner::CollisionChecker>(
  space_info, mode_atlas, initial_sg, problem.workspace_bounds, problem.blocklist_file_path);
  space_info->setStateValidityChecker(state_validator);
  log->debug("Set up collision checker");
  space_info->setup();
  log->debug("Set up space information object");

  // Construct and setup the initial state
  ompl::base::ScopedState<planner::CompositeStateSpace> initial_state(space_info);
  initial_state->set_mode({problem.symbolic_problem.initial_state, 0});
  initial_state->set_base_pose(problem.initial_configuration.base_x,
                               problem.initial_configuration.base_y,
                               problem.initial_configuration.base_yaw);
  initial_state->set_joint_states(problem.initial_configuration.joint_poses);
  log->debug("Set up initial state");

  // Check the validity of the initial state
  if (!space_info->isValid(initial_state.get())) {
    log->error("Initial state is invalid!");
    return {};
  }

  log->debug("Initial state is valid");

  // Construct the goal
  auto goal = std::make_shared<planner::CompositeConstraintGoal>(
  space_info, problem.goal_constraint, lua_env, &mode_atlas, solver_bounds);
  log->debug("Made the goal");

  // Construct and setup the problem definition
  auto optimization_objective =
  std::make_shared<planner::CompositePathLengthOptimizationObjective>(space_info);
  auto problem_definition = std::make_shared<ompl::base::ProblemDefinition>(space_info);
  problem_definition->addStartState(initial_state.get());
  problem_definition->setGoal(goal);
  problem_definition->setOptimizationObjective(optimization_objective);
  log->debug("Set up problem definition");

  // Construct and setup the planner
  auto planner = std::make_shared<ompl::geometric::MultimodalAITstar>(space_info,
                                                                      &mode_atlas,
                                                                      experiment.batch_budget);
  planner->setProblemDefinition(problem_definition);
  planner->setBatchSize(experiment.batch_size);
  planner->enablePruning(false);
  planner->setup();
  log->debug("Set up the planner");

  std::vector<output::PlannerSolution> result;
  std::chrono::system_clock::time_point start_time;

  // Start the timer and solve the problem
  start_time = std::chrono::system_clock::now();
  log->info("Planning for {} seconds...", experiment.max_duration);
  if (should_optimize) {
    planner->solve(ompl::base::plannerOrTerminationCondition(
    ompl::base::CostConvergenceTerminationCondition(problem_definition, 10, 0.1),
    ompl::base::timedPlannerTerminationCondition(experiment.max_duration)));
  } else {
    planner->solve(ompl::base::plannerOrTerminationCondition(
    ompl::base::exactSolnPlannerTerminationCondition(problem_definition),
    ompl::base::timedPlannerTerminationCondition(experiment.max_duration)));
  }

  bool success = problem_definition->hasSolution();

  // Output the solution if we found one
  if (success) {
    auto* solution_path =
    problem_definition->getSolutionPath()->as<ompl::geometric::PathGeometric>();
    if (should_interpolate) {
      log->info("Interpolating solution...");
      output::action_aware_interpolate(solution_path);
    }

    output::output_plan(solution_path,
                        &mode_atlas,
                        std::filesystem::path(fmt::format(
                        fmt::runtime(experiment.plan_output_path_format), trial_number)));
    if (save_plans) {
      log->info("Saving initial plan...");
      auto solutions              = problem_definition->getSolutions();
      auto* initial_solution_path = solutions.back().path_->as<ompl::geometric::PathGeometric>();
      if (should_interpolate) {
        log->info("Interpolating initial solution...");
        output::action_aware_interpolate(initial_solution_path);
      }

      output::output_plan(initial_solution_path,
                          &mode_atlas,
                          std::filesystem::path(fmt::format(
                          fmt::runtime(experiment.initial_plan_output_path_format), trial_number)));
    }
  }

  // Return the results, time-shifted to the start of the experiment
  result.reserve(planner->solutions.size());
  for (const auto [cost, time_point] : planner->solutions) {
    result.emplace_back(output::PlannerSolution{
    cost, std::chrono::duration_cast<std::chrono::milliseconds>(time_point - start_time).count()});
  }

  return result;
}
}  // namespace experiments
