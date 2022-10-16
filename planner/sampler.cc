#include "sampler.hh"

#include "cspace.hh"
#include "lua_env.hh"
#include "ompl/base/State.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "solver.hh"
#include "symbolic.hh"

namespace planner {

CompositeSampler::CompositeSampler(const CompositeStateSpace* state_space,
                                   symbolic::LuaEnv& lua_env,
                                   const SolverBounds& bounds)
: ompl::base::StateSampler(state_space)
, mode_atlas(state_space->mode_atlas)
, base_pose_sampler(
  state_space->as<ompl::base::SE2StateSpace>(BASE_SPACE_IDX)->allocDefaultStateSampler())
, joint_state_sampler(
  state_space->as<KinematicTreeStateSpace>(JOINT_SPACE_IDX)->allocDefaultStateSampler())
, state_vec(state_space->get_num_controllable_joints() + 3, 0.0)
, lua_env(lua_env)
, solver(state_space->get_num_controllable_joints() + 3) {
  solver.setup_bounds(bounds);
}

void CompositeSampler::sampleUniformNear(ompl::base::State* state,
                                         const ompl::base::State* near,
                                         double distance) {
  auto* cstate = state->as<CompositeStateSpace::StateType>();
  auto* cnear  = near->as<CompositeStateSpace::StateType>();

  // Duplicate the symbolic state
  cstate->set_mode(cnear->get_mode());

  // Sample robot state
  base_pose_sampler->sampleUniformNear(
  cstate->as<ompl::base::SE2StateSpace::StateType>(BASE_SPACE_IDX),
  cnear->as<ompl::base::SE2StateSpace::StateType>(BASE_SPACE_IDX),
  distance);
  joint_state_sampler->sampleUniformNear(
  cstate->as<KinematicTreeStateSpace::StateType>(JOINT_SPACE_IDX),
  cnear->as<KinematicTreeStateSpace::StateType>(JOINT_SPACE_IDX),
  distance);
}

void CompositeSampler::sampleGaussian(ompl::base::State* state,
                                      const ompl::base::State* mean,
                                      double stdDev) {
  auto* cstate = state->as<CompositeStateSpace::StateType>();
  auto* cmean  = mean->as<CompositeStateSpace::StateType>();

  // Duplicate the symbolic state
  cstate->set_mode(cmean->get_mode());

  // Sample robot state
  base_pose_sampler->sampleGaussian(cstate->as<ompl::base::SE2StateSpace::StateType>(
                                    BASE_SPACE_IDX),
                                    cmean->as<ompl::base::SE2StateSpace::StateType>(BASE_SPACE_IDX),
                                    stdDev);
  joint_state_sampler->sampleGaussian(
  cstate->as<KinematicTreeStateSpace::StateType>(JOINT_SPACE_IDX),
  cmean->as<KinematicTreeStateSpace::StateType>(JOINT_SPACE_IDX),
  stdDev);
}

void CompositeSampler::sampleUniform(ompl::base::State* state) {
  auto* cstate = state->as<CompositeStateSpace::StateType>();

  // The planner should've filled in the mode for us already. Get its metadata
  const auto& symbolic_state_info = mode_atlas->get_symbolic_state_info(cstate->get_mode().symbols);

  // Sample uniformly in that mode
  cstate->mode.pose_set = rng_.uniformInt(0, symbolic_state_info.num_modes - 1);
  base_pose_sampler->sampleUniform(
  cstate->as<ompl::base::SE2StateSpace::StateType>(BASE_SPACE_IDX));
  joint_state_sampler->sampleUniform(
  cstate->as<KinematicTreeStateSpace::StateType>(JOINT_SPACE_IDX));
}

bool CompositeSampler::sample_transition_near(ompl::base::State* state,
                                              const ompl::base::State* near,
                                              const Transition* transition) {
  const auto* cnear     = near->as<CompositeStateSpace::StateType>();
  const auto& mode_info = mode_atlas->get_mode_info(cnear->get_mode());

  // Solve for a precondition-satisfying state starting at the given state
  fill_state_vec(cnear, state_vec);
  if (solver.solve(lua_env, transition->action->precondition, mode_info.sg, state_vec) &&
      solver.last_value <= symbolic::CONSTRAINT_TOLERANCE) {
    // Copy out the resulting state
    auto* cstate          = state->as<CompositeStateSpace::StateType>();
    const auto& from_mode = cnear->get_mode();
    cstate->set_mode(from_mode);
    cstate->transition.emplace(*transition);
    extract_state(state_vec, cstate);
    return true;
  }

  return false;
}

double CompositeSampler::transition_distance(const ompl::base::State* state,
                                             const std::vector<geometric::Transform3<>>& poses,
                                             const Transition* transition) {
  const auto* cstate = state->as<CompositeStateSpace::StateType>();
  const auto& mode   = cstate->get_mode();
  const auto& sg     = mode_atlas->get_mode_info(mode).sg;
  return transition->action->precondition.distance_at(
  lua_env, symbolic::PosedScene<geometric::Transform3<>>(sg, poses));
}

UninformedSampler::UninformedSampler(const ompl::base::ProblemDefinitionPtr& problem_def,
                                     unsigned int max_number_calls)
: ompl::base::InformedSampler(problem_def, max_number_calls)
, base_sampler(space_->allocStateSampler()) {}

bool UninformedSampler::sampleUniform(ompl::base::State* state, const ompl::base::Cost& max_cost) {
  // We ignore the cost because we're uninformed
  base_sampler->sampleUniform(state);
  return true;
}

bool UninformedSampler::sampleUniform(ompl::base::State* state,
                                      const ompl::base::Cost& min_cost,
                                      const ompl::base::Cost& max_cost) {
  // We ignore the cost because we're uninformed
  base_sampler->sampleUniform(state);
  return true;
}

bool UninformedSampler::hasInformedMeasure() const { return false; }
double UninformedSampler::getInformedMeasure(const ompl::base::Cost& current_cost) const {
  return space_->getMeasure();
}

double UninformedSampler::getInformedMeasure(const ompl::base::Cost& min_cost,
                                             const ompl::base::Cost& max_cost) const {
  return space_->getMeasure();
}
bool UninformedSampler::sample_transition_near(ompl::base::State* state,
                                               const ompl::base::State* near,
                                               const Transition* transition) {
  return static_cast<CompositeSampler*>(base_sampler.get())
  ->sample_transition_near(state, near, transition);
}

double UninformedSampler::transition_distance(const ompl::base::State* state,
                                              const std::vector<geometric::Transform3<>>& poses,
                                              const Transition* transition) const {
  return static_cast<CompositeSampler*>(base_sampler.get())
  ->transition_distance(state, poses, transition);
}
}  // namespace planner
