#include "solver.hh"

#include <algorithm>
#include <nlopt.hpp>
#include <numbers>

#include "cspace.hh"
#include "lua_env.hh"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "scenegraph.hh"
#include "state.hh"
#include "symbolic.hh"

namespace planner {

namespace {
  double objective_fn(const std::vector<double>& x, std::vector<double>& grad, void* f_data) {
    auto* opt_data = static_cast<OptData*>(f_data);
    // TODO: Find a way to avoid the repetition in choosing the right state field to use
    if (!grad.empty()) {
      auto& state         = opt_data->diff_state;
      state.base_x        = x[0];
      state.base_y        = x[1];
      state.base_yaw      = x[2];
      auto joint_state_it = x.begin() + 3;
      std::copy(joint_state_it, x.end(), state.joint_poses.begin());
      // TODO: See if it would be more efficient to avoid this clear by having the pose functions
      // not use emplace_back
      opt_data->pose_partial_derivatives.clear();
      geometric::pose_partial_derivatives(*opt_data->sg, state, opt_data->pose_partial_derivatives);
      const auto result =
      opt_data->formula->gradient_at(*opt_data->lua_env,
                                     {*opt_data->sg, opt_data->pose_partial_derivatives},
                                     Eigen::Map<Eigen::VectorXd>(grad.data(), grad.size()));
      return result;
    } else {
      auto& state         = opt_data->state;
      state.base_x        = x[0];
      state.base_y        = x[1];
      state.base_yaw      = x[2];
      auto joint_state_it = x.begin() + 3;
      std::copy(joint_state_it, x.end(), state.joint_poses.begin());
      // TODO: See if it would be more efficient to avoid this clear by having the pose functions
      // not use emplace_back
      opt_data->poses.clear();
      geometric::pose_all(*opt_data->sg, state, opt_data->poses);
      const auto result =
      opt_data->formula->distance_at(*opt_data->lua_env, {*opt_data->sg, opt_data->poses});
      return result;
    }
  }
}  // namespace

FormulaSolver::FormulaSolver(size_t grad_len)
: optimizer(nlopt::AUGLAG, grad_len)
, local_optimizer(nlopt::LD_SLSQP, grad_len)
// TODO: Don't just assume a fixed gradient of joints + 3dim for base pose
, opt_data(grad_len - 3)
, lower_bounds(grad_len, 0.0)
, upper_bounds(grad_len, 0.0) {
  optimizer.set_min_objective(objective_fn, static_cast<void*>(&opt_data));
  optimizer.set_stopval(1e-3);
  optimizer.set_ftol_abs(1e-5);
  local_optimizer.set_stopval(1e-3);
  local_optimizer.set_ftol_abs(1e-5);
  optimizer.set_local_optimizer(local_optimizer);
}

bool FormulaSolver::solve(symbolic::LuaEnv& lua_env,
                          const symbolic::Constraint& precondition,
                          const geometric::SceneGraph& sg,
                          std::vector<double>& state_vec) {
  // Fill the struct of additional function parameters
  opt_data.lua_env = &lua_env;
  opt_data.sg      = &sg;
  opt_data.formula = &precondition;

  // Run optimization
  const auto optimization_result = optimizer.optimize(state_vec, last_value);
  if (optimization_result <= 0) {
    return false;
  }

  return true;
}

void FormulaSolver::setup_bounds(const SolverBounds& bounds) {
  const auto& [base_bounds, joint_bounds] = bounds;
  // Bound the base (x, y, yaw)
  lower_bounds[0] = base_bounds[0].lower;
  lower_bounds[1] = base_bounds[1].lower;
  lower_bounds[2] = -std::numbers::pi;

  upper_bounds[0] = base_bounds[0].upper;
  upper_bounds[1] = base_bounds[1].upper;
  upper_bounds[2] = std::numbers::pi;

  // Bound the joints
  for (size_t i = 0; i < joint_bounds.size(); ++i) {
    lower_bounds[3 + i] = joint_bounds[i].lower;
    upper_bounds[3 + i] = joint_bounds[i].upper;
  }

  // Apply the bounds
  optimizer.set_lower_bounds(lower_bounds);
  optimizer.set_upper_bounds(upper_bounds);
  local_optimizer.set_lower_bounds(lower_bounds);
  local_optimizer.set_upper_bounds(upper_bounds);
}


void fill_state_vec(const CompositeStateSpace::StateType* const state,
                    std::vector<double>& state_vec) {
  const auto* base_state         = state->as<ompl::base::SE2StateSpace::StateType>(BASE_SPACE_IDX);
  const auto* joints_state       = state->as<KinematicTreeStateSpace::StateType>(JOINT_SPACE_IDX);
  state_vec[0]                   = base_state->getX();
  state_vec[1]                   = base_state->getY();
  state_vec[2]                   = base_state->getYaw();
  const auto* const joint_states = joints_state->get_joint_states();
  std::copy(joint_states, joint_states + joints_state->num_joints, state_vec.begin() + 3);
}

void extract_state(const std::vector<double>& state_vec,
                   CompositeStateSpace::StateType* const state) {
  auto* base_state = state->as<ompl::base::SE2StateSpace::StateType>(BASE_SPACE_IDX);
  base_state->setXY(state_vec[0], state_vec[1]);
  base_state->setYaw(state_vec[2]);
  auto* joint_state = state->as<KinematicTreeStateSpace::StateType>(JOINT_SPACE_IDX);
  joint_state->set_joint_states(state_vec.begin() + 3, state_vec.end());
}
}  // namespace planner
