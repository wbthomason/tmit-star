#include "goal.hh"

#include <algorithm>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "cspace.hh"
#include "lua_env.hh"
#include "mode_atlas.hh"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/util/RandomNumbers.h"
#include "scenegraph.hh"
#include "solver.hh"
#include "state.hh"
#include "symbolic.hh"

namespace planner {
CompositeConstraintGoal::CompositeConstraintGoal(const ompl::base::SpaceInformationPtr& si,
                                                 const symbolic::Constraint& goal,
                                                 symbolic::LuaEnv& lua_env,
                                                 ModeAtlas* mode_atlas,
                                                 const SolverBounds& bounds)
: ompl::base::GoalSampleableRegion(si)
, goal(goal)
, goal_symbols(symbolic::bitset_from_pred_list(symbolic::gather_predicates(goal.constraint_formula),
                                               mode_atlas->mode_dims))
, grad_dim(si->getStateSpace()->as<CompositeStateSpace>()->get_num_controllable_joints() + 3)
, lua_env(lua_env)
, mode_atlas(mode_atlas)
, state_vec(grad_dim, 0.0)
, solver(grad_dim) {
  solver.setup_bounds(bounds);
  mode_atlas->add_symbolic_state(goal_symbols);
}

void CompositeConstraintGoal::sampleGoal(ompl::base::State* state) const {
  // NOTE: Assumes that state is randomly sampled before being passed in
  auto* cstate = state->as<CompositeStateSpace::StateType>();
  fill_state_vec(cstate, state_vec);
  const auto random_goal_mode_idx = static_cast<std::size_t>(
  rng.uniformInt(0, mode_atlas->get_symbolic_state_info(goal_symbols).num_modes - 1));
  Mode random_goal_mode{goal_symbols, random_goal_mode_idx};
  const auto& random_goal_sg = mode_atlas->get_mode_info(random_goal_mode).sg;
  do {
    cstate->set_mode(random_goal_mode);
    extract_state(state_vec, cstate);
  } while (!solver.solve(lua_env, goal, random_goal_sg, state_vec) ||
           solver.last_value > symbolic::CONSTRAINT_TOLERANCE);
}

unsigned int CompositeConstraintGoal::maxSampleCount() const {
  if (valid_symbols_reached) {
    return std::numeric_limits<unsigned int>::max();
  }

  return 0;
}

double CompositeConstraintGoal::distanceGoal(const ompl::base::State* state) const {
  const auto* cstate = state->as<CompositeStateSpace::StateType>();
  const auto& mode   = cstate->get_mode();
  if ((goal_symbols & mode.symbols) == goal_symbols) {
    valid_symbols_reached = true;
    return 0.0;
  } else {
    return std::numeric_limits<double>::infinity();
  }
}
}  // namespace planner
