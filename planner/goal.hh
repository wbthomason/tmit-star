#ifndef GOAL_HH
#define GOAL_HH

#include <memory>
#include <vector>

#include "lua_env.hh"
#include "mode.hh"
#include "mode_atlas.hh"
#include "ompl/base/Goal.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/State.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/util/RandomNumbers.h"
#include "scenegraph.hh"
#include "solver.hh"
#include "state.hh"
#include "symbolic.hh"

namespace planner {
struct CompositeConstraintGoal : public ompl::base::GoalSampleableRegion {
  CompositeConstraintGoal(const ompl::base::SpaceInformationPtr& si,
                          const symbolic::Constraint& goal,
                          symbolic::LuaEnv& lua_env,
                          ModeAtlas* mode_atlas,
                          const SolverBounds& bounds);
  void sampleGoal(ompl::base::State* st) const override;
  unsigned int maxSampleCount() const override;
  double distanceGoal(const ompl::base::State* st) const override;
  const Mode::SymbolicStateT& get_symbols() const { return goal_symbols; }

 protected:
  const symbolic::Constraint& goal;
  const Mode::SymbolicStateT goal_symbols;
  const size_t grad_dim;
  symbolic::LuaEnv& lua_env;
  ModeAtlas* const mode_atlas;
  mutable std::vector<double> state_vec;
  mutable FormulaSolver solver;
  mutable ompl::RNG rng;
  mutable bool valid_symbols_reached{false};
};
}  // namespace planner
#endif
