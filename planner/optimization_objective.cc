#include "optimization_objective.hh"

#include "cspace.hh"
#include "ompl/base/Cost.h"
#include "ompl/base/State.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"

namespace planner {
ompl::base::Cost CompositePathLengthOptimizationObjective::motionCostHeuristic(
const ompl::base::State* const state_1, const ompl::base::State* const state_2) const {
  // Because our "true" distance is a conservative estimate that can return infinity, we need to
  // return something else here to make the heuristic admissible
  // NOTE: I have removed the transition_graph heuristic here because I don't yet know how to make
  // it admissible/consistent. If we don't know of a transition between two modes, then I'm not sure
  // what other than "infinity" to return as the heuristic distance, and some part of AIT*'s search
  // requests distances between impossible transitions (I think distanceToStart?)
  const auto* cstate_1 = state_1->as<CompositeStateSpace::StateType>();
  const auto* cstate_2 = state_2->as<CompositeStateSpace::StateType>();
  auto distance        = space->continuous_distance(cstate_1, cstate_2);
  return ompl::base::Cost(distance);
}
}  // namespace planner
