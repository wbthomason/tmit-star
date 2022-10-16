#ifndef OPTIMIZATION_OBJECTIVE_HH
#define OPTIMIZATION_OBJECTIVE_HH
#include <vector>

#include "cspace.hh"
#include "ompl/base/Cost.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/State.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"

namespace planner {
struct CompositePathLengthOptimizationObjective
: public ompl::base::PathLengthOptimizationObjective {
  CompositePathLengthOptimizationObjective(const ompl::base::SpaceInformationPtr& si)
  : ompl::base::PathLengthOptimizationObjective(si)
  , space(static_cast<const CompositeStateSpace*>(si->getStateSpace()->as<CompositeStateSpace>())) {
  }
  ompl::base::Cost motionCostHeuristic(const ompl::base::State* const state_1,
                                       const ompl::base::State* const state_2) const override;

 protected:
  const CompositeStateSpace* const space;
};
};  // namespace planner
#endif
