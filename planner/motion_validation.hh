#ifndef MOTION_VALIDATION_HH
#define MOTION_VALIDATION_HH
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>

#include <utility>

#include "mode_atlas.hh"

namespace planner {
struct MultimodalMotionValidator : public ompl::base::DiscreteMotionValidator {
  MultimodalMotionValidator(ompl::base::SpaceInformation* si, const ModeAtlas* mode_atlas)
  : ompl::base::DiscreteMotionValidator(si), mode_atlas(mode_atlas) {}
  bool checkMotion(const ompl::base::State* start, const ompl::base::State* end) const override;
  bool checkMotion(const ompl::base::State* start, const ompl::base::State* end, std::pair<ompl::base::State*, double>& last_valid) const override;

 protected:
  const ModeAtlas* const mode_atlas;
};
}  // namespace planner
#endif
