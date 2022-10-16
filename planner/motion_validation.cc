#include "motion_validation.hh"

#include <ompl/base/State.h>

#include <utility>
#include <vector>

#include "cspace.hh"

namespace planner {
bool is_transition_state(const CompositeStateSpace::StateType* state, const Mode& to_mode) {
  return state->transition && state->transition_mode == to_mode;
}

bool MultimodalMotionValidator::checkMotion(const ompl::base::State* start, const ompl::base::State* end) const {
  if (!si_->isValid(end)) {
    invalid_++;
    return false;
  }

  const auto* cstart     = start->as<CompositeStateSpace::StateType>();
  const auto* cend       = end->as<CompositeStateSpace::StateType>();
  const auto& start_mode = cstart->get_mode();
  const auto& end_mode   = cend->get_mode();

  // NOTE: We could additionally check that cend satisfies the precondition for some action
  // transitioning from start_mode to end_mode, but that is likely not worth the trouble
  auto valid_transition = (start_mode == end_mode) || is_transition_state(cstart, end_mode);
  auto result      = true;
  const auto space = si_->getStateSpace();
  unsigned int nd  = space->validSegmentCount(start, end);
  std::vector<std::pair<unsigned int, unsigned int>> test_positions;
  test_positions.reserve(nd);
  size_t test_position_index = 0;
  if (nd >= 2) {
    test_positions.emplace_back(1, nd - 1);
    auto* test = si_->allocState()->as<CompositeStateSpace::StateType>();
    while (test_position_index < test_positions.size()) {
      std::pair<int, int> x = test_positions[test_position_index];
      int mid               = (x.first + x.second) / 2;
      space->interpolate(start, end, static_cast<double>(mid) / static_cast<double>(nd), test);
      if (!si_->isValid(test)) {
        result = false;
        break;
      }

      // NOTE: Similarly, we might want to check intermediate states for satisfying a transitioning
      // action's precondition here
      ++test_position_index;
      if (x.first < mid) {
        test_positions.emplace_back(x.first, mid - 1);
      }

      if (x.second > mid) {
        test_positions.emplace_back(mid + 1, x.second);
      }
    }

    si_->freeState(test);
  }

  result &= valid_transition;

  if (result) {
    valid_++;
  } else {
    invalid_++;
  }

  return result;
}

bool MultimodalMotionValidator::checkMotion(const ompl::base::State* start,
                                            const ompl::base::State* end,
                                            std::pair<ompl::base::State*, double>& last_valid) const {
  const auto* cstart     = start->as<CompositeStateSpace::StateType>();
  const auto* cend       = end->as<CompositeStateSpace::StateType>();
  const auto& start_mode = cstart->get_mode();
  const auto& end_mode   = cend->get_mode();

  // NOTE: We could additionally check that cend satisfies the precondition for some action
  // transitioning from start_mode to end_mode, but that is likely not worth the trouble
  auto valid_transition = (start_mode == end_mode) || is_transition_state(cstart, end_mode);
  auto result           = true;
  const auto space      = si_->getStateSpace();
  unsigned int nd       = space->validSegmentCount(start, end);
  auto* prev            = si_->allocState();
  si_->copyState(prev, start);
  if (nd > 1) {
    auto* test = si_->allocState()->as<CompositeStateSpace::StateType>();
    for (unsigned int j = 1; j < nd; ++j) {
      space->interpolate(start, end, static_cast<double>(j) / static_cast<double>(nd), test);
      if (!si_->isValid(test)) {
        last_valid.second = static_cast<double>(j - 1) / static_cast<double>(nd);
        if (last_valid.first != nullptr) {
          space->interpolate(start, end, last_valid.second, last_valid.first);
        }

        result = false;
        break;
      }

      si_->copyState(prev, test);
    }

    si_->freeState(test);
  }

  result &= valid_transition;

  if (result) {
    if (!si_->isValid(end)) {
      last_valid.second = static_cast<double>(nd - 1) / static_cast<double>(nd);
      if (last_valid.first != nullptr) {
        space->interpolate(start, end, last_valid.second, last_valid.first);
      }

      result = false;
    }
  }

  if (result) {
    valid_++;
  } else {
    invalid_++;
  }

  return result;
}
}  // namespace planner
