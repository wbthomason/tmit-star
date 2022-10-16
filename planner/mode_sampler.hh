#ifndef MODE_SAMPLER_HH
#define MODE_SAMPLER_HH
// #include <iostream>
#include <memory>
#include <vector>

#include "collisions.hh"
#include "cspace.hh"
#include "mode_atlas.hh"
#include "ompl/base/Cost.h"
#include "ompl/base/Goal.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/StateValidityChecker.h"
#include "sampler.hh"

constexpr unsigned int ABSOLUTE_SUCCESS_CAP = 5;
namespace planner {
struct ModeSampler {
  ModeSampler(const ompl::base::SpaceInformationPtr space,
              const ompl::base::ProblemDefinitionPtr problem,
              const std::shared_ptr<UninformedSampler> sampler,
              const ompl::base::Cost& solution_cost,
              const std::function<double(std::size_t)> compute_connection_radius,
              std::size_t& batch_id,
              ModeAtlas* mode_atlas,
              std::size_t& num_samples,
              std::size_t& num_valid_samples,
              std::size_t num_batches_per_plan  = 5,
              std::size_t initial_success_cap   = 0,
              std::size_t success_cap_increment = 1)
  : space(space)
  , problem(problem)
  , sampler(sampler)
  , state_validity_checker(
    static_cast<const planner::CollisionChecker*>(space->getStateValidityChecker().get()))
  , goal(problem->getGoal())
  , solution_cost(solution_cost)
  , compute_connection_radius(compute_connection_radius)
  , batch_id(batch_id)
  , mode_atlas(mode_atlas)
  , num_samples(num_samples)
  , num_valid_samples(num_valid_samples)
  , num_batches_per_plan(num_batches_per_plan)
  , success_cap(initial_success_cap)
  , success_cap_increment(success_cap_increment) {}

  void reset() {
    need_new_task_plan = false;
    batch_budget       = num_batches_per_plan;
  }

  void init_new_batch() {
    ++batch_id;
    tried_a_transition   = false;
    no_transition_budget = true;
    mode_idx             = 0;
    next_mode_idx        = 0;
    --batch_budget;
    success_cap += success_cap_increment;
    mode_frontier.clear();
    mode_frontier.reserve(mode_atlas->num_known_modes());
    mode_atlas->sampled_modes.reserve(mode_atlas->num_known_modes());
    mode_atlas->sampled_modes.assign(mode_atlas->num_known_modes(), false);
  }

  template <typename NodeT>
  bool add_samples(std::size_t num_samples_per_mode,
                   const ompl::base::PlannerTerminationCondition& termination_condition,
                   std::vector<std::shared_ptr<NodeT>>& new_samples,
                   std::vector<const ompl::base::State*>& new_goal_samples) {
    // TODO: Maybe add logic to decide if we've sampled enough in a given mode and can skip it?

    // Steps:
    // 1. For each mode in the reachable modes, sample a batch:
    //  i. sample up to the current batch number of samples, validity checking each sample
    //  ii. check each sample's distance to any transitions that still have successes budget left
    //  iii. sample transitions that are close enough (using connection radius function)
    //  iv. handle transition sampling success/failure
    //  v. if we took a transition, sample in the resulting mode next. Otherwise, sample in the
    //  next yet-unsampled mode. But what about succeeding at multiple transitions? Keep a
    //  frontier
    // 2. Set up for next time:
    //   i. if we didn't succeed at any new transitions this time through, request a new task plan
    //   if we've taken the requisite number of batches already and raise the success budget cap

    if (need_new_task_plan) {
      mode_atlas->update_task_plan();
      reset();
      init_new_batch();
    } else if (mode_idx >= mode_atlas->num_known_modes()) {
      init_new_batch();
    }

    // Have we sampled in the goal mode this batch entry? (NOTE: Not the same as "this batch", deals
    // with early return and resumption)
    bool reached_goal_mode = false;

    // Add samples to modes until we reach the goal, we run out of modes, or the termination
    // condition triggers
    while (!reached_goal_mode && next_mode_idx <= mode_atlas->reachable_modes.size() &&
           !termination_condition) {
      const auto mode = mode_atlas->reachable_modes[mode_idx];
      // Mark this mode as sampled in, for this batch
      mode_atlas->sampled_modes[mode_idx] = true;
      auto& symbolic_state_info           = mode_atlas->get_symbolic_state_info(mode.symbols);
      auto& mode_info                     = mode_atlas->get_mode_info(mode);
      auto& transitions                   = symbolic_state_info.transitions;
      const auto mode_connection_radius   = compute_connection_radius(mode_info.num_samples);
      int num_needed_samples              = num_samples_per_mode;
      new_samples.reserve(new_samples.size() + num_needed_samples);

      // Add samples to this mode until we have a full batch or the termination condition triggers
      while (num_needed_samples > 0 && !termination_condition) {
        new_samples.emplace_back(std::make_shared<NodeT>(space, problem, batch_id));
        auto* sample_state = new_samples.back()->getState();
        sample_state->template as<CompositeStateSpace::StateType>()->set_mode(mode);
        if (goal->isSatisfied(sample_state)) {
          new_samples.pop_back();
          break;
        }

        do {
          sampler->sampleUniform(sample_state, solution_cost);
          ++num_samples;
        } while (!state_validity_checker->isValid(sample_state));

        ++num_valid_samples;
        --num_needed_samples;
        ++mode_info.num_samples;

        // Check if our uniform sample is close enough to any transitions out of the mode to attempt
        // them
        for (auto& transition : transitions) {
          if (transition->successes < ABSOLUTE_SUCCESS_CAP && transition->successes < success_cap) {
            no_transition_budget = false;
            if (sampler->transition_distance(sample_state,
                                             state_validity_checker->get_last_poses(),
                                             transition.get()) < mode_connection_radius) {
              tried_a_transition     = true;
              auto transition_node   = std::make_shared<NodeT>(space, problem, batch_id);
              auto* transition_state = transition_node->getState();
              if (sampler->sample_transition_near(
                  transition_state, sample_state, transition.get()) &&
                  state_validity_checker->isValid(transition_state)) {
                const auto transition_mode = mode_atlas->transition_succeeded(
                transition.get(), transition_state, state_validity_checker->get_last_poses());
                auto bridge_node = std::make_shared<NodeT>(space, problem, batch_id);
                auto* bridge_state =
                bridge_node->getState()->template as<CompositeStateSpace::StateType>();
                space->copyState(bridge_state, transition_state);
                bridge_state->transition = std::nullopt;
                bridge_state->set_mode(transition_mode);

                // TODO: This is a hack that's only valid as long as our goals are purely symbolic,
                // and there's probably a more efficient way to do this
                if (goal->isSatisfied(bridge_state)) {
                  reached_goal_mode     = true;
                  auto* goal_mode_state = space->allocState();
                  space->copyState(goal_mode_state, bridge_state);
                  new_goal_samples.emplace_back(goal_mode_state);
                } else {
                  new_samples.emplace_back(std::move(bridge_node));
                }

                transition_state->template as<CompositeStateSpace::StateType>()->transition_mode =
                transition_mode;
                // transition_states.push_back(
                // transition_state->as<planner::CompositeStateSpace::StateType>());
                new_samples.emplace_back(std::move(transition_node));

                ++num_samples;
                ++num_valid_samples;
                --num_needed_samples;
                ++mode_info.num_samples;

                mode_frontier.push_back(
                mode_atlas->get_mode_info(transition_mode).reachable_mode_idx);
              } else {
                mode_atlas->transition_failed(transition.get());
              }
            }
          }
        }
      }

      // Update the mode index: follow the current transition chain if we can, otherwise scan
      // forward for the next yet-unvisited mode in this batch
      if (mode_frontier.empty()) {
        while (next_mode_idx < mode_atlas->reachable_modes.size() &&
               mode_atlas->sampled_modes[next_mode_idx]) {
          ++next_mode_idx;
        }

        mode_idx = next_mode_idx;
        ++next_mode_idx;
      } else {
        mode_idx = mode_frontier.back();
        mode_frontier.pop_back();
      }

    }

    // Determine if we will be continuing this batch, starting a new batch with the same task plan
    // set, or requesting a new task plan
    // There are three cases:
    // 1. We succeeded at a transition into the goal mode, and we're exiting (potentially) early
    if (reached_goal_mode) {
      return true;
    }

    // 2. We didn't succeed at a transition into the goal mode, and:
    // 2(a). We tried other transitions (may or may not have succeeded). We need to start another
    // batch if there's batch budget left
    if (no_transition_budget || (tried_a_transition && batch_budget <= 0)) {
      need_new_task_plan = true;
      // NOTE: We return this value because, if we've made it to the goal at least once, then maybe
      // this batch improved local connectivity even if it didn't reach the goal anew
      return have_goal_mode_samples;
    }

    // 2(b). We didn't get a chance to try any transitions. We try another batch, for free?
    // NOTE: We return this value because, if we've made it to the goal at least once, then maybe
    // this batch improved local connectivity even if it didn't reach the goal anew
    return have_goal_mode_samples;
  }

 private:
  const ompl::base::SpaceInformationPtr space;
  const ompl::base::ProblemDefinitionPtr problem;
  const std::shared_ptr<UninformedSampler> sampler;
  const CollisionChecker* const state_validity_checker;
  const ompl::base::GoalPtr goal;
  const ompl::base::Cost& solution_cost;
  const std::function<double(std::size_t)> compute_connection_radius;
  std::size_t& batch_id;
  ModeAtlas* const mode_atlas;
  std::size_t& num_samples;
  std::size_t& num_valid_samples;

  const std::size_t num_batches_per_plan;
  std::size_t success_cap;
  const std::size_t success_cap_increment;

  bool need_new_task_plan{true};
  bool have_goal_mode_samples{false};
  bool tried_a_transition{false};
  bool no_transition_budget{true};
  std::size_t mode_idx{0};
  std::size_t next_mode_idx{1};
  std::size_t batch_budget;
  std::vector<std::size_t> mode_frontier;
};
}  // namespace planner

#endif
