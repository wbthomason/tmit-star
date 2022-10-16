#include "mode_atlas.hh"

#include <ompl/util/RandomNumbers.h>

#include <algorithm>
#include <boost/dynamic_bitset.hpp>
#include <limits>
#include <memory>
#include <optional>
#include <utility>

#include "cspace.hh"
#include "mode.hh"
#include "scenegraph.hh"
#include "task_plan.hh"

namespace planner {
ModeAtlas::ModeAtlas(const symbolic::Domain& domain,
                     const symbolic::Problem& problem,
                     geometric::SceneGraph&& initial_sg)
: reachable_modes{}
, start_symbolic_state{problem.initial_state}
, planner_state(domain, problem)
, symbolic_state_info{}
, mode_info{}
, last_plan_actions{}
, last_plan_transitions{}
, first_movable_object_idx{initial_sg.scene_info->num_robot_joints +
                           initial_sg.scene_info->num_static_objects} {
  add_symbolic_state(start_symbolic_state);
  const auto start_mode =
  add_mode(start_symbolic_state, std::forward<geometric::SceneGraph>(initial_sg));
  auto& start_mode_info              = get_mode_info(start_mode);
  start_mode_info.reached            = true;
  start_mode_info.reachable_mode_idx = 0;
  reachable_modes.emplace_back(start_mode);
  mode_dims = start_symbolic_state.size();
}

void ModeAtlas::add_symbolic_state(const Mode::SymbolicStateT& symbolic_state) {
  symbolic_state_info.emplace(symbolic_state, SymbolicStateInfo(symbolic_state));
}

Mode ModeAtlas::add_mode(const Mode::SymbolicStateT& symbolic_state, geometric::SceneGraph&& sg) {
  auto& symbolic_state_info = get_symbolic_state_info(symbolic_state);
  Mode new_mode{symbolic_state, symbolic_state_info.num_modes};
  ++symbolic_state_info.num_modes;
  mode_info.emplace(new_mode, ModeInfo(new_mode, std::forward<geometric::SceneGraph>(sg)));
  return new_mode;
}

ModeInfo& ModeAtlas::get_mode_info(const Mode& mode) { return mode_info.at(mode); }

SymbolicStateInfo& ModeAtlas::get_symbolic_state_info(const Mode::SymbolicStateT& mode) {
  return symbolic_state_info.at(mode);
}

// NOTE: The way we're selecting where to sample is naive. We're always sampling in
// every reachable mode, which can get expensive as we explore more. But we can't just start
// at the start mode and only add modes to sample as we succeed at a transition through to them; we
// wouldn't necessarily revisit modes we already know how to reach.
// IDEA: Maybe we should track where the reverse/forward search failed, and use that to guide where
// we sample more?

bool ModeAtlas::update_task_plan() {
  // Block the last plan prefix
  if (!last_plan_transitions.empty()) {
    std::size_t first_blocked_transition_idx = 0;
    while (first_blocked_transition_idx < last_plan_transitions.size()) {
      if (last_plan_transitions[first_blocked_transition_idx]->successes == 0) {
        break;
      }

      ++first_blocked_transition_idx;
    }

    forbid_plan_prefix(planner_state,
                       last_plan_actions,
                       std::min(last_plan_transitions.size(), first_blocked_transition_idx + 1));
  }

  last_plan_transitions.clear();
  last_plan_actions.clear();

  // Invoke the task planner
  const auto new_plan_opt = plan(planner_state);

  // If we succeeded, add the sequence of transitions and corresponding modes
  if (!new_plan_opt) {
    return false;
  }

  const auto& new_plan = new_plan_opt.value();
  Mode::SymbolicStateT from_symbolic_state(start_symbolic_state);
  for (unsigned int step = 0; step < new_plan.size(); ++step) {
    const auto* action           = new_plan[step];
    const auto to_symbolic_state = action->effect.apply(from_symbolic_state);
    // If the mode is new, set it up
    if (!symbolic_state_info.contains(to_symbolic_state)) {
      add_symbolic_state(to_symbolic_state);
    }

    // If the transition is new, set it up
    auto& from_symbolic_state_info = get_symbolic_state_info(from_symbolic_state);
    Transition* transition         = nullptr;

    for (auto& known_transition : from_symbolic_state_info.transitions) {
      if (known_transition->to_symbolic_state == to_symbolic_state &&
          known_transition->action == action) {
        transition = known_transition.get();
        break;
      }
    }

    if (transition == nullptr) {
      auto& new_transition = from_symbolic_state_info.transitions.emplace_back(
      std::make_unique<Transition>(from_symbolic_state, to_symbolic_state, action));
      transition = new_transition.get();
    }

    transition->steps.push_back(step);
    from_symbolic_state = to_symbolic_state;
    last_plan_transitions.push_back(transition);
    last_plan_actions.push_back(action);
  }

  return true;
}


Mode ModeAtlas::transition_succeeded(Transition* const transition,
                                     const ompl::base::State* const state,
                                     const std::vector<geometric::Transform3<>>& poses) {
  // Add another success to the transition
  ++transition->successes;

  // Record this state as associated with the transition
  transition->states.push_back(state);

  // Generate the scenegraph resulting from the transition and check if it is new
  // TODO: Could check if the action has any kinematic effects first to save some effort in a
  // minority of cases
  const auto* cstate    = state->as<CompositeStateSpace::StateType>();
  const auto& from_mode = cstate->get_mode();
  auto updated_sg =
  update_scenegraph_for_action(get_mode_info(from_mode).sg, transition->action, poses);

  const auto& to_symbolic_state = transition->to_symbolic_state;
  auto& to_symbolic_state_info  = get_symbolic_state_info(to_symbolic_state);
  auto equivalent_sg_idx        = -1;
  const auto& updated_sg_joints = updated_sg.joints;
  for (std::size_t i = 0; i < to_symbolic_state_info.num_modes && equivalent_sg_idx < 0; ++i) {
    // Check the scenegraph associated with this mode
    const auto& mode_sg   = get_mode_info({to_symbolic_state, i}).sg;
    const auto& sg_joints = mode_sg.joints;
    // Two scenegraphs are different if any of the movable objects differ between the two in (1)
    // their parent link or (2) their pose relative to their common parent link
    bool sg_differs = false;
    for (size_t j = first_movable_object_idx; j < updated_sg_joints.size(); ++j) {
      if (sg_joints[j].parent != updated_sg_joints[j].parent ||
          !updated_sg_joints[j].tf.isApprox(sg_joints[j].tf, 1e-3)) {
        sg_differs = true;
        break;
      }
    }

    if (!sg_differs) {
      equivalent_sg_idx = i;
    }
  }

  // Update the reached mode
  Mode to_mode;
  if (equivalent_sg_idx < 0) {
    // This is a new mode!
    to_mode = add_mode(to_symbolic_state, std::forward<geometric::SceneGraph>(updated_sg));
  } else {
    to_mode = {to_symbolic_state, static_cast<Mode::PoseSetT>(equivalent_sg_idx)};
  }

  auto& to_mode_info = get_mode_info(to_mode);

  if (to_mode_info.reached == false) {
    // It's the first time we've reached this mode
    reachable_modes.emplace_back(to_mode);
    to_mode_info.reachable_mode_idx = reachable_modes.size() - 1;
    sampled_modes.push_back(false);
  }

  // Mark mode as reachable
  to_mode_info.reached = true;

  // Add the entry point
  to_mode_info.entry_states.push_back(state);

  return to_mode;
}

void ModeAtlas::transition_failed(Transition* transition) { ++transition->failures; }

geometric::SceneGraph
ModeAtlas::update_scenegraph_for_action(const geometric::SceneGraph& sg,
                                        const symbolic::Action* const action,
                                        const std::vector<geometric::Transform3<>>& poses) {
  geometric::SceneGraph result(sg);
  // Check every predicate in the action's effect for kinematic effects, and play them out on the
  // scenegraph
  // NOTE: This assumes that no two predicates conflict in their kinematic effects in the same
  // action effect

  // Unlink for deleted predicates
  for (const auto& predicate : action->effect.del) {
    for (const auto& [parent, child] : predicate->kinematic_links) {
      const auto child_idx = result.scene_info->index.at(predicate->arguments[child]);
      auto& child_joint    = result.joints[child_idx];
      child_joint.parent   = -1;
      child_joint.tf       = poses[child_idx];
    }
  }

  // Link for added predicates
  for (const auto& predicate : action->effect.add) {
    for (const auto& [parent, child] : predicate->kinematic_links) {
      const auto child_idx  = result.scene_info->index.at(predicate->arguments[child]);
      const auto parent_idx = result.scene_info->index.at(predicate->arguments[parent]);

      // Link the two joints
      auto& child_joint  = result.joints[child_idx];
      child_joint.parent = parent_idx;
      // child_joint.tf     = parent_pose.inverse() * child_pose;
      // NOTE: This is only really valid if we only ever link objects to manipulators. If, e.g., we
      // wanted to re-parent objects to a tray or similar, this would not be correct. The above line
      // should be mathematically correct, but likely caused problems because the relative poses
      // between manipulators and picked objects was large
      child_joint.tf = geometric::Transform3<>::Identity();
    }
  }

  return result;
}

size_t ModeAtlas::num_known_modes() const { return mode_info.size(); }
}  // namespace planner
