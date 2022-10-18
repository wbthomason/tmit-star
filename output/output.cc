#include "output.hh"

#include <ompl/geometric/PathGeometric.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <optional>
#include <unordered_map>

#include "cspace.hh"
#include "mode_atlas.hh"
#include "nlohmann/json.hpp"
#include "scenegraph.hh"
#include "state.hh"
#include "symbolic.hh"
#include "utils.hh"

namespace output {
struct Pose {
  std::array<double, 3> translation;
  std::array<double, 4> rotation;
};

struct LinkState {
  Pose pose;
  int parent;
};

struct PlanStep {
  Pose base_pose;
  std::unordered_map<std::string, double> joint_config;
  std::unordered_map<std::string, Pose> object_poses;
  std::optional<std::string> symbolic_action;
};

void to_json(nlohmann::json& j, const Pose& pose) {
  j["translation"] = pose.translation;
  j["rotation"]    = pose.rotation;
}

void to_json(nlohmann::json& j, const LinkState& link_state) {
  j["pose"]   = link_state.pose;
  j["parent"] = link_state.parent;
}

void to_json(nlohmann::json& j, const PlanStep& ps) {
  j["joints"]       = ps.joint_config;
  j["base_pose"]    = ps.base_pose;
  j["object_poses"] = ps.object_poses;
  if (ps.symbolic_action) {
    j["action"] = *ps.symbolic_action;
  }
}

void to_json(nlohmann::json& j, const PlannerSolution& p) {
  j["cost"]    = p.cost;
  j["time_ms"] = p.time_ms;
}

Pose pose_from_transform(const planner::geometric::Transform3<>& tf) {
  Pose result;
  // NOTE: We could avoid constructing here, but it doesn't matter much; this is just the output
  // step
  const Eigen::Translation3d translation(tf.translation());
  const Eigen::Quaterniond rotation(tf.linear());
  result.translation = {translation.x(), translation.y(), translation.z()};
  result.rotation    = {rotation.x(), rotation.y(), rotation.z(), rotation.w()};
  return result;
}

PlanStep state_to_plan_step(const planner::CompositeStateSpace::StateType* state,
                            planner::ModeAtlas* mode_atlas) {
  PlanStep result;
  const auto configuration = state->geometric_configuration();
  std::vector<planner::geometric::Transform3<>> poses;
  const auto& mode = state->get_mode();
  const auto& sg   = mode_atlas->get_mode_info(mode).sg;
  planner::geometric::pose_all(sg, configuration, poses);

  // Set the base pose
  result.base_pose.translation = {configuration.base_x, configuration.base_y, 0.0};
  Eigen::Quaterniond base_rotation_quat(
  Eigen::AngleAxisd(configuration.base_yaw, Eigen::Vector3d::UnitZ()));
  result.base_pose.rotation = {
  base_rotation_quat.x(), base_rotation_quat.y(), base_rotation_quat.z(), base_rotation_quat.w()};

  // Set the joint configuration and object poses
  size_t joint_config_idx = 0;
  for (size_t i = 0; i < sg.scene_info->joint_info.size(); ++i) {
    const auto& joint_info = sg.scene_info->joint_info[i];
    const auto& joint      = sg.joints[i];
    if (joint_info.type == planner::geometric::JointInfo::Type::RobotJoint &&
        joint.type != planner::geometric::Joint::Type::Fixed) {
      // This is a controllable joint
      result.joint_config[joint_info.name] = configuration.joint_poses[joint_config_idx];
      ++joint_config_idx;
    } else if (joint_info.type != planner::geometric::JointInfo::Type::RobotJoint) {
      // NOTE: We could be more efficient by only outputting the movable objects for each step.
      result.object_poses[joint_info.name] = pose_from_transform(poses[i]);
    }
  }

  // Set the action name, if any
  if (state->transition) {
    result.symbolic_action = state->transition->action->name;
  }

  return result;
}

void output_plan(const ompl::geometric::PathGeometric* const plan,
                 planner::ModeAtlas* mode_atlas,
                 const std::filesystem::path output_filepath) {
  const auto plan_length = plan->getStateCount();
  std::vector<PlanStep> plan_steps;
  plan_steps.reserve(plan_length);

  for (size_t i = 0; i < plan_length; ++i) {
    const auto* current_state = plan->getState(i)->as<planner::CompositeStateSpace::StateType>();
    auto& current_step = plan_steps.emplace_back(state_to_plan_step(current_state, mode_atlas));

    // Check that we only include symbolic actions if they're actually used
    if (current_step.symbolic_action) {
      if (i + 1 >= plan_length) {
        current_step.symbolic_action = std::nullopt;
      } else {
        const auto* next_state =
        plan->getState(i + 1)->as<planner::CompositeStateSpace::StateType>();
        if (current_state->get_mode() == next_state->get_mode()) {
          current_step.symbolic_action = std::nullopt;
        }
      }
    }
  }

  nlohmann::json output_json(plan_steps);
  std::ofstream output_file(output_filepath);
  if (!output_file.is_open()) {
    auto log = utils::get_logger("tmit-star::output");
    log->critical("Couldn't open file {} for plan output!", output_filepath.c_str());
    return;
  }

  output_file << std::setw(4) << output_json << std::endl;
  output_file.flush();
}

void action_aware_interpolate(ompl::geometric::PathGeometric* plan) {
  // Adapted from PathGeometric.cpp
  std::vector<ompl::base::State*> newStates;
  const auto& states       = plan->getStates();
  const int segments       = states.size() - 1;
  const auto& si           = plan->getSpaceInformation();
  ompl::base::State* start = si->allocState();
  for (int i = 0; i < segments; ++i) {
    ompl::base::State* s1 = states[i];
    ompl::base::State* s2 = states[i + 1];
    newStates.push_back(s1);
    const auto* cstate1 = static_cast<planner::CompositeStateSpace::StateType*>(s1);
    unsigned int n      = si->getStateSpace()->validSegmentCount(s1, s2);
    std::vector<ompl::base::State*> block;
    si->copyState(start, cstate1);
    auto* cstart = static_cast<planner::CompositeStateSpace::StateType*>(start);
    if (cstart->transition) {
      cstart->mode = cstart->transition_mode;
    }

    si->getMotionStates(start, s2, block, n - 1, false, true);
    newStates.insert(newStates.end(), block.begin(), block.end());
  }

  newStates.push_back(states[segments]);
  plan->getStates() = newStates;
}
}  // namespace output
