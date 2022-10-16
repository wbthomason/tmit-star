#include "scenegraph.hh"

namespace planner::geometric {
bool SceneGraph::has_robot_parent(size_t object_idx) const {
  const auto* joint      = &joints[scene_info->num_robot_joints + scene_info->num_static_objects + object_idx];
  const auto& joint_info = scene_info->joint_info;
  while (joint->parent >= 0) {
    if (joint_info[joint->parent].type == JointInfo::Type::RobotJoint) {
      return true;
    }

    joint = &joints[joint->parent];
  }

  return false;
}

void pose_all_collisions(const SceneGraph& sg, const std::vector<Transform3<>>& link_poses, std::vector<Transform3<>>& result) {
  const auto& collision_info = sg.scene_info->collision_info;
  result.clear();
  result.reserve(collision_info.size());
  for (const auto& [joint_idx, collision_tf] : collision_info) {
    result.emplace_back(link_poses[joint_idx] * collision_tf);
  }
}

void pose_partial_derivatives(const SceneGraph& sg, State<autodiff::real>& state, std::vector<PoseSetPartialDerivative>& partials) {
  partials.reserve(3 + state.joint_poses.size());

  // First the base state
  state.base_x[1] = 1.0;
  pose_all(sg, state, partials.emplace_back());
  state.base_x[1] = 0.0;

  state.base_y[1] = 1.0;
  pose_all(sg, state, partials.emplace_back());
  state.base_y[1] = 0.0;

  state.base_yaw[1] = 1.0;
  pose_all(sg, state, partials.emplace_back());
  state.base_yaw[1] = 0.0;

  // Then each joint
  for (auto& joint_pose : state.joint_poses) {
    joint_pose[1] = 1.0;
    pose_all(sg, state, partials.emplace_back());
    joint_pose[1] = 0.0;
  }
}

std::vector<PoseSetPartialDerivative> pose_partial_derivatives(const SceneGraph& sg, State<autodiff::real>& state) {
  std::vector<PoseSetPartialDerivative> result;
  pose_partial_derivatives(sg, state, result);
  return result;
}
}  // namespace planner::geometric
