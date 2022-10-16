#ifndef SCENEGRAPH_HH
#define SCENEGRAPH_HH
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>
#include <string>
#include <vector>

#include "autodiff/forward/real.hpp"
#include "autodiff/forward/real/eigen.hpp"
#include "bullet/btBulletCollisionCommon.h"
#include "robin_hood.h"
#include "state.hh"

namespace planner::geometric {
struct Joint {
  enum struct Type { Revolute, Prismatic, Fixed, Continuous };
  const Joint::Type type;
  int parent;
  Transform3<> tf;
  // HACK: Uninitialized if not an AxialJoint
  Eigen::Vector3d axis;

 protected:
  Joint(Joint::Type type, const Transform3<>& tf, int parent)
  : type(type), parent(parent), tf(tf) {}
};

struct FixedJoint : public Joint {
  FixedJoint(const Transform3<>& tf, int parent) : Joint(Joint::Type::Fixed, tf, parent) {}
};

struct AxialJoint : public Joint {
 protected:
  AxialJoint(Joint::Type type, const Eigen::Vector3d& axis, const Transform3<>& tf, int parent)
  : Joint(type, tf, parent) {
    this->axis = axis;
  }
};

struct RevoluteJoint : public AxialJoint {
  RevoluteJoint(const Eigen::Vector3d& axis, const Transform3<>& tf, int parent)
  : AxialJoint(Joint::Type::Revolute, axis, tf, parent) {}
};

struct PrismaticJoint : public AxialJoint {
  PrismaticJoint(const Eigen::Vector3d& axis, const Transform3<>& tf, int parent)
  : AxialJoint(Joint::Type::Prismatic, axis, tf, parent) {}
};

struct ContinuousJoint : public AxialJoint {
  ContinuousJoint(const Eigen::Vector3d& axis, const Transform3<>& tf, int parent)
  : AxialJoint(Joint::Type::Continuous, axis, tf, parent) {}
};

struct JointBounds {
  static auto infinity() -> JointBounds {
    return {-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
  }

  double lower = 0.0;
  double upper = 0.0;
};

struct AreaBounds {
  double x_low  = 0.0;
  double x_high = 0.0;
  double y_low  = 0.0;
  double y_high = 0.0;
  double z_low  = 0.0;
  double z_high = 0.0;
};

struct JointInfo {
  enum struct Type { RobotJoint, StaticObject, MovableObject };
  JointInfo(const std::string& name,
            Type type,
            const std::optional<size_t>& collision_idx,
            size_t joint_idx,
            size_t configuration_idx,
            const std::optional<JointBounds>& bounds,
            const std::optional<AreaBounds>& surface_bounds)
  : name(name)
  , type(type)
  , collision_idx(collision_idx)
  , joint_idx(joint_idx)
  , configuration_idx(configuration_idx)
  , bounds(bounds)
  , surface_bounds(surface_bounds) {}
  std::string name;
  Type type;
  std::optional<size_t> collision_idx;
  size_t joint_idx;
  size_t configuration_idx;
  std::optional<JointBounds> bounds;
  std::optional<AreaBounds> surface_bounds;
};

struct CollisionInfo {
  size_t joint_idx = -1;
  Transform3<> collision_transform;
};

struct SceneInfo {
  const size_t num_robot_joints;
  const size_t num_controllable_joints;
  const size_t num_movable_objects;
  const size_t num_static_objects;
  const size_t num_surfaces;
  const std::vector<JointInfo> joint_info;
  const std::vector<std::shared_ptr<btCollisionShape>> geometries;
  const std::vector<CollisionInfo> collision_info;
  const robin_hood::unordered_map<std::string, size_t> index;
};

struct SceneGraph {
  SceneGraph(std::vector<Joint>&& joints, std::shared_ptr<const SceneInfo> scene_info)
  : joints(std::move(joints)), scene_info(scene_info) {}

  SceneGraph(const SceneGraph& other) = default;

  bool has_robot_parent(size_t object_idx) const;

  std::vector<Joint> joints;
  const std::shared_ptr<const SceneInfo> scene_info;
};

std::vector<Transform3<>>
pose_all_collisions(const SceneGraph& sg, const std::vector<Transform3<>>& link_poses);

template <typename JointT, typename NumT>
Transform3<NumT> pose(const JointT& joint,
                      const Transform3<NumT>& parent_tf,
                      const State<NumT>& q,
                      size_t& joint_state_index);

template <typename NumT>
Transform3<NumT> pose(const AxialJoint& joint,
                      const Transform3<NumT>& parent_tf,
                      const State<NumT>& q,
                      size_t& joint_state_index) {
  Eigen::Quaternion<NumT> posed_axis(
  Eigen::AngleAxis<NumT>(q.joint_poses[joint_state_index], joint.axis.template cast<NumT>()));
  ++joint_state_index;
  return parent_tf * joint.tf.template cast<NumT>() * posed_axis;
}

template <typename NumT>
Transform3<NumT> pose(const PrismaticJoint& joint,
                      const Transform3<NumT>& parent_tf,
                      const State<NumT>& q,
                      size_t& joint_state_index) {
  const auto posed_displacement =
  Eigen::Translation<NumT, 3>(q.joint_poses[joint_state_index] * joint.axis.template cast<NumT>());
  ++joint_state_index;
  return parent_tf * joint.tf.template cast<NumT>() * posed_displacement;
}

template <typename NumT>
void pose_all(const SceneGraph& sg, const State<NumT>& q, std::vector<Transform3<NumT>>& result) {
  result.clear();
  result.reserve(sg.joints.size());
  size_t joint_state_index = 0;
  // First, the robot joints
  const Transform3<NumT> base_pose = make_base_pose(q);
  result.emplace_back(base_pose.template cast<NumT>() * sg.joints[0].tf.template cast<NumT>());
  const auto num_robot_joints   = sg.scene_info->num_robot_joints;
  const auto num_static_objects = sg.scene_info->num_static_objects;
  for (size_t i = 1; i < num_robot_joints; ++i) {
    const auto& joint = sg.joints[i];
    switch (joint.type) {
      case Joint::Type::Fixed:
        result.emplace_back(result[joint.parent] * joint.tf.template cast<NumT>());
        break;
      case Joint::Type::Revolute:
      case Joint::Type::Continuous:
        result.emplace_back(
        pose(static_cast<const AxialJoint&>(joint), result[joint.parent], q, joint_state_index));
        break;
      case Joint::Type::Prismatic:
        result.emplace_back(pose(
        static_cast<const PrismaticJoint&>(joint), result[joint.parent], q, joint_state_index));
        break;
    }
  }

  // Then the static objects
  for (size_t i = num_robot_joints; i < num_robot_joints + num_static_objects; ++i) {
    result.emplace_back(sg.joints[i].tf);
  }

  // Could make this more efficient by not returning the poses of the static
  // objects and instead checking the range of the parent index in the below conditional to
  // determine if we load a parent transform from result (in the case of a robot or movable object
  // joint) or from sg.joints (in the case of a static object)

  // Finally, the movable objects
  for (size_t i = num_robot_joints + num_static_objects; i < sg.joints.size(); ++i) {
    const auto& object = sg.joints[i];
    if (object.parent >= 0) {
      result.emplace_back(result[object.parent] * object.tf.template cast<NumT>());
    } else {
      result.emplace_back(object.tf);
    }
  }
}

template <typename NumT>
Transform3<NumT>
pose_single_joint(const SceneGraph& sg, const State<NumT>& q, unsigned int joint_idx) {
  std::vector<unsigned int> queue;
  queue.reserve(joint_idx);
  const auto* current_joint = &sg.joints[joint_idx];
  do {
    queue.push_back(joint_idx);
    joint_idx     = current_joint->parent;
    current_joint = &sg.joints[joint_idx];
  } while (current_joint->parent >= 0);
  Transform3<NumT> result;
  const auto& joint_info = sg.scene_info->joint_info;
  if (joint_info[joint_idx].type == JointInfo::Type::RobotJoint) {
    result = make_base_pose(q);
  } else {
    result = Transform3<NumT>::Identity();
  }

  for (auto idx_it = queue.rbegin(); idx_it != queue.rend(); ++idx_it) {
    const auto idx               = *idx_it;
    const auto& joint            = sg.joints[idx];
    const auto configuration_idx = joint_info[idx].configuration_idx;
    switch (joint.type) {
      case Joint::Type::Fixed:
        result *= joint.tf;
        break;
      case Joint::Type::Revolute:
      case Joint::Type::Continuous:
        result *= pose(static_cast<const AxialJoint&>(joint), q, configuration_idx);
        break;
      case Joint::Type::Prismatic:
        result *= pose(static_cast<const PrismaticJoint&>(joint), q, configuration_idx);
        break;
    }
  }

  return result;
}

void pose_all_collisions(const SceneGraph& sg,
                         const std::vector<Transform3<>>& link_poses,
                         std::vector<Transform3<>>& result);

using PoseSetPartialDerivative = std::vector<planner::geometric::Transform3<autodiff::real>>;
void pose_partial_derivatives(const SceneGraph& sg,
                              State<autodiff::real>& state,
                              std::vector<PoseSetPartialDerivative>& partials);
std::vector<PoseSetPartialDerivative>
pose_partial_derivatives(const SceneGraph& sg, State<autodiff::real>& state);
}  // namespace planner::geometric
#endif
