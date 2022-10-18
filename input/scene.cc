#include "scene.hh"

#include <urdf_model/joint.h>
#include <urdf_model/link.h>
#include <urdf_model/model.h>
#include <urdf_model/types.h>
#include <urdf_parser/urdf_parser.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <boost/container_hash/hash_fwd.hpp>
#include <cassert>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "bounds.hh"
#include "btBulletCollisionCommon.h"
#include "nlohmann/json.hpp"
#include "robin_hood.h"
#include "scenegraph.hh"
#include "state.hh"
#include "stlio.hpp"
#include "utils.hh"
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

struct AnnotatedJoint {
  std::string name;
  std::optional<std::string> parent_name;
  std::optional<size_t> geometry_idx;
  std::optional<size_t> configuration_idx;
  planner::geometric::Joint joint;
  std::optional<planner::geometric::JointBounds> bounds;
  std::optional<planner::geometric::AreaBounds> surface_bounds;
};

void import_joints(const std::vector<AnnotatedJoint>& annotated_joints,
                   planner::geometric::JointInfo::Type joint_type,
                   std::vector<planner::geometric::Joint>& joints,
                   std::vector<planner::geometric::JointInfo>& joint_info,
                   robin_hood::unordered_map<std::string, size_t>& index) {
  for (size_t i = 0; i < annotated_joints.size(); ++i) {
    const auto& annotated_joint = annotated_joints[i];
    joints.push_back(annotated_joint.joint);
    index.emplace(annotated_joint.name, joints.size() - 1);
    joint_info.emplace_back(planner::geometric::JointInfo{
    annotated_joint.name,
    joint_type,
    annotated_joint.geometry_idx,
    joints.size() - 1,
    annotated_joint.configuration_idx ? *annotated_joint.configuration_idx : -1,
    annotated_joint.bounds,
    annotated_joint.surface_bounds});
    if (annotated_joint.parent_name) {
      joints.back().parent = joint_info[index.at(*annotated_joint.parent_name)].joint_idx;
    } else {
      joints.back().parent = -1;
    }
  }
}

struct ScenegraphBuilder {
  void add_robot_joint(const std::string& name,
                       const std::optional<std::string>& parent_name,
                       const planner::geometric::Joint& joint,
                       const planner::geometric::Transform3<>& collision_transform,
                       const std::shared_ptr<btCollisionShape> collision_geometry,
                       const std::optional<planner::geometric::JointBounds> bounds) {
    std::optional<size_t> configuration_idx = std::nullopt;
    if (joint.type != planner::geometric::Joint::Type::Fixed) {
      configuration_idx = num_controllable_joints;
      ++num_controllable_joints;
    }

    if (collision_geometry != nullptr) {
      collision_info.emplace_back(
      planner::geometric::CollisionInfo{robot_joints.size(), collision_transform});
      robot_joints.emplace_back(AnnotatedJoint{
      name, parent_name, geometries.size(), configuration_idx, joint, bounds, std::nullopt});
      geometries.push_back(collision_geometry);
    } else {
      robot_joints.emplace_back(AnnotatedJoint{
      name, parent_name, std::nullopt, configuration_idx, joint, bounds, std::nullopt});
    }
  }

  void add_static_object(const std::string& name,
                         const std::optional<std::string>& parent_name,
                         const planner::geometric::FixedJoint& joint,
                         const std::shared_ptr<btCollisionShape> geometry,
                         const std::optional<planner::geometric::AreaBounds>& surface_bounds) {
    if (geometry != nullptr) {
      collision_info.emplace_back(planner::geometric::CollisionInfo{
      std::numeric_limits<size_t>::max(), planner::geometric::Transform3<>::Identity()});
      static_objects.emplace_back(AnnotatedJoint{
      name, parent_name, geometries.size(), std::nullopt, joint, std::nullopt, surface_bounds});
      geometries.push_back(geometry);
    } else {
      static_objects.emplace_back(AnnotatedJoint{
      name, parent_name, std::nullopt, std::nullopt, joint, std::nullopt, surface_bounds});
    }

    if (surface_bounds) {
      ++num_surfaces;
    }
  }

  void add_movable_object(const std::string& name,
                          const std::optional<std::string>& parent_name,
                          const planner::geometric::FixedJoint& joint,
                          const std::shared_ptr<btCollisionShape> geometry,
                          const std::optional<planner::geometric::AreaBounds>& surface_bounds) {
    if (geometry != nullptr) {
      collision_info.emplace_back(planner::geometric::CollisionInfo{
      std::numeric_limits<size_t>::max(), planner::geometric::Transform3<>::Identity()});
      movable_objects.emplace_back(AnnotatedJoint{
      name, parent_name, geometries.size(), std::nullopt, joint, std::nullopt, surface_bounds});
      geometries.push_back(geometry);
    } else {
      movable_objects.emplace_back(AnnotatedJoint{
      name, parent_name, std::nullopt, std::nullopt, joint, std::nullopt, surface_bounds});
    }

    if (surface_bounds) {
      ++num_surfaces;
    }
  }

  planner::geometric::SceneGraph build() {
    // Generates a scenegraph with a single vector for all joints from the three sets of object
    // types
    std::vector<planner::geometric::Joint> joints;
    std::vector<planner::geometric::JointInfo> joint_info;
    robin_hood::unordered_map<std::string, size_t> index;
    joints.reserve(robot_joints.size() + static_objects.size() + movable_objects.size());
    import_joints(
    robot_joints, planner::geometric::JointInfo::Type::RobotJoint, joints, joint_info, index);
    import_joints(
    static_objects, planner::geometric::JointInfo::Type::StaticObject, joints, joint_info, index);
    import_joints(
    movable_objects, planner::geometric::JointInfo::Type::MovableObject, joints, joint_info, index);
    // Now update collision indices
    for (const auto& joint : joint_info) {
      if (joint.collision_idx) {
        collision_info[*joint.collision_idx].joint_idx = joint.joint_idx;
      }
    }

    return planner::geometric::SceneGraph(std::move(joints),
                                          std::make_shared<planner::geometric::SceneInfo>(
                                          planner::geometric::SceneInfo{robot_joints.size(),
                                                                        num_controllable_joints,
                                                                        movable_objects.size(),
                                                                        static_objects.size(),
                                                                        num_surfaces,
                                                                        std::move(joint_info),
                                                                        std::move(geometries),
                                                                        std::move(collision_info),
                                                                        std::move(index)}));
  }

 private:
  std::vector<AnnotatedJoint> robot_joints;
  std::vector<AnnotatedJoint> movable_objects;
  std::vector<AnnotatedJoint> static_objects;
  std::vector<std::shared_ptr<btCollisionShape>> geometries;
  std::vector<planner::geometric::CollisionInfo> collision_info;
  unsigned int num_controllable_joints = 0;
  unsigned int num_surfaces            = 0;
};

constexpr btScalar COLLISION_MARGIN = 0.001;

struct ShapeInfo {
  std::string name;
  float x_scale = 1.0;
  float y_scale = 1.0;
  float z_scale = 1.0;
  bool operator==(const ShapeInfo& o) const {
    return x_scale == o.x_scale && y_scale == o.y_scale && z_scale == o.z_scale && name == o.name;
  }
};

template <> struct std::hash<ShapeInfo> {
  std::size_t operator()(const ShapeInfo& s) const noexcept {
    std::size_t name_hash    = std::hash<std::string>{}(s.name);
    std::size_t x_scale_hash = std::hash<float>{}(s.x_scale);
    std::size_t y_scale_hash = std::hash<float>{}(s.y_scale);
    std::size_t z_scale_hash = std::hash<float>{}(s.z_scale);
    std::size_t seed         = 0;
    boost::hash_combine(seed, name_hash);
    boost::hash_combine(seed, x_scale_hash);
    boost::hash_combine(seed, y_scale_hash);
    boost::hash_combine(seed, z_scale_hash);

    return seed;
  }
};

using ShapeCache = robin_hood::unordered_map<ShapeInfo, std::shared_ptr<btCollisionShape>>;

std::shared_ptr<btCollisionShape> load_obj(const std::filesystem::path& obj_path,
                                           float x_scale   = 1.0,
                                           float y_scale   = 1.0,
                                           float z_scale   = 1.0,
                                           bool is_concave = false) {
  auto log = utils::get_logger("tmit-star::input::scene");
  tinyobj::ObjReaderConfig obj_reader_config;
  obj_reader_config.mtl_search_path = obj_path.parent_path();
  obj_reader_config.triangulate     = false;
  tinyobj::ObjReader obj_reader;
  if (!obj_reader.ParseFromFile(obj_path, obj_reader_config)) {
    if (!obj_reader.Error().empty()) {
      log->error("Error reading .obj file at {}: {}", obj_path.c_str(), obj_reader.Error());
      return nullptr;
    }

    if (!obj_reader.Warning().empty()) {
      log->warn("Warning while reading .obj file at {}: {}",
                obj_path.c_str(),
                obj_reader.Warning());
    }
  }

  const auto& attributes = obj_reader.GetAttrib();

  std::vector<btVector3> vertices;
  vertices.reserve(attributes.vertices.size());
  btTransform identity;
  identity.setIdentity();
  const bool multiple_shapes = obj_reader.GetShapes().size() > 1;
  std::shared_ptr<btCollisionShape> result(nullptr);
  if (multiple_shapes) {
    result = std::make_shared<btCompoundShape>();
    result->setMargin(COLLISION_MARGIN);
  }

  for (const auto& shape : obj_reader.GetShapes()) {
    size_t index_offset = 0;
    for (const size_t num_face_vertices : shape.mesh.num_face_vertices) {
      for (size_t v = 0; v < num_face_vertices; ++v) {
        assert(num_face_vertices == 3);
        const auto v_idx = shape.mesh.indices[index_offset + v];
        const auto v_x =
        attributes.vertices[3 * static_cast<size_t>(v_idx.vertex_index) + 0] * x_scale;
        const auto v_y =
        attributes.vertices[3 * static_cast<size_t>(v_idx.vertex_index) + 1] * y_scale;
        const auto v_z =
        attributes.vertices[3 * static_cast<size_t>(v_idx.vertex_index) + 2] * z_scale;
        vertices.emplace_back(v_x, v_y, v_z);
      }

      index_offset += num_face_vertices;
    }

    if (multiple_shapes) {
      auto cvx_hull = new btConvexHullShape();
      cvx_hull->setMargin(COLLISION_MARGIN);
      for (const auto& vertex : vertices) {
        cvx_hull->addPoint(vertex, false);
      }
      cvx_hull->recalcLocalAabb();
      cvx_hull->optimizeConvexHull();
      static_cast<btCompoundShape*>(result.get())->addChildShape(identity, cvx_hull);
      vertices.clear();
    }
  }

  if (is_concave) {
    assert(!multiple_shapes);
    auto* trimesh = new btTriangleMesh();
    for (size_t i = 0; i < vertices.size(); i += 3) {
      trimesh->addTriangle(vertices[i], vertices[i + 1], vertices[i + 2]);
    }

    result = std::make_shared<btBvhTriangleMeshShape>(trimesh, true, true);
    result->setMargin(COLLISION_MARGIN);
  } else if (multiple_shapes) {
    static_cast<btCompoundShape*>(result.get())->recalculateLocalAabb();
  } else {
    result           = std::make_shared<btConvexHullShape>();
    auto* cvx_result = static_cast<btConvexHullShape*>(result.get());
    cvx_result->setMargin(COLLISION_MARGIN);
    for (const auto& vertex : vertices) {
      cvx_result->addPoint(vertex);
    }

    cvx_result->recalcLocalAabb();
    cvx_result->optimizeConvexHull();
  }

  assert(result != nullptr);
  return result;
}

std::shared_ptr<btCollisionShape> load_obj(const std::filesystem::path& obj_path,
                                           std::array<float, 3>& scale,
                                           bool is_concave = false) {
  return load_obj(obj_path, scale[0], scale[1], scale[2], is_concave);
}

std::shared_ptr<btCollisionShape> load_stl(const std::filesystem::path& stl_path,
                                           float x_scale   = 1.0,
                                           float y_scale   = 1.0,
                                           float z_scale   = 1.0,
                                           bool is_concave = false) {
  auto log                         = utils::get_logger("tmit-star::input::scene");
  const auto [shape_data, success] = tyti::stl::read(stl_path.string());
  if (!success) {
    log->error("Failed to read .stl file at {}!", stl_path.c_str());
    return nullptr;
  }

  std::vector<btVector3> vertices;
  vertices.reserve(shape_data.vertices.size());
  for (const auto& vertex : shape_data.vertices) {
    vertices.emplace_back(vertex.data[0] * x_scale,
                          vertex.data[1] * y_scale,
                          vertex.data[2] * z_scale);
  }

  if (is_concave) {
    auto* trimesh = new btTriangleMesh();
    for (size_t i = 0; i < vertices.size(); i += 3) {
      trimesh->addTriangle(vertices[i], vertices[i + 1], vertices[i + 2]);
    }

    return std::make_shared<btBvhTriangleMeshShape>(trimesh, true, true);
  } else {
    auto shape = std::make_shared<btConvexHullShape>();
    for (const auto& vertex : vertices) {
      shape->addPoint(vertex);
    }

    shape->optimizeConvexHull();
    shape->setMargin(COLLISION_MARGIN);
    shape->recalcLocalAabb();
    return shape;
  }
}

std::shared_ptr<btCollisionShape> load_stl(const std::filesystem::path& stl_path,
                                           std::array<float, 3>& scale,
                                           bool is_concave = false) {
  return load_stl(stl_path, scale[0], scale[1], scale[2], is_concave);
}

std::shared_ptr<btCollisionShape> get_shape(const std::filesystem::path& shape_path,
                                            ShapeCache& shape_cache,
                                            float x_scale,
                                            float y_scale,
                                            float z_scale,
                                            bool is_concave) {
  auto log = utils::get_logger("tmit-star::input::scene");
  log->debug("Getting object {}", shape_path.c_str());
  ShapeInfo shape_info{shape_path, x_scale, y_scale, z_scale};
  if (!shape_cache.contains(shape_info)) {
    log->debug("Loading object file from {}", shape_path.c_str());
    if (!std::filesystem::exists(shape_path)) {
      log->error("No object file at provided path: {}", shape_path.c_str());
      return nullptr;
    }

    std::shared_ptr<btCollisionShape> shape_ptr = nullptr;

    if (shape_path.extension() == ".obj") {
      shape_ptr = load_obj(shape_path, x_scale, y_scale, z_scale, is_concave);
    } else if (shape_path.extension() == ".stl") {
      shape_ptr = load_stl(shape_path, x_scale, y_scale, z_scale, is_concave);
    } else {
      log->error("Unsupported shape file type: {}", shape_path.extension().c_str());
      return nullptr;
    }

    if (shape_ptr != nullptr) {
      shape_cache.emplace(ShapeInfo{shape_path, x_scale, y_scale, z_scale}, shape_ptr);
    }

    return shape_ptr;
  } else {
    return shape_cache[shape_info];
  }
}

std::shared_ptr<btCollisionShape> get_shape(const std::filesystem::path& obj_path,
                                            ShapeCache& shape_cache,
                                            const std::array<float, 3>& scale,
                                            bool is_concave) {
  return get_shape(obj_path, shape_cache, scale[0], scale[1], scale[2], is_concave);
}

using json = nlohmann::json;
std::shared_ptr<btCollisionShape> load_box(const std::array<double, 3>& extents) {
  return std::make_shared<btBoxShape>(btVector3(extents[0], extents[1], extents[2]));
}

std::shared_ptr<btCollisionShape> get_shape(const json& shape_data) {
  const auto shape_type = shape_data["type"].get<std::string>();
  if (shape_type == "box") {
    return load_box(shape_data["extents"].get<std::array<double, 3>>());
  } else {
    throw std::logic_error("Support for shapes other than boxes hasn't been implemented yet!");
  }
}

std::shared_ptr<btCollisionShape>
urdf_collision_to_bullet_collision(const urdf::CollisionSharedPtr& urdf_collision,
                                   const std::filesystem::path& model_root_path,
                                   ShapeCache& shape_cache) {
  if (urdf_collision == nullptr) {
    return nullptr;
  }

  std::shared_ptr<btCollisionShape> bullet_collision(nullptr);
  switch (urdf_collision->geometry->type) {
    case urdf::Geometry::MESH: {
      const auto collision_geometry = static_cast<const urdf::Mesh&>(*urdf_collision->geometry);
      std::filesystem::path obj_path(collision_geometry.filename);
      if (obj_path.is_relative()) {
        obj_path = model_root_path / obj_path;
      }

      bullet_collision = get_shape(obj_path,
                                   shape_cache,
                                   collision_geometry.scale.x,
                                   collision_geometry.scale.y,
                                   collision_geometry.scale.z,
                                   false);
      break;
    }

    case urdf::Geometry::BOX: {
      const auto collision_geometry = static_cast<const urdf::Box&>(*urdf_collision->geometry);
      const auto x                  = collision_geometry.dim.x / 2.0;
      const auto y                  = collision_geometry.dim.y / 2.0;
      const auto z                  = collision_geometry.dim.z / 2.0;
      bullet_collision              = std::make_shared<btBoxShape>(btVector3(x, y, z));
      break;
    }

    case urdf::Geometry::CYLINDER: {
      const auto collision_geometry = static_cast<const urdf::Cylinder&>(*urdf_collision->geometry);
      // URDF cylinders are Z-aligned, and Bullet expects half-extents here
      bullet_collision = std::make_shared<btCylinderShapeZ>(btVector3(
      collision_geometry.radius, collision_geometry.radius, collision_geometry.length / 2.0));
      break;
    }

    case urdf::Geometry::SPHERE: {
      const auto collision_geometry = static_cast<const urdf::Sphere&>(*urdf_collision->geometry);
      bullet_collision              = std::make_shared<btSphereShape>(collision_geometry.radius);
      break;
    }
  }

  assert(bullet_collision != nullptr);
  return bullet_collision;
}

planner::geometric::Transform3<> urdf_pose_to_transform(const urdf::Pose& pose) {
  return Eigen::Translation3d(pose.position.x, pose.position.y, pose.position.z) *
         Eigen::Quaterniond(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
}

planner::geometric::Transform3<>
link_collision_transform(const urdf::CollisionSharedPtr& collision) {
  if (collision == nullptr) {
    return planner::geometric::Transform3<>::Identity();
  }

  return urdf_pose_to_transform(collision->origin);
}

void add_robot_joints(const urdf::LinkConstSharedPtr& link,
                      ScenegraphBuilder& sgb,
                      robin_hood::unordered_map<std::string, std::string>& link_joint_index,
                      ShapeCache& shape_cache,
                      const std::filesystem::path& model_path) {
  const auto& joint             = link->parent_joint;
  const auto urdf_axis          = joint->axis;
  const auto& parent_joint_name = link_joint_index.at(joint->parent_link_name);
  std::optional<planner::geometric::JointBounds> bounds = std::nullopt;
  if (joint->limits != nullptr) {
    bounds.emplace(planner::geometric::JointBounds{joint->limits->lower, joint->limits->upper});
  }

  // TODO: There's probably a cleaner way to do this, but this repetition avoids munging with
  // dynamically allocating & casting a pointer to the right Joint subtype. Is avoiding that worth
  // it? Unclear!
  switch (joint->type) {
    case urdf::Joint::PRISMATIC:
      sgb.add_robot_joint(
      joint->name,
      parent_joint_name,
      planner::geometric::PrismaticJoint(Eigen::Vector3d(urdf_axis.x, urdf_axis.y, urdf_axis.z),
                                         urdf_pose_to_transform(
                                         joint->parent_to_joint_origin_transform),
                                         -1),
      link_collision_transform(link->collision),
      urdf_collision_to_bullet_collision(link->collision, model_path, shape_cache),
      bounds);
      break;
    case urdf::Joint::REVOLUTE:
      sgb.add_robot_joint(
      joint->name,
      parent_joint_name,
      planner::geometric::RevoluteJoint(Eigen::Vector3d(urdf_axis.x, urdf_axis.y, urdf_axis.z),
                                        urdf_pose_to_transform(
                                        joint->parent_to_joint_origin_transform),
                                        -1),
      link_collision_transform(link->collision),
      urdf_collision_to_bullet_collision(link->collision, model_path, shape_cache),
      bounds);
      break;
    case urdf::Joint::CONTINUOUS:
      sgb.add_robot_joint(
      joint->name,
      parent_joint_name,
      planner::geometric::ContinuousJoint(Eigen::Vector3d(urdf_axis.x, urdf_axis.y, urdf_axis.z),
                                          urdf_pose_to_transform(
                                          joint->parent_to_joint_origin_transform),
                                          -1),
      link_collision_transform(link->collision),
      urdf_collision_to_bullet_collision(link->collision, model_path, shape_cache),
      bounds);
      break;
    case urdf::Joint::FIXED:
      sgb.add_robot_joint(
      joint->name,
      parent_joint_name,
      planner::geometric::FixedJoint(
      urdf_pose_to_transform(joint->parent_to_joint_origin_transform), -1),
      link_collision_transform(link->collision),
      urdf_collision_to_bullet_collision(link->collision, model_path, shape_cache),
      bounds);
      break;
    default:
      auto log = utils::get_logger("tmit-star::input::scene");
      log->warn("Unknown joint type for {}. Treating as fixed!", joint->name);
      sgb.add_robot_joint(
      joint->name,
      parent_joint_name,
      planner::geometric::FixedJoint(
      urdf_pose_to_transform(joint->parent_to_joint_origin_transform), -1),
      link_collision_transform(link->collision),
      urdf_collision_to_bullet_collision(link->collision, model_path, shape_cache),
      bounds);
  };

  link_joint_index.emplace(link->name, joint->name);
  for (const auto& child_link : link->child_links) {
    add_robot_joints(child_link, sgb, link_joint_index, shape_cache, model_path);
  }
}

bool load_urdf(const std::filesystem::path& urdf_path,
               ScenegraphBuilder& sgb,
               ShapeCache& shape_cache) {
  auto log         = utils::get_logger("tmit-star::input::scene");
  auto robot_model = urdf::parseURDFFile(urdf_path);
  if (robot_model == nullptr) {
    log->error("Failed to load robot model from {}!", urdf_path.c_str());
    return false;
  }

  auto model_root = robot_model->getRoot();
  planner::geometric::FixedJoint root_joint(planner::geometric::Transform3<>::Identity(), -1);
  const auto model_path = urdf_path.parent_path();
  sgb.add_robot_joint(
  model_root->name,
  std::nullopt,
  root_joint,
  urdf_pose_to_transform(model_root->collision->origin),
  urdf_collision_to_bullet_collision(model_root->collision, model_path, shape_cache),
  std::nullopt);
  robin_hood::unordered_map<std::string, std::string> link_joint_index;
  link_joint_index.emplace(model_root->name, model_root->name);
  for (const auto& child_link : model_root->child_links) {
    add_robot_joints(child_link, sgb, link_joint_index, shape_cache, model_path);
  }

  return true;
}

bool load_object(const json& object_spec, ScenegraphBuilder& sgb, ShapeCache& shape_cache) {
  auto log = utils::get_logger("tmit-star::input::scene");
  if (!object_spec.contains("name")) {
    log->error("Object does not specify a name!");
    return false;
  }

  const auto object_name = object_spec["name"].get<std::string>();
  GET_FIELD_WITH_DEFAULT(bool, is_static, true, object_spec, is_boolean);
  std::optional<std::string> object_parent = std::nullopt;
  if (object_spec.contains("parent_name")) {
    object_parent = object_spec["parent_name"].get<std::string>();
  }

  log->debug("Loading {} object {}", is_static ? "static" : "movable", object_name);
  if (!object_spec.contains("pose")) {
    log->error("Object does not provide an initial pose!");
    return false;
  }

  const auto translation = object_spec["pose"]["translation"].get<std::array<double, 3>>();
  const auto rotation    = object_spec["pose"]["rotation"].get<std::array<double, 4>>();
  // NOTE: The vector constructor of Quaterniond (used below) implicitly takes its coefficients in
  // (x, y, z, w) order, in contrast to the scalar constructor
  const auto object_pose = Eigen::Translation3d(translation[0], translation[1], translation[2]) *
                           Eigen::Quaterniond(rotation.data());

  std::optional<planner::geometric::AreaBounds> surface_bounds = std::nullopt;
  if (object_spec.contains("surface_bounds")) {
    const auto x_bounds = object_spec["surface_bounds"]["x"].get<std::array<double, 2>>();
    const auto y_bounds = object_spec["surface_bounds"]["y"].get<std::array<double, 2>>();
    const auto z_bounds = object_spec["surface_bounds"]["z"].get<std::array<double, 2>>();
    surface_bounds      = planner::geometric::AreaBounds{
    x_bounds[0], x_bounds[1], y_bounds[0], y_bounds[1], z_bounds[0], z_bounds[1]};
  }

  std::shared_ptr<btCollisionShape> obj_ptr{nullptr};
  if (object_spec.contains("obj_file")) {
    std::array<float, 3> obj_scale{1.0, 1.0, 1.0};
    if (object_spec.contains("scale")) {
      obj_scale = object_spec["scale"].get<std::array<float, 3>>();
    }

    GET_FIELD_WITH_DEFAULT(bool, is_concave, false, object_spec, is_boolean);
    std::filesystem::path obj_path(object_spec["obj_file"].get<std::string>());
    log->debug("Loading object file from {}", obj_path.c_str());
    obj_ptr = get_shape(obj_path, shape_cache, obj_scale, is_concave);
  } else if (object_spec.contains("shape")) {
    log->debug("Loading shape for {}", object_name);
    obj_ptr = get_shape(object_spec["shape"]);
  } else {
    log->error("Object {} did not specify a collision geometry!", object_name);
    return false;
  }

  planner::geometric::FixedJoint object_joint(object_pose, -1);
  if (is_static) {
    sgb.add_static_object(object_name, object_parent, object_joint, obj_ptr, surface_bounds);
  } else {
    sgb.add_movable_object(object_name, object_parent, object_joint, obj_ptr, surface_bounds);
  }

  return true;
}

std::optional<std::pair<std::tuple<double, double, double>, std::unordered_map<std::string, double>>>
load_robot(const json& robot_spec, ScenegraphBuilder& sgb, ShapeCache& shape_cache) {
  auto log = utils::get_logger("tmit-star::input::scene");
  ENSURE_SPEC_FIELD(robot_spec, "urdf", "No URDF file specified for robot!");
  std::filesystem::path urdf_path(robot_spec["urdf"].get<std::string>());
  if (!std::filesystem::exists(urdf_path)) {
    log->error("No URDF file at provided path!");
    return std::nullopt;
  }

  if (!load_urdf(urdf_path, sgb, shape_cache)) {
    return std::nullopt;
  }

  ENSURE_SPEC_FIELD(robot_spec, "base_pose", "No initial robot base pose provided!");
  // NOTE: We assume by convention that the robot base's z coordinate is always 0.0, and that it
  // is a planar mobile robot only capable of rotating around the z axis. This isn't
  // fundamental/inherent to the algorithm, though.
  const auto base_translation = robot_spec["base_pose"]["translation"].get<std::array<double, 2>>();
  const auto base_rotation    = robot_spec["base_pose"]["rotation"].get<double>();
  ENSURE_SPEC_FIELD(robot_spec, "joint_poses", "No initial robot joint poses provided!");
  auto joint_poses = robot_spec["joint_poses"].get<std::unordered_map<std::string, double>>();
  return std::make_pair(std::make_tuple(base_translation[0], base_translation[1], base_rotation),
                        joint_poses);
}

namespace input::geometric {
std::optional<SceneInfo> load(const std::filesystem::path& scene_path) {
  auto log = utils::get_logger("tmit-star::input::scene");
  log->debug("Loading robot, objects, and initial scene configuration from {}", scene_path.c_str());
  if (!std::filesystem::exists(scene_path)) {
    log->error("No scene file at provided path!");
    return std::nullopt;
  }

  std::ifstream scene_file(scene_path);
  if (!scene_file.is_open()) {
    log->error("Could not open initial scene file!");
    return std::nullopt;
  }

  json scene_spec;
  scene_file >> scene_spec;
  scene_file.close();
  ENSURE_SPEC_FIELD(scene_spec, "bounds", "Initial scene does not specify workspace bounds!");
  std::array<planner::geometric::Bounds, 3> bounds;
  for (size_t i = 0; i < 3; ++i) {
    const auto& dim_bounds = scene_spec["bounds"][i].get<std::array<float, 2>>();
    bounds[i].lower        = dim_bounds[0];
    bounds[i].upper        = dim_bounds[1];
  }

  ENSURE_SPEC_FIELD(scene_spec, "robot", "Initial scene does not specify a robot!");
  ScenegraphBuilder sg_builder;
  ShapeCache shape_cache;
  const auto robot_result = load_robot(scene_spec["robot"], sg_builder, shape_cache);
  if (!robot_result) {
    return std::nullopt;
  }

  if (scene_spec.contains("objects") && scene_spec["objects"].is_array()) {
    log->debug("Loading objects");
    if (std::any_of(scene_spec["objects"].begin(),
                    scene_spec["objects"].end(),
                    [&](const auto& object_spec) -> bool {
                      return !load_object(object_spec, sg_builder, shape_cache);
                    })) {
      return std::nullopt;
    }
  }

  const auto& [base_x, base_y, base_yaw] = robot_result->first;
  auto init_sg                           = sg_builder.build();
  const auto& joint_pose_map             = robot_result->second;
  std::vector<double> joint_poses(joint_pose_map.size(), 0.0);
  const auto& init_scene_info = init_sg.scene_info;
  for (const auto& [joint_name, pose] : joint_pose_map) {
    const auto joint_idx           = init_scene_info->index.at(joint_name);
    const auto configuration_idx   = init_sg.scene_info->joint_info[joint_idx].configuration_idx;
    joint_poses[configuration_idx] = pose;
  }

  return SceneInfo{bounds, std::move(init_sg), {base_x, base_y, base_yaw, joint_poses}};
}
}  // namespace input::geometric
