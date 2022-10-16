#include "collisions.hh"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <bullet/LinearMath/btIDebugDraw.h>
#include <bullet/btBulletCollisionCommon.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <boost/dynamic_bitset.hpp>
#include <filesystem>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <set>
#include <stdexcept>

#include "bounds.hh"
#include "cspace.hh"
#include "lazycsv.hpp"
#include "mode_atlas.hh"
#include "robin_hood.h"
#include "scenegraph.hh"
#include "utils.hh"

// This class adapted from https://github.com/opengl-tutorials/ogl/tree/master/misc05_picking
struct DebugDrawer : public btIDebugDraw {
  DebugDrawer(btCollisionWorld& collision_world) : collision_world(collision_world) {
    collision_world.setDebugDrawer(this);
  }

  void show_collisions() {
    glfwInit();
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
    window = glfwCreateWindow(1024, 768, "Debug Drawing", NULL, NULL);
    glfwMakeContextCurrent(window);
    glewInit();
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

    glEnable(GL_CULL_FACE);
    GLuint vertexBuffer;
    GLuint VertexArrayID;

    glGenBuffers(1, &vertexBuffer);
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
    setDebugMode(btIDebugDraw::DBG_DrawWireframe | btIDebugDraw::DBG_DrawContactPoints);

    while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
           glfwGetKey(window, GLFW_KEY_Q) != GLFW_PRESS && glfwWindowShouldClose(window) == 0) {
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      update_view();
      draw_axes();
      collision_world.debugDrawWorld();
      glfwSwapBuffers(window);
      glfwPollEvents();
    }

    glfwTerminate();
  }

  void drawLine(const btVector3& from, const btVector3& to, const btVector3& color) override {
    glColor3f(color.x(), color.y(), color.z());
    glBegin(GL_LINES);
    glVertex3f(from.x(), from.y(), from.z());
    glVertex3f(to.x(), to.y(), to.z());
    glEnd();
  }

  void drawContactPoint(const btVector3& PointOnB,
                        const btVector3& normalOnB,
                        btScalar distance,
                        int lifeTime,
                        const btVector3& color) override {
    glColor3f(color.x(), color.y(), color.z());
    glBegin(GL_POLYGON);
    glVertex3f(PointOnB.x() + 0.0314, PointOnB.y(), PointOnB.z() - 0.0111);
    glVertex3f(PointOnB.x() - 0.0157, PointOnB.y() + 0.0272, PointOnB.z() - 0.0111);
    glVertex3f(PointOnB.x() - 0.0157, PointOnB.y() - 0.0272, PointOnB.z() - 0.0111);
    glVertex3f(PointOnB.x(), PointOnB.y(), PointOnB.z() + 0.0333);
    glEnd();
  }

  void reportErrorWarning(const char*) override {}
  void draw3dText(const btVector3&, const char*) override {}
  void draw_axes() {
    drawLine(btVector3(0, 0, 0), btVector3(10, 0, 0), btVector3(1.0, 0.0, 0.0));
    drawLine(btVector3(0, 0, 0), btVector3(0, 10, 0), btVector3(0.0, 1.0, 0.0));
    drawLine(btVector3(0, 0, 0), btVector3(0, 0, 10), btVector3(0.0, 0.0, 1.0));
  }

  void setDebugMode(int p) override { m = p; }
  int getDebugMode(void) const override { return m; }
  int m;

  glm::mat4 ViewMatrix;
  glm::mat4 ProjectionMatrix;
  // Initial position : on +Z
  glm::vec3 position = glm::vec3(0, 0, 5);
  // Initial horizontal angle : toward -Z
  float horizontalAngle = 3.14f;
  // Initial vertical angle : none
  float verticalAngle = 0.0f;
  // Initial Field of View
  float initialFoV = 45.0f;

  float speed      = 3.0f;  // 3 units / second
  float mouseSpeed = 0.005f;
  void update_view() {
    static double lastTime = glfwGetTime();
    double currentTime     = glfwGetTime();
    float deltaTime        = float(currentTime - lastTime);
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);
    glfwSetCursorPos(window, 1024 / 2, 768 / 2);
    horizontalAngle += mouseSpeed * float(1024 / 2 - xpos);
    verticalAngle += mouseSpeed * float(768 / 2 - ypos);
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
      horizontalAngle += mouseSpeed;
    }

    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
      horizontalAngle -= mouseSpeed;
    }

    glm::vec3 direction(cos(verticalAngle) * sin(horizontalAngle),
                        sin(verticalAngle),
                        cos(verticalAngle) * cos(horizontalAngle));
    glm::vec3 right =
    glm::vec3(sin(horizontalAngle - 3.14f / 2.0f), 0, cos(horizontalAngle - 3.14f / 2.0f));
    glm::vec3 up = glm::cross(right, direction);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
      position += direction * deltaTime * speed;
    }

    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
      position -= direction * deltaTime * speed;
    }

    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
      position += right * deltaTime * speed;
    }

    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
      position -= right * deltaTime * speed;
    }

    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
      position += up * deltaTime * speed;
    }

    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
      position -= up * deltaTime * speed;
    }

    float FoV = initialFoV;

    ProjectionMatrix = glm::perspective(glm::radians(FoV), 4.0f / 4.0f, 0.1f, 100.0f);
    ViewMatrix       = glm::lookAt(position, position + direction, up);
    lastTime         = currentTime;
    SetMatrices();
  }

  void SetMatrices() {
    glUseProgram(0);
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(&ViewMatrix[0][0]);
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(&ProjectionMatrix[0][0]);
  }

  void flushLines() override {}
  btCollisionWorld& collision_world;
  GLFWwindow* window;
};

robin_hood::unordered_map<unsigned int, boost::dynamic_bitset<>>
build_collision_blocklists(const std::filesystem::path& blocklist_file,
                           const planner::geometric::SceneGraph& sg) {
  if (!std::filesystem::exists(blocklist_file)) {
    throw std::runtime_error("Blocklist file path does not exist!");
  }

  robin_hood::unordered_map<unsigned int, std::set<unsigned int>> blocklists;
  lazycsv::parser<lazycsv::mmap_source,
                  lazycsv::has_header<false>,
                  lazycsv::delimiter<','>,
                  lazycsv::trim_chars<' ', '\t', '\n', '\r'>>
  csv(blocklist_file.c_str());
  unsigned int row_count = 0;
  auto log               = utils::get_logger("exoplanet::collision");
  for (const auto& row : csv) {
    ++row_count;
    const auto& [link_1, link_2] = row.cells(0, 1);
    const std::string link_1_name(link_1.trimed());
    const std::string link_2_name(link_2.trimed());
    if (!sg.scene_info->index.contains(link_1_name) ||
        !sg.scene_info->index.contains(link_2_name)) {
      log->debug("Skipping blocked pair ({}, {}); one or both links isn't in the kinematic tree",
                 link_1_name,
                 link_2_name);
      continue;
    }

    const auto link_1_idx = sg.scene_info->index.at(link_1_name);
    const auto link_2_idx = sg.scene_info->index.at(link_2_name);
    if (!blocklists[link_1_idx].contains(link_2_idx)) {
      blocklists[link_1_idx].insert(link_2_idx);
    }

    if (!blocklists[link_2_idx].contains(link_1_idx)) {
      blocklists[link_2_idx].insert(link_1_idx);
    }
  }

  log->debug("Loaded {} blocked collision pairs", row_count);
  robin_hood::unordered_map<unsigned int, boost::dynamic_bitset<>> result;
  result.reserve(blocklists.size());
  for (const auto& [idx, blocklist] : blocklists) {
    boost::dynamic_bitset<> blocklist_bits(sg.joints.size());
    for (const auto blocked_idx : blocklist) {
      blocklist_bits[blocked_idx] = true;
    }

    result[idx] = blocklist_bits;
  }

  return result;
}

namespace planner {
constexpr double COLLISION_PENETRATION_DISTANCE = 0.029;
bool CollisionObject::checkCollideWithOverride(const btCollisionObject* co) const {
  const auto* cco = static_cast<const CollisionObject*>(co);
  // Forbid collisions with parent
  if (static_cast<unsigned int>(cco->parent) == index ||
      static_cast<unsigned int>(parent) == cco->index) {
    return false;
  }

  return !forbidden_collisions[cco->index];
}

CollisionChecker::CollisionChecker(const ompl::base::SpaceInformationPtr& si,
                                   ModeAtlas& mode_atlas,
                                   const geometric::SceneGraph& initial_scenegraph,
                                   const std::array<geometric::Bounds, 3>& bounds,
                                   const std::filesystem::path& blocklist_file)
: ompl::base::StateValidityChecker(si)
, mode_atlas(mode_atlas)
, collision_dispatcher(&collision_configuration)
, collision_world(&collision_dispatcher, &broadphase_interface, &collision_configuration)
, bounds(bounds) {
  auto log                       = utils::get_logger("exoplanet::collision");
  const auto& initial_scene_info = *initial_scenegraph.scene_info;
  collision_objects.reserve(initial_scene_info.collision_info.size());
  poses.reserve(initial_scene_info.joint_info.size());
  collision_poses.reserve(collision_objects.size());
  auto blocklists = build_collision_blocklists(blocklist_file, initial_scenegraph);
  for (const auto& joint_info : initial_scene_info.joint_info) {
    if (joint_info.collision_idx) {
      auto& collision             = collision_objects.emplace_back();
      collision.is_movable_object = joint_info.type == geometric::JointInfo::Type::MovableObject;
      collision.index             = joint_info.joint_idx;
      collision.parent            = initial_scenegraph.joints[joint_info.joint_idx].parent;
      if (blocklists.contains(joint_info.joint_idx)) {
        collision.forbidden_collisions = blocklists[joint_info.joint_idx];
      } else {
        collision.forbidden_collisions = boost::dynamic_bitset<>(initial_scenegraph.joints.size());
      }

      collision.setCollisionShape(initial_scene_info.geometries[*joint_info.collision_idx].get());
      collision.setWorldTransform(utils::transform3_to_btTransform(
      initial_scenegraph.joints[joint_info.joint_idx].tf *
      initial_scene_info.collision_info[*joint_info.collision_idx].collision_transform));
      collision.setUserIndex(joint_info.joint_idx);
      collision_world.addCollisionObject(&collision);
    }
  }
}

bool CollisionChecker::isValid(const ompl::base::State* state) const {
  if (!si_->satisfiesBounds(state)) {
    return false;
  }

  const auto* cstate = state->as<CompositeStateSpace::StateType>();
  const auto& mode   = cstate->get_mode();
  const auto& sg     = mode_atlas.get_mode_info(mode).sg;

  // Update link and moveable object poses
  poses.clear();
  collision_poses.clear();
  geometric::pose_all(sg, cstate->geometric_configuration(), poses);

  // Validate that all posed objects are within the workspace bounds
  if (std::any_of(poses.cbegin(), poses.cend(), [&](const auto& pose) {
        const auto& translation = pose.translation();
        const auto x            = translation.x();
        const auto y            = translation.y();
        const auto z            = translation.z();
        return (bounds[0].lower > x || bounds[0].upper < x) ||
               (bounds[1].lower > y || bounds[1].upper < y) ||
               (bounds[2].lower > z || bounds[2].upper < z);
      })) {
    return false;
  }

  // Update the collision object poses
  geometric::pose_all_collisions(sg, poses, collision_poses);
  for (size_t i = 0; i < collision_poses.size(); ++i) {
    const auto& collision_pose = collision_poses[i];
    const auto& translation    = collision_pose.translation();
    Eigen::Quaterniond rotation(collision_pose.linear());
    rotation.normalize();
    auto& collision = collision_objects[i];
    auto& world_tf  = collision.getWorldTransform();
    world_tf.setOrigin(btVector3(translation.x(), translation.y(), translation.z()));
    world_tf.setRotation(btQuaternion(rotation.x(), rotation.y(), rotation.z(), rotation.w()));
    collision.setWorldTransform(world_tf);
    if (collision.is_movable_object) {
      collision.parent = sg.joints[collision.index].parent;
    }
  }

  // Check collisions
  collision_world.performDiscreteCollisionDetection();
  const auto num_manifolds = collision_dispatcher.getNumManifolds();
  for (int i = 0; i < num_manifolds; ++i) {
    auto* manifold          = collision_dispatcher.getManifoldByIndexInternal(i);
    const auto num_contacts = manifold->getNumContacts();
    for (int j = 0; j < num_contacts; ++j) {
      const auto& contact = manifold->getContactPoint(j);
      if (contact.getDistance() <= -COLLISION_PENETRATION_DISTANCE) {
        collision_dispatcher.clearManifold(manifold);
        return false;
      }
    }

    collision_dispatcher.clearManifold(manifold);
  }

  return true;
}
}  // namespace planner
