#ifndef COLLISIONS_HH
#define COLLISIONS_HH

#include <bullet/btBulletCollisionCommon.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateValidityChecker.h>

#include <array>
#include <boost/dynamic_bitset.hpp>
#include <vector>

#include "bounds.hh"
#include "mode_atlas.hh"
#include "scenegraph.hh"
#include "state.hh"

namespace planner {

struct CollisionObject : public btCollisionObject {
  bool is_movable_object;
  unsigned int index;
  int parent;
  boost::dynamic_bitset<> forbidden_collisions;
  bool checkCollideWithOverride(const btCollisionObject* co) const override;
  CollisionObject() : btCollisionObject() { m_checkCollideWith = 1; }
};

struct CollisionChecker : public ompl::base::StateValidityChecker {
  CollisionChecker(const ompl::base::SpaceInformationPtr& si,
                   ModeAtlas& mode_atlas,
                   const geometric::SceneGraph& initial_scenegraph,
                   const std::array<geometric::Bounds, 3>& bounds,
                   const std::filesystem::path& blocklist_file);
  bool isValid(const ompl::base::State* state) const override;

  const std::vector<geometric::Transform3<>>& get_last_poses() const { return poses; }

  ModeAtlas& mode_atlas;
  btDefaultCollisionConfiguration collision_configuration;
  mutable btCollisionDispatcher collision_dispatcher;
  btDbvtBroadphase broadphase_interface;
  mutable btCollisionWorld collision_world;
  mutable std::vector<CollisionObject> collision_objects;
  mutable std::vector<geometric::Transform3<>> poses;
  mutable std::vector<geometric::Transform3<>> collision_poses;
  const std::array<geometric::Bounds, 3> bounds;
};
}  // namespace planner
#endif
