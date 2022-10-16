#include "cspace.hh"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <numbers>

#include "mode_atlas.hh"
#include "ompl/base/State.h"
#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"
#include "scenegraph.hh"
#include "state.hh"

namespace planner {
auto KinematicTreeStateSpace::allocState() const -> ompl::base::State* {
  auto* state = new StateType(num_joints);
  return state;
}

void KinematicTreeStateSpace::freeState(ompl::base::State* state) const {
  auto* kts = static_cast<StateType*>(state);
  delete kts;
}

void KinematicTreeStateSpace::registerProjections() {}

auto KinematicTreeStateSpace::StateType::get_joint_states() const -> const double* {
  return joint_states.data();
}

void KinematicTreeStateSpace::StateType::set_joint_states(const std::vector<double>& joint_values) {
  assert(joint_values.size() == joint_states.size());
  joint_states =
  Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(joint_values.data(), joint_values.size());
}

void KinematicTreeStateSpace::StateType::set_joint_states(std::vector<double>::const_iterator start,
                                                          std::vector<double>::const_iterator end) {
  std::copy(start, end, joint_states.data());
}

void KinematicTreeStateSpace::StateType::set_joint_states(
const KinematicTreeStateSpace::StateType& new_joint_state) {
  joint_states = new_joint_state.joint_states;
}

auto KinematicTreeStateSpace::StateType::get_joint_state(Eigen::Index idx) const -> double {
  return joint_states[idx];
}

void KinematicTreeStateSpace::StateType::set_joint_state(Eigen::Index idx, double value) {
  joint_states[idx] = value;
}

void KinematicTreeStateSpace::add_continuous_joint() {
  add_joint(-std::numbers::pi, std::numbers::pi);
}

void KinematicTreeStateSpace::add_joint(double lower_bound, double upper_bound) {
  ++num_joints;
  lower_bounds.conservativeResize(num_joints);
  upper_bounds.conservativeResize(num_joints);
  lower_bounds[num_joints - 1] = lower_bound;
  upper_bounds[num_joints - 1] = upper_bound;
}

KinematicTreeStateSpace::KinematicTreeStateSpace() : num_joints{0} {}

auto KinematicTreeStateSpace::distance(const ompl::base::State* state_1,
                                       const ompl::base::State* state_2) const -> double {
  // static_cast is OK here because we know this is only ever called with
  // KinematicTreeStateSpace::StateType values
  const auto* kts_1 = static_cast<const StateType*>(state_1);
  const auto* kts_2 = static_cast<const StateType*>(state_2);
  // NOTE: This treats continuous joints as (-pi, pi)-bounded regular joints, which ignores the
  // SO(2) metric. OK in practice
  return (kts_1->joint_states - kts_2->joint_states).norm();
}
auto KinematicTreeStateSpace::getDimension() const -> unsigned int { return num_joints; }
auto KinematicTreeStateSpace::getMaximumExtent() const -> double {
  return (upper_bounds - lower_bounds).norm();
}

auto KinematicTreeStateSpace::getMeasure() const -> double {
  return (upper_bounds - lower_bounds).prod();
}

void KinematicTreeStateSpace::enforceBounds(ompl::base::State* state) const {
  auto* kts         = static_cast<StateType*>(state);
  kts->joint_states = kts->joint_states.cwiseMin(upper_bounds).cwiseMax(lower_bounds);
}

auto KinematicTreeStateSpace::satisfiesBounds(const ompl::base::State* state) const -> bool {
  const auto* const kts = static_cast<const StateType* const>(state);
  return (kts->joint_states.array() <= upper_bounds.array()).all() &&
         (kts->joint_states.array() >= lower_bounds.array()).all();
}

void KinematicTreeStateSpace::copyState(ompl::base::State* destination,
                                        const ompl::base::State* source) const {
  auto* kts_dest            = static_cast<StateType*>(destination);
  const auto* const kts_src = static_cast<const StateType* const>(source);
  kts_dest->joint_states    = kts_src->joint_states;
}

auto KinematicTreeStateSpace::equalStates(const ompl::base::State* state_1,
                                          const ompl::base::State* state_2) const -> bool {
  const auto* const kts_1 = static_cast<const StateType* const>(state_1);
  const auto* const kts_2 = static_cast<const StateType* const>(state_2);
  return kts_1->joint_states.isApprox(kts_2->joint_states);
}

void KinematicTreeStateSpace::interpolate(const ompl::base::State* from,
                                          const ompl::base::State* to,
                                          double t,
                                          ompl::base::State* state) const {
  const auto* const kts_from = static_cast<const StateType* const>(from);
  const auto* const kts_to   = static_cast<const StateType* const>(to);
  auto* kts_dest             = static_cast<StateType*>(state);
  kts_dest->joint_states =
  kts_from->joint_states + t * (kts_to->joint_states - kts_from->joint_states);
}

void KinematicTreeStateSpace::Sampler::sampleUniform(ompl::base::State* state) {
  auto* kts = static_cast<StateType*>(state);
  kts->joint_states =
  lower_bounds.array() +
  (Eigen::VectorXd::Random(num_joints).array() * 0.5 + 0.5) * (upper_bounds - lower_bounds).array();
}

void KinematicTreeStateSpace::Sampler::sampleUniformNear(ompl::base::State* state,
                                                         const ompl::base::State* near,
                                                         double distance) {
  throw std::runtime_error("sampleUniformNear is not implemented");
}

void KinematicTreeStateSpace::Sampler::sampleGaussian(ompl::base::State* state,
                                                      const ompl::base::State* mean,
                                                      double stdDev) {
  throw std::runtime_error("sampleGaussian is not implemented");
}

auto KinematicTreeStateSpace::allocDefaultStateSampler() const -> ompl::base::StateSamplerPtr {
  return std::make_shared<Sampler>(this, num_joints, lower_bounds, upper_bounds);
}

const ompl::base::SE2StateSpace::StateType& CompositeStateSpace::StateType::get_base_pose() const {
  return *as<ompl::base::SE2StateSpace::StateType>(BASE_SPACE_IDX);
}

void CompositeStateSpace::StateType::set_base_pose(double x, double y, double rotation) {
  auto* base_state = as<ompl::base::SE2StateSpace::StateType>(BASE_SPACE_IDX);
  base_state->setXY(x, y);
  base_state->setYaw(rotation);
}

void CompositeStateSpace::StateType::set_base_state(
const ompl::base::SE2StateSpace::StateType& new_base_state) {
  auto* old_base_state = as<ompl::base::SE2StateSpace::StateType>(BASE_SPACE_IDX);
  old_base_state->setXY(new_base_state.getX(), new_base_state.getY());
  old_base_state->setYaw(new_base_state.getYaw());
}

// Controllable joints
const KinematicTreeStateSpace::StateType& CompositeStateSpace::StateType::get_joint_states() const {
  return *as<KinematicTreeStateSpace::StateType>(JOINT_SPACE_IDX);
}

double CompositeStateSpace::StateType::get_joint_state(const std::string& joint_name) const {
  return as<KinematicTreeStateSpace::StateType>(JOINT_SPACE_IDX)
  ->get_joint_state(joint_indices->at(joint_name));
}

void CompositeStateSpace::StateType::set_joint_state(const std::string& joint_name,
                                                     double joint_value) {
  as<KinematicTreeStateSpace::StateType>(JOINT_SPACE_IDX)
  ->set_joint_state(joint_indices->at(joint_name), joint_value);
}

void CompositeStateSpace::StateType::set_joint_states(const std::vector<double>& joint_values) {
  as<KinematicTreeStateSpace::StateType>(JOINT_SPACE_IDX)->set_joint_states(joint_values);
}

void CompositeStateSpace::StateType::set_joint_states(
const KinematicTreeStateSpace::StateType& new_joint_state) {
  as<KinematicTreeStateSpace::StateType>(JOINT_SPACE_IDX)->set_joint_states(new_joint_state);
}

// Symbolic state
auto CompositeStateSpace::StateType::get_mode() const -> const Mode& { return mode; }

void CompositeStateSpace::StateType::set_mode(const Mode& mode) { this->mode = mode; }

auto CompositeStateSpace::StateType::geometric_configuration() const -> geometric::State<> {
  const auto& base_state = get_base_pose();
  // NOTE: This could be more efficient if we somehow don't allocate the vector of joint
  // values every time
  const auto* joint_state_data = get_joint_states().get_joint_states();
  return geometric::State<>{base_state.getX(),
                            base_state.getY(),
                            base_state.getYaw(),
                            std::vector<double>(joint_state_data,
                                                joint_state_data + get_joint_states().num_joints)};
}

CompositeStateSpace::CompositeStateSpace(
ModeAtlas* const mode_atlas,
robin_hood::unordered_map<std::string, size_t>& symbol_indices,
planner::geometric::SceneGraph* const sg,
const ompl::base::RealVectorBounds& workspace_bounds)
: mode_atlas(mode_atlas), symbol_indices(symbol_indices), workspace_bounds(workspace_bounds) {
  setName("Composite Space");
  type_ = ompl::base::STATE_SPACE_UNKNOWN;

  // Add the robot base pose space
  addSubspace(std::make_shared<ompl::base::SE2StateSpace>(), 1.0);
  as<ompl::base::SE2StateSpace>(BASE_SPACE_IDX)->setName("Robot base pose");
  set_workspace_bounds(workspace_bounds);

  // Add the controllable robot joint space
  addSubspace(std::make_shared<KinematicTreeStateSpace>(), 1.0);
  auto* joints_subspace = as<KinematicTreeStateSpace>(JOINT_SPACE_IDX);
  joints_subspace->setName("Joint angles");
  auto num_movable_joints = 0;
  const auto& joint_index = sg->scene_info->index;
  const auto& joint_info  = sg->scene_info->joint_info;
  for (size_t i = 0; i < joint_index.size(); ++i) {
    const auto& joint     = joint_info[i];
    const auto joint_type = sg->joints[joint.joint_idx].type;
    if (joint.type == geometric::JointInfo::Type::RobotJoint &&
        joint_type != geometric::Joint::Type::Fixed) {
      if (joint_type == geometric::Joint::Type::Continuous) {
        joints_subspace->add_continuous_joint();
      } else {
        auto lower = -std::numeric_limits<double>::infinity();
        auto upper = std::numeric_limits<double>::infinity();
        if (joint.bounds) {
          lower = joint.bounds->lower;
          upper = joint.bounds->upper;
        }

        joints_subspace->add_joint(lower, upper);
      }

      joint_indices.emplace(joint.name, num_movable_joints);
      ++num_movable_joints;
    }
  }

  lock();
}

void CompositeStateSpace::set_workspace_bounds(const ompl::base::RealVectorBounds& bounds) {
  workspace_bounds = bounds;
  as<ompl::base::SE2StateSpace>(BASE_SPACE_IDX)->setBounds(bounds);
}

const ompl::base::RealVectorBounds& CompositeStateSpace::get_workspace_bounds() const {
  return workspace_bounds;
}

ompl::base::State* CompositeStateSpace::allocState() const {
  auto* state = new StateType(this, &joint_indices);
  allocStateComponents(state);
  return state;
}

void CompositeStateSpace::freeState(ompl::base::State* state) const {
  auto* cstate = static_cast<StateType*>(state);
  for (unsigned int i = 0; i < componentCount_; ++i)
    components_[i]->freeState(cstate->components[i]);
  delete[] cstate->components;
  delete cstate;
}

bool CompositeStateSpace::isMetricSpace() const { return false; }

bool valid_pose_difference(const geometric::SceneGraph& mode_sg_1,
                           const geometric::SceneGraph& mode_sg_2,
                           size_t object_idx) {
  // Test if movable object i is "allowed" to be different between these two modes, i.e. it is
  // being manipulated in at least one of the modes
  return mode_sg_1.has_robot_parent(object_idx) || mode_sg_2.has_robot_parent(object_idx);
}

double
CompositeStateSpace::continuous_distance(const CompositeStateSpace::StateType* const state_1,
                                         const CompositeStateSpace::StateType* const state_2) const {
  double dist = as<KinematicTreeStateSpace>(JOINT_SPACE_IDX)
                ->distance(state_1->as<KinematicTreeStateSpace::StateType>(JOINT_SPACE_IDX),
                           state_2->as<KinematicTreeStateSpace::StateType>(JOINT_SPACE_IDX));
  // Then the base
  const double x1     = state_1->as<ompl::base::SE2StateSpace::StateType>(BASE_SPACE_IDX)->getX();
  const double x2     = state_2->as<ompl::base::SE2StateSpace::StateType>(BASE_SPACE_IDX)->getX();
  const double x_diff = x1 - x2;
  const double y1     = state_1->as<ompl::base::SE2StateSpace::StateType>(BASE_SPACE_IDX)->getY();
  const double y2     = state_2->as<ompl::base::SE2StateSpace::StateType>(BASE_SPACE_IDX)->getY();
  const double y_diff = y1 - y2;
  dist += std::sqrt(x_diff * x_diff + y_diff * y_diff);
  const double z1 = state_1->as<ompl::base::SE2StateSpace::StateType>(BASE_SPACE_IDX)->getYaw();
  const double z2 = state_2->as<ompl::base::SE2StateSpace::StateType>(BASE_SPACE_IDX)->getYaw();
  const double d  = std::fabs(z1 - z2);
  dist += (d > std::numbers::pi_v<double>) ? 2.0 * std::numbers::pi_v<double> - d : d;
  return dist;
}

bool transitions_to(const CompositeStateSpace::StateType* state_1,
                    const CompositeStateSpace::StateType* state_2) {
  return state_1->transition && state_1->transition_mode == state_2->get_mode();
}

double CompositeStateSpace::distance(const ompl::base::State* state1,
                                     const ompl::base::State* state2) const {
  const auto* cstate_1 = state1->as<CompositeStateSpace::StateType>();
  const auto* cstate_2 = state2->as<CompositeStateSpace::StateType>();
  if (cstate_1->get_mode() == cstate_2->get_mode() || transitions_to(cstate_1, cstate_2) ||
      transitions_to(cstate_2, cstate_1)) {
    return continuous_distance(cstate_1, cstate_2);
  }

  return std::numeric_limits<double>::infinity();
}

#define INTERPOLATE(space_type, space_idx)                                            \
  as<space_type>(space_idx)->interpolate(cfrom->as<space_type::StateType>(space_idx), \
                                         cto->as<space_type::StateType>(space_idx),   \
                                         t,                                           \
                                         cstate->as<space_type::StateType>(space_idx));
#define COPY_STATE(space_type, space_idx)                                            \
  as<space_type>(space_idx)->copyState(cstate->as<space_type::StateType>(space_idx), \
                                       cfrom->as<space_type::StateType>(space_idx));

void CompositeStateSpace::copyState(ompl::base::State* destination,
                                    const ompl::base::State* source) const {
  ompl::base::CompoundStateSpace::copyState(destination, source);
  auto* cdest         = destination->as<CompositeStateSpace::StateType>();
  const auto* csource = source->as<CompositeStateSpace::StateType>();
  cdest->mode         = csource->mode;
  if (csource->transition) {
    cdest->transition.emplace(*csource->transition);
    cdest->transition_mode = csource->transition_mode;
  }
}

void CompositeStateSpace::interpolate(const ompl::base::State* from,
                                      const ompl::base::State* to,
                                      double t,
                                      ompl::base::State* state) const {
  // As a bit of a hack, we just interpolate in the robot state and then run FK (during collision
  // checking) to pose any manipulated objects
  const auto* cfrom = from->as<StateType>();
  const auto* cto   = to->as<StateType>();
  auto* cstate      = state->as<StateType>();

  // We can't be making a transition in an interpolated state
  cstate->transition = std::nullopt;

  // Interpolate the robot state
  INTERPOLATE(ompl::base::SE2StateSpace, BASE_SPACE_IDX);
  INTERPOLATE(KinematicTreeStateSpace, JOINT_SPACE_IDX);

  // Copy the symbolic state
  cstate->mode = cfrom->mode;
}

bool CompositeStateSpace::equalStates(const ompl::base::State* state1,
                                      const ompl::base::State* state2) const {
  const auto* cstate1 = state1->as<StateType>();
  const auto* cstate2 = state2->as<StateType>();

  if (cstate1->mode != cstate2->mode) {
    return false;
  }

  if (!components_[BASE_SPACE_IDX]->equalStates(cstate1->components[BASE_SPACE_IDX],
                                                cstate2->components[BASE_SPACE_IDX])) {
    return false;
  }

  return components_[JOINT_SPACE_IDX]->equalStates(cstate1->components[JOINT_SPACE_IDX],
                                                   cstate2->components[JOINT_SPACE_IDX]);
}
}  // namespace planner
