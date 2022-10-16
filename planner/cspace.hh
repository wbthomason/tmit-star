#ifndef CSPACE_HH
#define CSPACE_HH
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/dynamic_bitset.hpp>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "bounds.hh"
#include "mode.hh"
#include "mode_atlas.hh"
#include "ompl/base/State.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/StateSpace.h"
#include "ompl/base/StateSpaceTypes.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "robin_hood.h"
#include "scenegraph.hh"
#include "state.hh"

namespace planner {
struct KinematicTreeStateSpace : public ompl::base::StateSpace {
  struct StateType : public ompl::base::State {
    friend struct KinematicTreeStateSpace;
    explicit StateType(Eigen::Index num_joints)
    : num_joints(num_joints), joint_states(num_joints) {}
    [[nodiscard]] auto get_joint_state(Eigen::Index idx) const -> double;
    void set_joint_state(Eigen::Index idx, double value);
    [[nodiscard]] auto get_joint_states() const -> const double*;
    void set_joint_states(const std::vector<double>& joint_values);
    void set_joint_states(std::vector<double>::const_iterator start,
                          std::vector<double>::const_iterator end);
    void set_joint_states(const StateType& new_joint_state);

    const Eigen::Index num_joints;

   protected:
    Eigen::VectorXd joint_states;
  };

  struct Sampler : public ompl::base::StateSampler {
    Sampler(const ompl::base::StateSpace* space,
            Eigen::Index num_joints,
            Eigen::VectorXd lower_bounds,
            Eigen::VectorXd upper_bounds)
    : ompl::base::StateSampler(space)
    , num_joints{num_joints}
    , lower_bounds(std::move(lower_bounds))
    , upper_bounds(std::move(upper_bounds)) {}
    ~Sampler() override = default;
    void sampleUniform(ompl::base::State* state) override;
    void sampleUniformNear(ompl::base::State* state,
                           const ompl::base::State* near,
                           double distance) override;
    void
    sampleGaussian(ompl::base::State* state, const ompl::base::State* mean, double stdDev) override;

   protected:
    const Eigen::Index num_joints;
    const Eigen::VectorXd lower_bounds;
    const Eigen::VectorXd upper_bounds;
  };

  KinematicTreeStateSpace();
  ~KinematicTreeStateSpace() override = default;

  void add_continuous_joint();
  void add_joint(double lower_bound, double upper_bound);

  [[nodiscard]] auto allocState() const -> ompl::base::State* override;
  void freeState(ompl::base::State* state) const override;
  void registerProjections() override;
  auto distance(const ompl::base::State* state_1, const ompl::base::State* state_2) const
  -> double override;
  [[nodiscard]] auto getDimension() const -> unsigned int override;
  [[nodiscard]] auto getMaximumExtent() const -> double override;
  [[nodiscard]] auto getMeasure() const -> double override;
  void enforceBounds(ompl::base::State* state) const override;
  auto satisfiesBounds(const ompl::base::State* state) const -> bool override;
  void copyState(ompl::base::State* destination, const ompl::base::State* source) const override;
  auto equalStates(const ompl::base::State* state_1, const ompl::base::State* state_2) const
  -> bool override;
  void interpolate(const ompl::base::State* from,
                   const ompl::base::State* to,
                   double t,
                   ompl::base::State* state) const override;
  [[nodiscard]] auto allocDefaultStateSampler() const -> ompl::base::StateSamplerPtr override;

 protected:
  Eigen::VectorXd lower_bounds;
  Eigen::VectorXd upper_bounds;
  Eigen::Index num_joints;
};

constexpr unsigned int BASE_SPACE_IDX  = 0;
constexpr unsigned int JOINT_SPACE_IDX = 1;

struct CompositeStateSpace : public ompl::base::CompoundStateSpace {
  struct StateType : public ompl::base::CompoundStateSpace::StateType {
    StateType(const StateType&) = delete;
    StateType(const ompl::base::StateSpace* const space,
              const robin_hood::unordered_map<std::string, size_t>* const joint_indices)
    : space(space), joint_indices(joint_indices), num_robot_joints(joint_indices->size()) {}

    // Conversion to State<>
    geometric::State<> geometric_configuration() const;

    // Base pose
    const ompl::base::SE2StateSpace::StateType& get_base_pose() const;
    void set_base_pose(double x, double y, double rotation);
    void set_base_state(const ompl::base::SE2StateSpace::StateType& new_base_state);

    // Controllable joints
    const KinematicTreeStateSpace::StateType& get_joint_states() const;
    double get_joint_state(const std::string& joint_name) const;
    void set_joint_state(const std::string& joint_name, double joint_value);
    void set_joint_states(const std::vector<double>& joint_values);
    void set_joint_states(const KinematicTreeStateSpace::StateType& new_joint_state);

    // Symbolic state: symbols and object pose set
    Mode mode;
    const Mode& get_mode() const;
    void set_mode(const Mode& mode);

    std::optional<Transition> transition;
    Mode transition_mode;

    bool operator==(const StateType& rhs) const { return space->equalStates(this, &rhs); }

   protected:
    const ompl::base::StateSpace* const space;
    const robin_hood::unordered_map<std::string, size_t>* const joint_indices;
    const size_t num_robot_joints;
  };

  CompositeStateSpace(ModeAtlas* const mode_atlas,
                      robin_hood::unordered_map<std::string, size_t>& symbol_indices,
                      planner::geometric::SceneGraph* const sg,
                      const ompl::base::RealVectorBounds& workspace_bounds);
  ~CompositeStateSpace() override = default;
  void set_workspace_bounds(const ompl::base::RealVectorBounds& bounds);
  const ompl::base::RealVectorBounds& get_workspace_bounds() const;
  ompl::base::State* allocState() const override;
  void freeState(ompl::base::State* state) const override;
  bool isMetricSpace() const override;
  void interpolate(const ompl::base::State* from,
                   const ompl::base::State* to,
                   double t,
                   ompl::base::State* state) const override;
  double distance(const ompl::base::State* state1, const ompl::base::State* state2) const override;
  void copyState(ompl::base::State* destination, const ompl::base::State* source) const override;
  bool equalStates(const ompl::base::State* state1, const ompl::base::State* state2) const override;

  size_t get_num_controllable_joints() const { return joint_indices.size(); }
  double continuous_distance(const StateType* const state_1, const StateType* const state_2) const;

  mutable ModeAtlas* mode_atlas;

 protected:
  robin_hood::unordered_map<std::string, size_t> joint_indices;
  robin_hood::unordered_map<std::string, size_t> symbol_indices;
  ompl::base::RealVectorBounds workspace_bounds;
};
}  // namespace planner
#endif
