#ifndef SAMPLER_HH
#define SAMPLER_HH
#include <memory>
#include <stdexcept>

#include "cspace.hh"
#include "lua_env.hh"
#include "mode_atlas.hh"
#include "ompl/base/Cost.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/State.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "solver.hh"
#include "state.hh"
#include "symbolic.hh"

namespace planner {
struct CompositeSampler : public ompl::base::StateSampler {
  CompositeSampler(const CompositeStateSpace *state_space,
                   symbolic::LuaEnv &lua_env, const SolverBounds &bounds);

  void sampleUniform(ompl::base::State *state) override;
  void sampleUniformNear(ompl::base::State *state,
                         const ompl::base::State *near,
                         double distance) override;
  void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean,
                      double stdDev) override;
  bool sample_transition_near(ompl::base::State *state,
                              const ompl::base::State *near,
                              const Transition *transition);
  double transition_distance(const ompl::base::State *state,
                             const std::vector<geometric::Transform3<>> &poses,
                             const Transition *transition);

protected:
  ModeAtlas *const mode_atlas;
  std::shared_ptr<ompl::base::StateSampler> base_pose_sampler;
  std::shared_ptr<ompl::base::StateSampler> joint_state_sampler;
  std::vector<double> state_vec;
  symbolic::LuaEnv &lua_env;
  FormulaSolver solver;
};

struct UninformedSampler : public ompl::base::InformedSampler {
  UninformedSampler(const ompl::base::ProblemDefinitionPtr &problem_def,
                    unsigned int max_number_calls);
  ~UninformedSampler() override = default;

  bool sampleUniform(ompl::base::State *state,
                     const ompl::base::Cost &max_cost) override;
  bool sampleUniform(ompl::base::State *state, const ompl::base::Cost &min_cost,
                     const ompl::base::Cost &max_cost) override;
  bool hasInformedMeasure() const override;
  double
  getInformedMeasure(const ompl::base::Cost &current_cost) const override;
  double getInformedMeasure(const ompl::base::Cost &min_cost,
                            const ompl::base::Cost &max_cost) const override;
  void setLocalSeed(std::uint_fast32_t /*localSeed*/) {
    // NOTE: Throws because vanilla OMPL does not support local seeds
    throw std::runtime_error(
        "setLocalSeed isn't implemented in official OMPL!");
    // base_sampler->setLocalSeed(localSeed);
  };
  double transition_distance(const ompl::base::State *state,
                             const std::vector<geometric::Transform3<>> &poses,
                             const Transition *transition) const;
  bool sample_transition_near(ompl::base::State *state,
                              const ompl::base::State *near,
                              const Transition *transition);

protected:
  std::shared_ptr<ompl::base::StateSampler> base_sampler;
};
} // namespace planner
#endif
