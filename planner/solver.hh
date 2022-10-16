#ifndef SOLVER_HH
#define SOLVER_HH
#include <array>
#include <nlopt.hpp>
#include <vector>

#include "autodiff/forward/real.hpp"
#include "cspace.hh"
#include "lua_env.hh"
#include "scenegraph.hh"
#include "state.hh"
#include "symbolic.hh"

namespace planner {
struct OptData {
  OptData(size_t num_joints) {
    diff_state.joint_poses.resize(num_joints, 0.0);
    state.joint_poses.resize(num_joints, 0.0);
  }

  symbolic::LuaEnv* lua_env;
  const geometric::SceneGraph* sg;
  const symbolic::Constraint* formula;
  geometric::State<autodiff::real> diff_state;
  geometric::State<> state;
  std::vector<geometric::Transform3<>> poses;
  std::vector<geometric::PoseSetPartialDerivative> pose_partial_derivatives;
};

struct SolverBounds {
  std::array<geometric::Bounds, 3> bounds;
  std::vector<geometric::JointBounds>& joint_bounds;
};

struct FormulaSolver {
  FormulaSolver(size_t grad_len);

  void setup_bounds(const SolverBounds& bounds);
  bool solve(symbolic::LuaEnv& lua_env,
             const symbolic::Constraint& formula,
             const geometric::SceneGraph& sg,
             std::vector<double>& state_vec);

  nlopt::opt optimizer;
  nlopt::opt local_optimizer;
  OptData opt_data;
  std::vector<double> lower_bounds;
  std::vector<double> upper_bounds;
  double last_value;
};

void fill_state_vec(const CompositeStateSpace::StateType* const state,
                    std::vector<double>& state_vec);
void extract_state(const std::vector<double>& state_vec,
                   CompositeStateSpace::StateType* const state);
}  // namespace planner
#endif
