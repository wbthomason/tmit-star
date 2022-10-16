#ifndef SYMBOLIC_HH
#define SYMBOLIC_HH
#include <fmt/format.h>

#include <boost/dynamic_bitset.hpp>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "autodiff/forward/real.hpp"
#include "expression_tree.hh"
#include "lua_env.hh"
#include "plan_context_fwd.hh"
#include "predicate.hh"
#include "robin_hood.h"
#include "scenegraph.hh"
#include "state.hh"
#include "utils.hh"
#include "z3++.h"

namespace planner::symbolic {
boost::dynamic_bitset<>
bitset_from_pred_list(const std::vector<const Predicate*>& predicates, const size_t num_predicates);

struct Formula {
  explicit Formula(std::shared_ptr<ExpressionNode> expression)
  : expression(std::move(expression)) {}

  double
  distance_at(LuaEnv& lua_env, const PosedScene<planner::geometric::Transform3<>>& state) const {
    return expression->distance_at(lua_env, state);
  }

  double distance_at(LuaEnv& lua_env,
                     const planner::geometric::SceneGraph& sg,
                     const planner::geometric::State<>& state) const {
    std::vector<planner::geometric::Transform3<>> poses;
    planner::geometric::pose_all(sg, state, poses);
    return distance_at(lua_env, std::move(PosedScene(sg, poses)));
  }

  double gradient_at(LuaEnv& lua_env,
                     const PosedScene<planner::geometric::PoseSetPartialDerivative>& state,
                     Eigen::Ref<Eigen::VectorXd> gradient) const {
    return expression->gradient_at(lua_env, state, gradient);
  }

  double gradient_at(LuaEnv& lua_env,
                     const planner::geometric::SceneGraph& sg,
                     planner::geometric::State<autodiff::real>& state,
                     Eigen::Ref<Eigen::VectorXd> gradient) const {
    return gradient_at(
    lua_env,
    std::move(PosedScene(sg, std::move(planner::geometric::pose_partial_derivatives(sg, state)))),
    gradient);
  }

  z3::expr add_at_step(PlanContext& plan_context, unsigned int step) const {
    fmt::memory_buffer name_buf;
    return expression->add_at_step(plan_context, step, &name_buf);
  }

  std::shared_ptr<ExpressionNode> expression;
};

std::vector<const Predicate*> gather_predicates(const Formula& formula);
constexpr double CONSTRAINT_TOLERANCE = 1e-3;

struct Constraint {
  explicit Constraint(Formula&& continuous_constraint)
  : constraint_formula(std::move(continuous_constraint)) {}

  z3::expr add_at_step(PlanContext& plan_context, unsigned int step) const {
    return constraint_formula.add_at_step(plan_context, step);
  }

  bool operator()(LuaEnv& lua_env,
                  const planner::geometric::SceneGraph& sg,
                  const planner::geometric::State<>& state) const {
    return is_satisfied_at(lua_env, sg, state);
  }

  bool is_satisfied_at(LuaEnv& lua_env,
                       const planner::geometric::SceneGraph& sg,
                       const planner::geometric::State<>& state) const {
    return distance_at(lua_env, sg, state) < CONSTRAINT_TOLERANCE;
  }

  double distance_at(LuaEnv& lua_env, const PosedScene<geometric::Transform3<>>& poses) const {
    return constraint_formula.distance_at(lua_env, poses);
  }

  double distance_at(LuaEnv& lua_env,
                     const planner::geometric::SceneGraph& sg,
                     const planner::geometric::State<>& state) const {
    return constraint_formula.distance_at(lua_env, sg, state);
  }

  double gradient_at(LuaEnv& lua_env,
                     const PosedScene<geometric::PoseSetPartialDerivative>& poses,
                     Eigen::Ref<Eigen::VectorXd> gradient) const {
    return constraint_formula.gradient_at(lua_env, poses, gradient);
  }

  double gradient_at(LuaEnv& lua_env,
                     const planner::geometric::SceneGraph& sg,
                     planner::geometric::State<autodiff::real>& state,
                     Eigen::VectorXd& gradient) const {
    return constraint_formula.gradient_at(lua_env, sg, state, gradient);
  }

  std::pair<double, Eigen::VectorXd>
  gradient_at(LuaEnv& lua_env,
              const planner::geometric::SceneGraph& sg,
              planner::geometric::State<autodiff::real>& state) const {
    // Base state is (x, y, yaw)
    Eigen::VectorXd result(3 + state.joint_poses.size());
    result.setZero();
    const auto dist = gradient_at(lua_env, sg, state, result);
    return {dist, result};
  }

  Formula constraint_formula;
};

struct Effect {
  Effect(std::vector<const Predicate*>&& add, std::vector<const Predicate*>&& del)
  : add(std::move(add)), del(std::move(del)) {}

  boost::dynamic_bitset<> apply(const planner::symbolic::State& mode) const;
  z3::expr add_at_step(PlanContext& plan_context, unsigned int step) const;

  void set_bits(size_t num_predicates) {
    add_bits = bitset_from_pred_list(add, num_predicates);
    del_bits = bitset_from_pred_list(del, num_predicates).flip();
  }

  std::vector<const Predicate*> add;
  std::vector<const Predicate*> del;
  boost::dynamic_bitset<> add_bits;
  boost::dynamic_bitset<> del_bits;
};

struct Action {
  Action(const std::string& name,
         const std::vector<std::string>& arguments,
         Constraint precondition,
         Effect effect)
  : name(utils::make_grounded_name(name, arguments))
  , precondition(std::move(precondition))
  , effect(std::move(effect)) {}

  z3::expr add_at_step(PlanContext& plan_context, unsigned int step) const;

  std::string name;
  Constraint precondition;
  Effect effect;
};

struct Domain {
  std::string name;
  std::vector<Action> actions;
  std::vector<std::shared_ptr<Predicate>> predicates;
};

struct Problem {
  std::string name;
  State initial_state;
  Constraint goal;
  unsigned int min_steps;
  unsigned int max_steps;
  Problem(const std::string& name,
          const State& initial_state,
          const Constraint& goal,
          unsigned int min_steps = 0,
          unsigned int max_steps = 100)
  : name(name)
  , initial_state(initial_state)
  , goal(goal)
  , min_steps{min_steps}
  , max_steps{max_steps} {}
};

}  // namespace planner::symbolic
#endif
