#ifndef EXPRESSION_TREE_HH
#define EXPRESSION_TREE_HH

#include <fmt/format.h>
#include <z3++.h>

#include <Eigen/Core>
#include <iostream>
#include <memory>
#include <vector>

#include "autodiff/forward/real.hpp"
#include "autodiff/forward/real/eigen.hpp"
#include "lua_env.hh"
#include "plan_context_fwd.hh"
#include "predicate.hh"
#include "robin_hood.h"
#include "scenegraph.hh"
#include "state.hh"

namespace planner::symbolic {
template <typename ExpressionT>
double
distance_at(const ExpressionT* const expression, LuaEnv& lua_env, const PosedScene<planner::geometric::Transform3<>>& state, double default_result);

template <typename ExpressionT>
double gradient_at(const ExpressionT* const expression,
                   LuaEnv& lua_env,
                   const PosedScene<planner::geometric::PoseSetPartialDerivative>& state,
                   Eigen::Ref<Eigen::VectorXd> gradient,
                   double default_result);

template <typename ExpressionT>
z3::expr
add_expression_at_step(const ExpressionT* const expression, PlanContext& plan_context, unsigned int step, fmt::memory_buffer* const name_buf);
template <typename ExpressionT> void print(const ExpressionT* const expression);

struct ExpressionNode {
  ExpressionNode() = delete;

  template <typename Derived>
  friend double
  distance_at(const Derived* const expression, LuaEnv& lua_env, const PosedScene<planner::geometric::Transform3<>>& state, double default_result);
  double distance_at(LuaEnv& lua_env, const PosedScene<planner::geometric::Transform3<>>& state, double default_result = 0.0) const;

  template <typename Derived>
  friend double gradient_at(const Derived* const expression,
                            LuaEnv& lua_env,
                            const PosedScene<planner::geometric::PoseSetPartialDerivative>& state,
                            Eigen::Ref<Eigen::VectorXd> gradient,
                            double default_result);

  double gradient_at(LuaEnv& lua_env,
                     const PosedScene<planner::geometric::PoseSetPartialDerivative>& state,
                     Eigen::Ref<Eigen::VectorXd> gradient,
                     double default_result = 0.0) const;

  template <typename Derived>
  friend z3::expr
  add_expression_at_step(const Derived* const expression, PlanContext& plan_context, unsigned int step, fmt::memory_buffer* const name_buf);

  z3::expr add_at_step(PlanContext& plan_context, unsigned int step, fmt::memory_buffer* const name_buf) const;

  friend void gather_predicates(const std::shared_ptr<ExpressionNode>& node, std::vector<const Predicate*>& result);
  friend void print(const ExpressionNode* node);
  void print() const;

 protected:
  // TODO: Add support for Not
  enum struct NodeType { And, Or, Predicate };
  NodeType type;
  explicit ExpressionNode(NodeType type) : type(type) {}
};

struct OperatorNode : public ExpressionNode {
  OperatorNode() = delete;
  z3::expr operation_expr(PlanContext& plan_context, unsigned int step, fmt::memory_buffer* const name_buf) const;

  std::vector<std::shared_ptr<ExpressionNode>> operands;

 protected:
  OperatorNode(NodeType type, z3::expr (*op)(const z3::expr_vector&)) : ExpressionNode(type), op(op) {}
  z3::expr (*op)(const z3::expr_vector&);
};

struct AndNode : public OperatorNode {
  AndNode() : OperatorNode(NodeType::And, z3::mk_and) {}
};

template <>
double
distance_at(const AndNode* const expression, LuaEnv& lua_env, const PosedScene<planner::geometric::Transform3<>>& state, double default_result);
template <>
double gradient_at(const AndNode* const expression,
                   LuaEnv& lua_env,
                   const PosedScene<planner::geometric::PoseSetPartialDerivative>& state,
                   Eigen::Ref<Eigen::VectorXd> gradient,
                   double default_result);

template <>
z3::expr add_expression_at_step(const AndNode* const expression, PlanContext& plan_context, unsigned int step, fmt::memory_buffer* const name_buf);
template <> void print(const AndNode* const expression);

struct OrNode : public OperatorNode {
  OrNode() : OperatorNode(NodeType::Or, z3::mk_or) {}
};

template <>
double distance_at(const OrNode* const expression, LuaEnv& lua_env, const PosedScene<planner::geometric::Transform3<>>& state, double default_result);
template <>
double gradient_at(const OrNode* const expression,
                   LuaEnv& lua_env,
                   const PosedScene<planner::geometric::PoseSetPartialDerivative>& state,
                   Eigen::Ref<Eigen::VectorXd> gradient,
                   double default_result);

template <>
z3::expr add_expression_at_step(const OrNode* const expression, PlanContext& plan_context, unsigned int step, fmt::memory_buffer* const name_buf);

template <> void print(const OrNode* const expression);

struct PredicateNode : public ExpressionNode {
  PredicateNode() : ExpressionNode(NodeType::Predicate), predicate(nullptr) {}

  const Predicate* predicate;
};

template <>
double
distance_at(const PredicateNode* const expression, LuaEnv& lua_env, const PosedScene<planner::geometric::Transform3<>>& state, double default_result);
template <>
double gradient_at(const PredicateNode* const expression,
                   LuaEnv& lua_env,
                   const PosedScene<planner::geometric::PoseSetPartialDerivative>& state,
                   Eigen::Ref<Eigen::VectorXd> gradient,
                   double default_result);

template <>
z3::expr
add_expression_at_step(const PredicateNode* const expression, PlanContext& plan_context, unsigned int step, fmt::memory_buffer* const name_buf);

template <> void print(const PredicateNode* const expression);
}  // namespace planner::symbolic
#endif
