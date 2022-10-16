#include "expression_tree.hh"

#include <fmt/format.h>
#include <z3++.h>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

#include "autodiff/forward/real.hpp"
#include "plan_context.hh"
#include "predicate.hh"
#include "robin_hood.h"
#include "state.hh"

namespace planner::symbolic {
z3::expr OperatorNode::operation_expr(PlanContext& plan_context,
                                      unsigned int step,
                                      fmt::memory_buffer* const name_buf) const {
  z3::expr_vector sub_exprs(plan_context.ctx);
  for (const auto& operand : operands) {
    sub_exprs.push_back(operand->add_at_step(plan_context, step, name_buf));
  }

  return op(sub_exprs);
}

constexpr double AND_DEFAULT_VALUE = -std::numeric_limits<double>::infinity();
template <>
double distance_at(const AndNode* const expression,
                   LuaEnv& lua_env,
                   const PosedScene<geometric::Transform3<>>& state,
                   double default_result) {
  const auto& operands = expression->operands;
  auto distance        = operands[0]->distance_at(lua_env, state, AND_DEFAULT_VALUE);
  for (size_t i = 1; i < operands.size(); ++i) {
    distance = std::max(distance, operands[i]->distance_at(lua_env, state, AND_DEFAULT_VALUE));
  }

  return distance;
}

template <>
double gradient_at(const AndNode* const expression,
                   LuaEnv& lua_env,
                   const PosedScene<geometric::PoseSetPartialDerivative>& state,
                   Eigen::Ref<Eigen::VectorXd> gradient,
                   double default_result) {
  const auto& operands = expression->operands;
  Eigen::VectorXd local_gradient(gradient.size());
  auto result = operands[0]->gradient_at(lua_env, state, gradient, AND_DEFAULT_VALUE);
  for (size_t i = 1; i < operands.size(); ++i) {
    local_gradient.setZero();
    const auto local_result =
    operands[i]->gradient_at(lua_env, state, local_gradient, AND_DEFAULT_VALUE);
    // To match how autodiff implements max()
    if (local_result > result) {
      result   = local_result;
      gradient = local_gradient;
    }
  }

  return result;
}

template <>
z3::expr add_expression_at_step(const AndNode* const expression,
                                PlanContext& plan_context,
                                unsigned int step,
                                fmt::memory_buffer* const name_buf) {
  return expression->operation_expr(plan_context, step, name_buf);
}

template <> void print(const AndNode* const expression) {
  std::cout << "And with " << expression->operands.size() << " operands:" << std::endl;
  for (const auto& operand : expression->operands) {
    operand->print();
  }

  std::cout << std::endl;
}

constexpr double OR_DEFAULT_VALUE = std::numeric_limits<double>::infinity();
template <>
double distance_at(const OrNode* const expression,
                   LuaEnv& lua_env,
                   const PosedScene<geometric::Transform3<>>& state,
                   double default_result) {
  const auto& operands = expression->operands;
  auto distance        = operands[0]->distance_at(lua_env, state, OR_DEFAULT_VALUE);
  for (size_t i = 1; i < operands.size(); ++i) {
    distance = std::min(distance, operands[i]->distance_at(lua_env, state, OR_DEFAULT_VALUE));
  }

  return distance;
}

template <>
double gradient_at(const OrNode* const expression,
                   LuaEnv& lua_env,
                   const PosedScene<geometric::PoseSetPartialDerivative>& state,
                   Eigen::Ref<Eigen::VectorXd> gradient,
                   double default_result) {
  const auto& operands = expression->operands;
  Eigen::VectorXd local_gradient(gradient.size());
  double result = std::numeric_limits<double>::max();
  for (const auto& operand : operands) {
    local_gradient.setZero();
    // To match how autodiff implements min()
    const auto local_result =
    operand->gradient_at(lua_env, state, local_gradient, OR_DEFAULT_VALUE);
    if (local_result < result) {
      result   = local_result;
      gradient = local_gradient;
    }
  }

  return result;
}

template <>
z3::expr add_expression_at_step(const OrNode* const expression,
                                PlanContext& plan_context,
                                unsigned int step,
                                fmt::memory_buffer* const name_buf) {
  return expression->operation_expr(plan_context, step, name_buf);
}

template <> void print(const OrNode* const expression) {
  std::cout << "Or with " << expression->operands.size() << "operands:" << std::endl;
  for (const auto& operand : expression->operands) {
    operand->print();
  }

  std::cout << std::endl;
}

template <>
double distance_at(const PredicateNode* const expression,
                   LuaEnv& lua_env,
                   const PosedScene<geometric::Transform3<>>& state,
                   double default_result) {
  if (expression->predicate->discrete) {
    // NOTE: This assumes that we've already ensured that we're in a compatible mode, i.e. that the
    // discrete bits are all satisfied. We'd need to take in a planner::State instead here and
    // check against its mode otherwise
    return default_result;
  };

  return expression->predicate->distance_at(lua_env, state);
}

template <>
double gradient_at(const PredicateNode* const expression,
                   LuaEnv& lua_env,
                   const PosedScene<geometric::PoseSetPartialDerivative>& state,
                   Eigen::Ref<Eigen::VectorXd> gradient,
                   double default_result) {
  if (expression->predicate->discrete) {
    // NOTE: This assumes that we've already ensured that we're in a compatible mode, i.e. that the
    // discrete bits are all satisfied. We'd need to take in a planner::State instead here and
    // check against its mode otherwise
    gradient.setZero();
    return default_result;
  };

  return expression->predicate->gradient_at(lua_env, state, gradient);
}

template <>
z3::expr add_expression_at_step(const PredicateNode* const expression,
                                PlanContext& plan_context,
                                unsigned int step,
                                fmt::memory_buffer* const name_buf) {
  if (expression->predicate->discrete) {
    return util::predicate_var_at_step(plan_context, expression->predicate, step, *name_buf);
  }

  return plan_context.ctx.bool_val(true);
}

template <> void print(const PredicateNode* const expression) {
  std::cout << "\tPredicate: " << expression->predicate->name;
}

double ExpressionNode::distance_at(LuaEnv& lua_env,
                                   const PosedScene<geometric::Transform3<>>& state,
                                   double default_result) const {
  switch (type) {
    case NodeType::And:
      return symbolic::distance_at(
      static_cast<const AndNode*>(this), lua_env, state, default_result);
    case NodeType::Or:
      return symbolic::distance_at(static_cast<const OrNode*>(this), lua_env, state, default_result);
    case NodeType::Predicate:
      return symbolic::distance_at(
      static_cast<const PredicateNode*>(this), lua_env, state, default_result);
  }
}

double ExpressionNode::gradient_at(LuaEnv& lua_env,
                                   const PosedScene<geometric::PoseSetPartialDerivative>& state,
                                   Eigen::Ref<Eigen::VectorXd> gradient,
                                   double default_result) const {
  switch (type) {
    case NodeType::And:
      return symbolic::gradient_at(
      static_cast<const AndNode*>(this), lua_env, state, gradient, default_result);
    case NodeType::Or:
      return symbolic::gradient_at(
      static_cast<const OrNode*>(this), lua_env, state, gradient, default_result);
    case NodeType::Predicate:
      return symbolic::gradient_at(
      static_cast<const PredicateNode*>(this), lua_env, state, gradient, default_result);
  }
}

z3::expr ExpressionNode::add_at_step(PlanContext& plan_context,
                                     unsigned int step,
                                     fmt::memory_buffer* const name_buf) const {
  switch (type) {
    case NodeType::And:
      return add_expression_at_step(static_cast<const AndNode*>(this), plan_context, step, name_buf);
    case NodeType::Or:
      return add_expression_at_step(static_cast<const OrNode*>(this), plan_context, step, name_buf);
    case NodeType::Predicate:
      return add_expression_at_step(
      static_cast<const PredicateNode*>(this), plan_context, step, name_buf);
  }
}

void ExpressionNode::print() const {
  switch (type) {
    case NodeType::And:
      return symbolic::print(static_cast<const AndNode*>(this));
    case NodeType::Or:
      return symbolic::print(static_cast<const OrNode*>(this));
    case NodeType::Predicate:
      return symbolic::print(static_cast<const PredicateNode*>(this));
  }
}
}  // namespace planner::symbolic
