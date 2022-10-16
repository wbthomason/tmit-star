#include "symbolic.hh"

#include <fmt/format.h>
#include <z3++.h>

#include <boost/dynamic_bitset/dynamic_bitset.hpp>
#include <limits>
#include <stdexcept>
#include <vector>

#include "autodiff/forward/real/real.hpp"
#include "boost/dynamic_bitset.hpp"
#include "expression_tree.hh"
#include "plan_context.hh"
#include "predicate.hh"
#include "state.hh"

namespace planner::symbolic {
namespace util {
  z3::expr predicate_var_at_step(PlanContext& plan_context,
                                 const Predicate* predicate,
                                 unsigned int step,
                                 fmt::memory_buffer& name_buf) {
    if (plan_context.predicate_expr_map.size() <= step) {
      plan_context.predicate_expr_map.emplace_back();
    }

    auto& step_vars        = plan_context.predicate_expr_map[step];
    auto& predicate_exprs  = plan_context.predicate_exprs;
    const auto pred_var_it = step_vars.find(predicate->id);
    if (pred_var_it != step_vars.end()) {
      return predicate_exprs[pred_var_it->second];
    }

    fmt::format_to(std::back_inserter(name_buf), "{}@{:d}{}", predicate->name, step, '\0');
    const auto predicate_var = plan_context.ctx.bool_const(name_buf.data());
    step_vars.emplace(predicate->id, predicate_exprs.size());
    predicate_exprs.push_back(predicate_var);
    name_buf.clear();
    return predicate_var;
  }
}  // namespace util

double Predicate::distance_at(LuaEnv& lua_env,
                              const PosedScene<planner::geometric::Transform3<>>& state) const {
  assert(!discrete);
  return call_distance_function(lua_env, state, fn_idx, argspec_idx);
}

double Predicate::gradient_at(LuaEnv& lua_env,
                              const PosedScene<planner::geometric::PoseSetPartialDerivative>& state,
                              Eigen::Ref<Eigen::VectorXd> gradient) const {
  assert(!discrete);
  return call_gradient_function(lua_env, state, gradient, fn_idx, argspec_idx);
}

z3::expr Action::add_at_step(PlanContext& plan_context, unsigned int step) const {
  fmt::memory_buffer var_name;
  fmt::format_to(std::back_inserter(var_name), "{}@{:d}{}", name, step, '\0');
  auto action_at_step_var = plan_context.ctx.bool_const(var_name.data());
  return action_at_step_var;
}

z3::expr Effect::add_at_step(PlanContext& plan_context, unsigned int step) const {
  z3::expr_vector effect_predicates(plan_context.ctx);
  fmt::memory_buffer pred_name;
  for (const auto predicate : add) {
    if (predicate->discrete) {
      effect_predicates.push_back(
      util::predicate_var_at_step(plan_context, predicate, step, pred_name));
    }
  }

  for (const auto predicate : del) {
    if (predicate->discrete) {
      effect_predicates.push_back(
      !util::predicate_var_at_step(plan_context, predicate, step, pred_name));
    }
  }

  return z3::mk_and(effect_predicates);
}

// NOTE: This only applies the *discrete* effect of an action; it does nothing for continuous
// predicates
boost::dynamic_bitset<> Effect::apply(const planner::symbolic::State& mode) const {
  boost::dynamic_bitset<> result(mode);
  // NOTE: Indices of atoms to add should be 1; all other indices must be 0
  result |= add_bits;
  // NOTE: Indices of atoms to delete should be 0; all other indices must be 1
  result &= del_bits;
  return result;
}

boost::dynamic_bitset<> bitset_from_pred_list(const std::vector<const Predicate*>& predicates,
                                              const size_t num_predicates) {
  boost::dynamic_bitset<> result(num_predicates);
  for (const auto predicate : predicates) {
    if (predicate->discrete) {
      result[predicate->mode_idx] = true;
    }
  }

  return result;
}

void gather_predicates(const std::shared_ptr<ExpressionNode>& node,
                       std::vector<const Predicate*>& result) {
  switch (node->type) {
    case ExpressionNode::NodeType::Predicate:
      result.push_back(static_cast<const PredicateNode*>(node.get())->predicate);
      break;
      // TODO handle disjunction
    case ExpressionNode::NodeType::Or:
      throw std::domain_error("Disjunction in formula passed to gather_predicates!");
    default:
      for (const auto& child : static_cast<const AndNode*>(node.get())->operands) {
        gather_predicates(child, result);
      }
      break;
  };
}

std::vector<const Predicate*> gather_predicates(const Formula& formula) {
  std::vector<const Predicate*> result;
  gather_predicates(formula.expression, result);
  return result;
}
}  // namespace planner::symbolic
