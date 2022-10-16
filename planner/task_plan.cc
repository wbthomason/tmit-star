#include "task_plan.hh"

#include <fmt/format.h>
#include <z3++.h>

#include <chrono>
// #include <iostream>
#include <optional>
#include <stdexcept>
#include <vector>

#include "plan_context.hh"
#include "predicate.hh"
#include "symbolic.hh"
#include "task_plan_propagator.hh"

namespace planner {
namespace symbolic {
  PlannerState::PlannerState(const Domain& domain, const Problem& problem)
  : predicates{domain.predicates}
  , actions{domain.actions}
  , goal{problem.goal}
  , steps{0}
  , ctx(problem.min_steps, problem.max_steps)
  , solver{ctx.ctx, Z3_mk_simple_solver(ctx.ctx)}
  , propagator(&solver, problem.initial_state.size(), actions, goal)
  , action_coeffs(domain.actions.size(), 1) {
    // Add the initial state to the solver
    ctx.action_expr_map.emplace_back();
    propagator.add_step();
    fmt::memory_buffer var_name_buf;
    for (const auto& pred : predicates) {
      if (pred->discrete) {
        const auto pred_var = util::predicate_var_at_step(ctx, pred.get(), 0, var_name_buf);
        propagator.add_state_var(*pred, 0, pred_var);
        if (problem.initial_state[pred->mode_idx]) {
          solver.add(pred_var == ctx.ctx.bool_val(true));
        } else {
          solver.add(pred_var == ctx.ctx.bool_val(false));
        }
      }
    }

    solver.push();
  }
}  // namespace symbolic

// TODO: It might make sense to take in an expression for the motion-induced constraints, to pass
// to check() as assumptions rather than adding as assertions
std::optional<std::vector<const symbolic::Action*>> plan(symbolic::PlannerState& state) {
  const auto& goal          = state.goal;
  const auto& actions       = state.actions;
  const auto& predicates    = state.predicates;
  const auto num_predicates = predicates.size();
  auto& steps               = state.steps;
  auto& ctx                 = state.ctx;
  auto& action_vars         = ctx.action_exprs;
  auto& action_var_map      = ctx.action_expr_map;
  auto& solver              = state.solver;
  auto& propagator          = state.propagator;
  fmt::memory_buffer var_name_buf;
  // Try to solve the current iteration
  while (steps < ctx.min_steps || (steps < ctx.max_steps && solver.check() == z3::unsat)) {
    // Remove the goal
    solver.pop();
    // Add a new step:
    ++steps;
    propagator.add_step();
    // Add a new step: start frame axioms
    for (std::size_t i = 0; i < num_predicates; ++i) {
      if (predicates[i]->discrete) {
        propagator.add_state_var(
        *predicates[i],
        steps,
        symbolic::util::predicate_var_at_step(ctx, predicates[i].get(), steps, var_name_buf));
      }
    }

    // Add a new step: add actions
    auto& step_actions = action_var_map.emplace_back();
    step_actions.reserve(actions.size());
    z3::expr_vector step_action_exprs(ctx.ctx);
    for (std::size_t i = 0; i < actions.size(); ++i) {
      const auto& action     = actions[i];
      const auto& action_var = action.add_at_step(ctx, steps);
      step_actions.insert(action_vars.size(), &action);
      action_vars.push_back(action_var);
      step_action_exprs.push_back(action_var);
      propagator.add_action(steps, i, action_var);
    }

    solver.add(z3::mk_or(step_action_exprs));
    solver.push();

    // Add the goal back at the end
    solver.add(goal.add_at_step(ctx, steps));
  }

  if (steps == ctx.max_steps) {
    return std::nullopt;
  }

  // We've found a plan! Extract it
  const auto solution = solver.get_model();
  std::vector<const symbolic::Action*> result;
  result.reserve(steps);
  for (const auto& step_actions : ctx.action_expr_map) {
    for (const auto& [action_var_id, action_ptr] : step_actions) {
      const auto& action_var = action_vars[action_var_id];
      if (solution.eval(action_var).is_true()) {
        result.push_back(action_ptr);
        break;
      }
    }
  }

  return result;
}

void forbid_plan_prefix(symbolic::PlannerState& state,
                        const std::vector<const symbolic::Action*>& actions,
                        unsigned int prefix_length) {
  z3::expr_vector action_exprs(state.ctx.ctx);
  for (unsigned int i = 0; i < prefix_length; ++i) {
    action_exprs.push_back(state.ctx.action_exprs[state.ctx.action_expr_map[i + 1][actions[i]]]);
  }

  state.solver.pop();
  state.solver.add(!z3::mk_and(action_exprs));
  state.solver.push();
}
}  // namespace planner
