#include "task_plan_propagator.hh"

#include <bits/ranges_algo.h>
#include <z3++.h>
#include <z3_api.h>

#include <boost/container_hash/hash.hpp>
#include <boost/logic/tribool.hpp>
#include <limits>
#include <optional>

namespace planner::symbolic {
auto constexpr show_tribool(const boost::tribool& val) {
  if (val == true) {
    return "true";
  }

  if (val == false) {
    return "false";
  }

  return "indeterminate";
}

auto operator<<(std::ostream& out, const boost::tribool& val) -> std::ostream& {
  out << show_tribool(val);
  return out;
}

auto process_constraint(const Constraint& constraint)
-> std::vector<NewTaskPlanPropagator::PredicateTypeT> {
  std::vector<NewTaskPlanPropagator::PredicateTypeT> constraint_predicate_indices;
  const auto constraint_predicates = gather_predicates(constraint.constraint_formula);
  for (const auto* predicate : constraint_predicates) {
    if (predicate->discrete) {
      constraint_predicate_indices.push_back(predicate->mode_idx);
    }
  }

  std::ranges::sort(constraint_predicate_indices);
  return constraint_predicate_indices;
}

auto process_preconditions(const std::vector<Action>& actions)
-> std::vector<std::vector<NewTaskPlanPropagator::PredicateTypeT>> {
  std::vector<std::vector<unsigned int>> preconditions;
  preconditions.reserve(actions.size());
  for (const auto& action : actions) {
    preconditions.emplace_back(process_constraint(action.precondition));
  }

  return preconditions;
}

auto NewTaskPlanPropagator::process_effects(const std::vector<Action>& actions)
-> std::vector<Effect> {
  std::vector<Effect> effects;
  effects.reserve(actions.size());
  for (std::size_t i = 0; i < actions.size(); ++i) {
    const auto& action = actions[i];
    auto& effect       = effects.emplace_back();
    effect.add.reserve(action.effect.add.size());
    for (const auto* predicate : action.effect.add) {
      if (predicate->discrete) {
        effect.add.emplace_back(predicate->mode_idx);
      }
    }

    effect.del.reserve(action.effect.del.size());
    for (const auto* predicate : action.effect.del) {
      if (predicate->discrete) {
        effect.del.emplace_back(predicate->mode_idx);
      }
    }

    std::ranges::sort(effect.add);
    for (const auto idx : effect.add) {
      predicate_adding_actions[idx].emplace_back(i);
    }

    std::ranges::sort(effect.del);
    for (const auto idx : effect.del) {
      predicate_deleting_actions[idx].emplace_back(i);
    }
  }

  return effects;
}

NewTaskPlanPropagator::NewTaskPlanPropagator(z3::solver* solver,
                                             unsigned int state_dim,
                                             const std::vector<Action>& actions,
                                             const Constraint& goal)
: z3::user_propagator_base(solver)
, solver{solver}
, state_dim{state_dim}
, num_actions{static_cast<unsigned int>(actions.size())}
, state_exprs(solver->ctx())
, negated_state_exprs(solver->ctx())
, fixed_exprs(solver->ctx())
, preconditions(process_preconditions(actions))
, effects(process_effects(actions))
, goal(process_constraint(goal)) {
  register_fixed();
  register_final();
}

auto state_var_at_step(unsigned int step,
                       NewTaskPlanPropagator::PredicateTypeT var_idx,
                       unsigned int state_dim) -> unsigned int {
  return step * state_dim + var_idx;
}


void NewTaskPlanPropagator::add_step() {
  state.resize(state.size() + state_dim, boost::indeterminate);
  plan.emplace_back(std::nullopt);
  actions_by_step.emplace_back(solver->ctx());
}

void NewTaskPlanPropagator::add_action(unsigned int step,
                                       unsigned int type_idx,
                                       const z3::expr& expr) {
  add(expr);
  vars.emplace(expr, Variable(expr, type_idx, step, true));
  actions_by_step[step].push_back(expr);
}

void NewTaskPlanPropagator::add_state_var(const Predicate& predicate,
                                          unsigned int step,
                                          const z3::expr& expr) {
  add(expr);
  vars.emplace(expr, Variable(expr, predicate.mode_idx, step, false));
  state_exprs.push_back(expr);
  negated_state_exprs.push_back(!expr);
}


auto NewTaskPlanPropagator::fresh(z3::context& /*ctx*/) -> z3::user_propagator_base* {
  return this;
}

void NewTaskPlanPropagator::pop(unsigned int num_scopes) {
  const auto new_size  = scopes.size() - num_scopes;
  const auto num_fixed = scopes[new_size];
  scopes.resize(new_size);
  for (auto i = fixed_exprs.size(); i > num_fixed; --i) {
    const auto& var = vars.at(fixed_exprs[i - 1]);
    if (var.is_action) {
      plan[var.step] = std::nullopt;
    } else {
      state[state_var_at_step(var.step, var.idx, state_dim)] = boost::indeterminate;
    }
  }

  fixed_exprs.resize(num_fixed);
}

void NewTaskPlanPropagator::push() { scopes.emplace_back(fixed_exprs.size()); }

void NewTaskPlanPropagator::final() {
  const auto unsatisfied_goal_preds = check_goal();
  if (!unsatisfied_goal_preds.empty()) {
    z3::expr_vector unsatisfied_exprs(solver->ctx());
    unsatisfied_exprs.push_back(state_exprs[unsatisfied_goal_preds[0]]);
    for (const auto pred_idx : unsatisfied_goal_preds) {
      unsatisfied_exprs[0] = state_exprs[pred_idx];
      conflict(unsatisfied_exprs);
    }

    return;
  }

  // TODO: Check banned prefixes
}

void NewTaskPlanPropagator::fixed(const z3::expr& var_expr, const z3::expr& value_expr) {
  const auto& var  = vars.at(var_expr);
  const auto value = value_expr.bool_value() == Z3_L_TRUE;
  z3::expr_vector fixed_vars(solver->ctx());
  fixed_vars.push_back(var_expr);
  fixed_exprs.push_back(var_expr);
  if (var.is_action) {
    if (!value) {
      return;
    }

    // Is the step available?
    if (step_filled(var)) {
      fixed_vars.push_back(plan[var.step].value());
      conflict(fixed_vars);
      return;
    }

    // Is the precondition compatible?
    const auto conflicting_preds = precondition_conflicts(var);
    if (!conflicting_preds.empty()) {
      fixed_vars.push_back(state_exprs[conflicting_preds[0]]);
      for (const auto pred_idx : conflicting_preds) {
        fixed_vars[1] = state_exprs[pred_idx];
        conflict(fixed_vars);
      }

      return;
    }

    // TODO: Check for banned prefix

    // Accept the action
    plan[var.step] = var_expr;

    enforce_mutex(var, fixed_vars);
    enforce_precondition(var, fixed_vars);
    enforce_effect(var, fixed_vars);
  } else {
    if (!frame_satisfied(var, value)) {
      if (!try_enforce_frame(var, value, fixed_vars)) {
        return;
      }
    }

    state[state_var_at_step(var.step, var.idx, state_dim)] = value;
  }
}

auto NewTaskPlanPropagator::step_filled(const Variable& action) -> bool {
  return (plan[action.step] != std::nullopt) && (!z3::eq(plan[action.step].value(), action.expr));
}

auto NewTaskPlanPropagator::precondition_conflicts(const Variable& action)
-> std::vector<unsigned int> {
  const auto& precondition = preconditions[action.idx];
  std::vector<unsigned int> conflicting_preds;
  conflicting_preds.reserve(precondition.size());
  for (const auto pred_type_offset : precondition) {
    const auto pred_idx = state_var_at_step(action.step - 1, pred_type_offset, state_dim);
    if (state[pred_idx] == false) {
      conflicting_preds.emplace_back(pred_idx);
    }
  }

  return conflicting_preds;
}

auto NewTaskPlanPropagator::missing_preconditions(const Variable& action)
-> std::vector<unsigned int> {
  const auto& precondition = preconditions[action.idx];
  std::vector<unsigned int> missing_preds;
  missing_preds.reserve(precondition.size());
  for (const auto pred_type_offset : precondition) {
    const auto pred_idx = state_var_at_step(action.step - 1, pred_type_offset, state_dim);
    if (boost::indeterminate(state[pred_idx])) {
      missing_preds.emplace_back(pred_idx);
    }
  }

  return missing_preds;
}

void NewTaskPlanPropagator::enforce_mutex(const Variable& action, const z3::expr_vector& var) {
  for (const auto& step_action : actions_by_step[action.step]) {
    if (!z3::eq(step_action, action.expr)) {
      propagate(var, vars.at(step_action).negated);
    }
  }
}

void NewTaskPlanPropagator::enforce_precondition(const Variable& action,
                                                 const z3::expr_vector& var) {
  for (const auto pred_idx : missing_preconditions(action)) {
    propagate(var, state_exprs[pred_idx]);
  }
}

void NewTaskPlanPropagator::enforce_effect(const Variable& action, const z3::expr_vector& var) {
  const auto& effect = effects[action.idx];
  for (const auto pred_type_offset : effect.add) {
    propagate(var, state_exprs[state_var_at_step(action.step, pred_type_offset, state_dim)]);
  }

  for (const auto pred_type_offset : effect.del) {
    propagate(var,

              negated_state_exprs[state_var_at_step(action.step, pred_type_offset, state_dim)]);
  }
}

auto NewTaskPlanPropagator::appropriate_action_set(
unsigned int step, const std::vector<ActionTypeT>& appropriate_actions) -> bool {
  if (plan[step] == std::nullopt) {
    return false;
  }

  const auto step_action_type = vars.at(plan[step].value()).idx;
  return std::ranges::any_of(appropriate_actions, [step_action_type](const auto action_type) {
    return step_action_type == action_type;
  });
}

auto NewTaskPlanPropagator::added_by_action(const Variable& var) -> bool {
  return appropriate_action_set(var.step, predicate_adding_actions[var.idx]);
}

auto NewTaskPlanPropagator::deleted_by_action(const Variable& var) -> bool {
  return appropriate_action_set(var.step, predicate_deleting_actions[var.idx]);
}

auto NewTaskPlanPropagator::frame_satisfied(const Variable& var, bool value) -> bool {
  if (var.step == 0) {
    return true;
  }

  if ((value && added_by_action(var)) || (!value && deleted_by_action(var))) {
    return true;
  }

  const auto previous_step_val = state[state_var_at_step(var.step - 1, var.idx, state_dim)];
  return static_cast<bool>((value && previous_step_val == true) ||
                           (!value && previous_step_val == false));
}

auto NewTaskPlanPropagator::frame_axiom_expr(const Variable& var, bool value) -> const z3::expr& {
  FrameAxiomKey key{var.expr, value, true};
  const auto& axiom_it = frame_axioms.find(key);
  if (axiom_it != frame_axioms.end()) {
    return axiom_it->second;
  }

  z3::expr_vector frame_axiom_conseqs(solver->ctx());
  const auto prev_step_state_idx = state_var_at_step(var.step - 1, var.idx, state_dim);
  frame_axiom_conseqs.push_back(value ? state_exprs[prev_step_state_idx] :
                                        negated_state_exprs[prev_step_state_idx]);
  const auto& step_actions = actions_by_step[var.step];
  for (const auto action_type :
       value ? predicate_adding_actions[var.idx] : predicate_deleting_actions[var.idx]) {
    frame_axiom_conseqs.push_back(step_actions[action_type]);
  }

  const auto& [new_axiom_it, _] = frame_axioms.emplace(key, z3::mk_or(frame_axiom_conseqs));
  return new_axiom_it->second;
}

auto NewTaskPlanPropagator::try_enforce_frame(const Variable& var,
                                              bool value,
                                              const z3::expr_vector& var_vec) -> bool {
  propagate(var_vec, frame_axiom_expr(var, value));
  return plan[var.step - 1] == std::nullopt;
}

auto NewTaskPlanPropagator::check_goal() -> std::vector<unsigned int> {
  // NOTE: Only valid for positive-only conjunctive goals
  std::vector<PredicateTypeT> unsatisfied_preds;
  unsatisfied_preds.reserve(goal.size());
  const auto last_step = plan.size() - 1;
  for (const auto pred_type_offset : goal) {
    const auto pred_idx = state_var_at_step(last_step, pred_type_offset, state_dim);
    if (!last_set_value(pred_idx)) {
      unsatisfied_preds.emplace_back(pred_idx);
    }
  }

  return unsatisfied_preds;
}

auto NewTaskPlanPropagator::last_set_value(unsigned int idx) -> boost::tribool {
  auto state_value = state[idx];
  while (boost::indeterminate(state_value) && idx >= 0) {
    idx -= state_dim;
    state_value = state[idx];
  }

  return state_value;
}

auto TaskPlanPropagator::get_state_var_at_step(unsigned int step, unsigned int state_var_idx) const
-> unsigned int {
  return step * state_dim + state_var_idx;
}

void TaskPlanPropagator::add_step() {
  plan_states.resize(plan_states.size() + state_dim, boost::indeterminate);
  plan_actions.emplace_back(std::nullopt);
  step_action_exprs.emplace_back(solver->ctx());
  idx_to_expr.resize(idx_to_expr.size() + state_dim, z3::expr(solver->ctx()));
}

void TaskPlanPropagator::pop(unsigned int num_scopes) {
  assert(num_scopes <= scope_stack.size());
  const auto new_size    = scope_stack.size() - num_scopes;
  const auto fixed_count = scope_stack[new_size];
  scope_stack.resize(new_size);
  for (auto i = fixed_exprs.size(); i > fixed_count; --i) {
    const auto& var_info = get_var_info(fixed_exprs[i - 1]);
    if (var_info.is_action) {
      plan_actions[var_info.step] = std::nullopt;
    } else {
      plan_states[get_state_var_at_step(var_info.step, var_info.idx)] = boost::indeterminate;
    }
  }

  fixed_exprs.resize(fixed_count);
}

void TaskPlanPropagator::push() { scope_stack.push_back(fixed_exprs.size()); }

void TaskPlanPropagator::add_action(const Action* const ptr,
                                    unsigned int step,
                                    unsigned int type,
                                    const z3::expr& expr) {
  this->add(expr);
  action_ptr_to_type.emplace(ptr, type);
  expr_info.emplace(expr, VariableInfo(expr, type, step, true));
  step_action_exprs[step].push_back(expr);
}

void TaskPlanPropagator::add_state_var(unsigned int idx, unsigned int step, const z3::expr& expr) {
  this->add(expr);
  expr_info.emplace(expr, VariableInfo(expr, idx, step, false));
  const auto step_idx   = get_state_var_at_step(step, idx);
  idx_to_expr[step_idx] = expr;
}

// void TaskPlanPropagator::block_prefix(const std::vector<const Action*>& action_sequence,
//                                       unsigned int prefix_length) {
//   std::vector<unsigned int> plan_prefix;
//   plan_prefix.reserve(prefix_length);
//   for (std::size_t i = 0; i < prefix_length; ++i) {
//     plan_prefix.emplace_back(action_ptr_to_type[action_sequence[i]]);
//   }
//
//   banned_prefixes.emplace_back(plan_prefix);
//   std::ranges::sort(banned_prefixes, [](const auto& prefix_1, const auto& prefix_2) {
//     return prefix_1.size() < prefix_2.size();
//   });
// }

// bool TaskPlanPropagator::check_banned_prefix() const {
//   prefix_actions.reserve(plan_actions.size());
//   const auto matching_prefix_iter = std::ranges::find_if(banned_prefixes, [&](const auto& prefix)
//   {
//     assert(prefix.size() <= plan_actions.size());
//     prefix_actions.clear();
//     // NOTE: j starts at 1 to skip the first "action", which corresponds to the zeroth step
//     std::size_t i = 0;
//     std::size_t j = 1;
//     while (i < prefix.size() && j < plan_actions.size()) {
//       const auto plan_action_id     = plan_actions[j];
//       const auto prefix_action_type = prefix[i];
//       if (plan_action_id == std::numeric_limits<unsigned int>::max()) {
//         ++j;
//         continue;
//       }
//
//       if (id_to_info[plan_action_id].idx != prefix_action_type) {
//         return false;
//       }
//
//       prefix_actions.push_back(plan_action_id);
//       ++i;
//       ++j;
//     }
//
//     return true;
//   });
//
//   return matching_prefix_iter != banned_prefixes.end();
// }

void TaskPlanPropagator::final() {
  // Did we attain the goal? If not, add a step and return a conflict on the goal variables
  const auto final_state_start = get_state_var_at_step(plan_actions.size() - 1, 0);
  for (const auto goal_predicate_idx : goal) {
    // NOTE: Again, this is only valid so long as the goal constraint contains neither
    // disjunction nor negation, which I'm currently just too lazy to handle here
    const auto state_idx = final_state_start + goal_predicate_idx;
    auto state_value     = plan_states[state_idx];
    if (boost::indeterminate(state_value)) {
      // Trace back to the last time this predicate was set, since action effects should've already
      // applied
      auto step_offset = 2;
      while (plan_actions.size() - step_offset >= 0) {
        auto prev_step_state_idx =
        get_state_var_at_step(plan_actions.size() - step_offset, goal_predicate_idx);
        if (!boost::indeterminate(plan_states[prev_step_state_idx])) {
          state_value = plan_states[prev_step_state_idx];
          break;
        }

        ++step_offset;
      }
    }

    if (!state_value) {
      const auto& predicate_expr = idx_to_expr[state_idx];
      z3::expr_vector conflicts(solver->ctx());
      conflicts.push_back(predicate_expr);
      conflict(conflicts);
    }
  }

  // if (check_banned_prefix()) {
  //   z3::expr_vector prefix_action_exprs(solver->ctx());
  //   for (const auto action_id : prefix_actions) {
  //     prefix_action_exprs.push_back(id_to_info[action_id].expr);
  //   }
  //
  //   propagate(prefix_actions.size(), prefix_actions.data(), !z3::mk_and(prefix_action_exprs));
  // }
}

auto TaskPlanPropagator::added_by_action(unsigned int pred_idx, unsigned int step) const -> bool {
  return action_set_contains(pred_idx, step, predicate_adding_actions);
}

auto TaskPlanPropagator::deleted_by_action(unsigned int pred_idx, unsigned int step) const -> bool {
  return action_set_contains(pred_idx, step, predicate_deleting_actions);
}

auto TaskPlanPropagator::action_set_contains(unsigned int pred_idx,
                                             unsigned int step,
                                             const auto& action_set) const -> bool {
  const auto step_action_id = plan_actions[step];
  return step_action_id != std::nullopt &&
         action_set.at(pred_idx).contains(get_var_info(*step_action_id).idx);
}

void TaskPlanPropagator::get_frame_modifying_actions(const VariableInfo& var_info,
                                                     z3::expr_vector& exprs,
                                                     bool fixed_value) {
  const auto& step_actions = step_action_exprs[var_info.step];
  const auto& frame_modifying_actions =
  fixed_value ? predicate_adding_actions[var_info.idx] : predicate_deleting_actions[var_info.idx];
  for (const auto action_idx : frame_modifying_actions) {
    exprs.push_back(get_var_info(step_actions[action_idx]).expr);
  }
}

auto TaskPlanPropagator::get_frame_axiom_expr(const z3::expr& expr,
                                              bool fixed_value,
                                              bool include_prev_step) -> const z3::expr& {
  FrameAxiomKey key{expr, fixed_value, include_prev_step};
  const auto& expr_it = frame_axiom_exprs.find(key);
  if (expr_it != frame_axiom_exprs.end()) {
    return expr_it->second;
  }

  z3::expr_vector frame_axiom_conseqs(solver->ctx());
  const auto& var_info = get_var_info(expr);
  const auto& prev_step_var_info =
  get_var_info(idx_to_expr[get_state_var_at_step(var_info.step - 1, var_info.idx)]);
  if (include_prev_step) {
    frame_axiom_conseqs.push_back(fixed_value ? prev_step_var_info.expr :
                                                prev_step_var_info.negated_expr);
  }

  get_frame_modifying_actions(var_info, frame_axiom_conseqs, fixed_value);
  const auto& [new_expr_it, _] = frame_axiom_exprs.emplace(key, z3::mk_or(frame_axiom_conseqs));
  return new_expr_it->second;
}

void TaskPlanPropagator::fixed(const z3::expr& var, const z3::expr& value_expr) {
  // Translate the ID to variable info
  const auto& var_info = get_var_info(var);
  // NOTE: I assume that we only ever get true or false here, not UNDEF?
  const auto fixed_value = value_expr.bool_value() == Z3_L_TRUE;
  z3::expr_vector fixed_var(solver->ctx());
  fixed_var.push_back(var);
  fixed_exprs.push_back(var);
  // Is the fixed variable an action or a state variable?
  if (var_info.is_action) {
    if (!fixed_value) {
      return;
    }

    // Did we choose or reject the action?
    // If we chose the action:
    // Check that no action is currently set; if one is, report a conflict. If not, propagate the
    // consequence that no other actions at the current step are true
    const auto current_step_action = plan_actions[var_info.step];
    if (current_step_action != std::nullopt && current_step_action != var) {
      z3::expr_vector conflicting_vars(solver->ctx());
      conflicting_vars.push_back(var);
      conflicting_vars.push_back(*current_step_action);
      conflict(conflicting_vars);
      return;
    }

    // Check if the precondition is compatible with the current set of fixed state variables. If
    // not, report conflicts with the incompatible settings (or maybe just the action variable?)
    const auto& precondition   = action_preconditions[var_info.idx];
    bool precondition_conflict = false;
    std::vector<unsigned int> unset_predicates;
    const auto prev_step_state_idx = get_state_var_at_step(var_info.step - 1, 0);
    for (const auto predicate_idx : precondition) {
      const auto state_idx       = prev_step_state_idx + predicate_idx;
      const auto predicate_value = plan_states[state_idx];
      // NOTE: Dependent on "positive atoms only" precondition form, without disjunction
      if (predicate_value == false) {
        precondition_conflict = true;
        z3::expr_vector conflicting_vars(solver->ctx());
        conflicting_vars.push_back(var);
        conflicting_vars.push_back(idx_to_expr[state_idx]);
        conflict(conflicting_vars);
      } else if (!precondition_conflict && boost::indeterminate(predicate_value)) {
        unset_predicates.push_back(predicate_idx);
      }
    }

    if (precondition_conflict) {
      return;
    }

    // Does the proposed plan use a banned prefix? If so, return a conflict on that prefix
    // NOTE: This is hard-broken right now because it implies that none of those actions together
    // are valid, when really we just want to say that the specific sequence of actions is invalid
    // Instead, we're blocking prefixes as explicit constraints in task_plan.cc

    plan_actions[var_info.step] = var;

    // Block all other actions at this step
    for (const auto other_expr : step_action_exprs[var_info.step]) {
      if (other_expr != var) {
        propagate(fixed_var, get_var_info(other_expr).negated_expr);
      }
    }


    // Propagate any unset portions of the precondition, and propagate the effect of the action
    for (const auto idx : unset_predicates) {
      const auto precondition_var_info = get_var_info(idx_to_expr[prev_step_state_idx + idx]);
      propagate(fixed_var, get_var_info(idx_to_expr[prev_step_state_idx + idx]).expr);
    }

    const auto& effect             = action_effects[var_info.idx];
    const auto this_step_state_idx = get_state_var_at_step(var_info.step, 0);
    for (const auto idx : effect.positive_predicates) {
      const auto effect_var_info = get_var_info(idx_to_expr[this_step_state_idx + idx]);
      propagate(fixed_var, get_var_info(idx_to_expr[this_step_state_idx + idx]).expr);
    }

    for (const auto idx : effect.negative_predicates) {
      const auto effect_var_info = get_var_info(idx_to_expr[this_step_state_idx + idx]);
      propagate(fixed_var, get_var_info(idx_to_expr[this_step_state_idx + idx]).negated_expr);
    }
  } else {
    // If it's a state variable:
    // Check that the setting is valid by determining if the frame axioms are maintained
    const auto pred_idx = get_state_var_at_step(var_info.step, var_info.idx);
    if (var_info.step > 0) {
      const auto prev_step_state_idx = get_state_var_at_step(var_info.step - 1, var_info.idx);
      // const unsigned both_step_ids[]{id, idx_to_id.at(prev_step_state_idx)};
      // Either the value is the same as the last step, or an action that causes it in this step was
      // taken at the last step
      const auto prev_step_state_value = plan_states[prev_step_state_idx];
      const auto& prev_step_state_info = get_var_info(idx_to_expr[prev_step_state_idx]);
      const auto correctly_added   = fixed_value && added_by_action(var_info.idx, var_info.step);
      const auto correctly_deleted = !fixed_value && deleted_by_action(var_info.idx, var_info.step);
      const auto step_action_expr  = plan_actions[var_info.step];
      const auto action_is_set     = step_action_expr != std::nullopt;
      if (boost::indeterminate(prev_step_state_value)) {
        if (!correctly_added && !correctly_deleted) {
          if (!action_is_set) {
            // Propagate the appropriate adding/deleting actions, as well as the previous step being
            // set to the right value in an even earlier step
            const auto& frame_axiom_expr = get_frame_axiom_expr(var, fixed_value, true);
            propagate(fixed_var, frame_axiom_expr);
          } else {
            // The only way for this to work is for the previous step to be set to the right value
            // in an even earlier step
            z3::expr_vector necessary_vars(solver->ctx());
            necessary_vars.push_back(var);
            necessary_vars.push_back(*step_action_expr);
            propagate(necessary_vars,
                      fixed_value ? prev_step_state_info.expr : prev_step_state_info.negated_expr);
          }
        }
      } else if (prev_step_state_value != fixed_value) {
        if (!correctly_added && !correctly_deleted) {
          if (!action_is_set) {
            // Propagate the appropriate adding/deleting actions
            z3::expr_vector necessary_vars(solver->ctx());
            necessary_vars.push_back(var);
            necessary_vars.push_back(idx_to_expr[prev_step_state_idx]);
            const auto& frame_axiom_expr = get_frame_axiom_expr(var, fixed_value, false);
            propagate(necessary_vars, frame_axiom_expr);
          } else {
            // We have a conflict between the previous state, current state, and set action
            z3::expr_vector conflicting_vars(solver->ctx());
            conflicting_vars.push_back(var);
            conflicting_vars.push_back(idx_to_expr[prev_step_state_idx]);
            conflicting_vars.push_back(*step_action_expr);
            conflict(conflicting_vars);
            return;
          }
        }
      }
    }

    plan_states[pred_idx] = fixed_value;
  }
}

auto TaskPlanPropagator::fresh(z3::context& /*ctx*/) -> z3::user_propagator_base* { return this; }

auto TaskPlanPropagator::get_var_info(const z3::expr& var) const
-> const TaskPlanPropagator::VariableInfo& {
  return expr_info.at(var);
}

auto FrameAxiomKey::operator==(const FrameAxiomKey& other) const -> bool {
  return expr.id() == other.expr.id() && value == other.value && prev_step == other.prev_step;
}
}  // namespace planner::symbolic

auto std::hash<planner::symbolic::FrameAxiomKey>::operator()(
const planner::symbolic::FrameAxiomKey& key) const noexcept -> std::size_t {
  std::size_t seed{0};
  boost::hash_combine(seed, key.expr.id());
  boost::hash_combine(seed, key.value);
  boost::hash_combine(seed, key.prev_step);
  return seed;
}
