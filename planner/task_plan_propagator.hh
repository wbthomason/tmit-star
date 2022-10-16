#ifndef TASK_PLAN_PROPAGATOR_HH
#define TASK_PLAN_PROPAGATOR_HH

#include <z3++.h>

#include <algorithm>
#include <boost/logic/tribool.hpp>
#include <vector>

#include "predicate.hh"
#include "robin_hood.h"
#include "symbolic.hh"

template <> struct std::hash<z3::expr> {
  std::size_t operator()(const z3::expr& k) const { return k.id(); }
};

// Do not use Z3's == operator in the hash table
template <> struct std::equal_to<z3::expr> {
  bool operator()(const z3::expr& lhs, const z3::expr& rhs) const { return z3::eq(lhs, rhs); }
};

namespace planner::symbolic {
struct FrameAxiomKey {
  z3::expr expr;
  bool value;
  bool prev_step;
  bool operator==(const FrameAxiomKey& o) const;
};
}  // namespace planner::symbolic

template <> struct std::hash<planner::symbolic::FrameAxiomKey> {
  std::size_t operator()(const planner::symbolic::FrameAxiomKey& k) const noexcept;
};

namespace planner::symbolic {

struct NewTaskPlanPropagator : public z3::user_propagator_base {
  using PredicateTypeT = unsigned int;
  using ActionTypeT    = unsigned int;
  struct Variable {
    Variable(const z3::expr& expr, unsigned int idx, unsigned int step, bool is_action)
    : expr{expr}, negated{!expr}, idx{idx}, step{step}, is_action{is_action} {}
    z3::expr expr;
    z3::expr negated;
    unsigned int idx;
    unsigned int step;
    bool is_action;
  };

  struct Effect {
    std::vector<PredicateTypeT> add;
    std::vector<PredicateTypeT> del;
  };

  NewTaskPlanPropagator(z3::solver* solver,
                        unsigned int state_dim,
                        const std::vector<Action>& actions,
                        const Constraint& goal);

  auto fresh(z3::context& ctx) -> z3::user_propagator_base* override;
  void final() override;
  void fixed(const z3::expr& var_expr, const z3::expr& value_expr) override;
  void pop(unsigned int num_scopes) override;
  void push() override;

  void add_step();
  void add_action(unsigned int step, unsigned int type_idx, const z3::expr& expr);
  void add_state_var(const Predicate& predicate, unsigned int step, const z3::expr& expr);

 private:
  auto process_effects(const std::vector<Action>& actions) -> std::vector<Effect>;

  auto step_filled(const Variable& action) -> bool;
  auto precondition_conflicts(const Variable& action) -> std::vector<unsigned int>;
  auto missing_preconditions(const Variable& action) -> std::vector<unsigned int>;
  void enforce_mutex(const Variable& action, const z3::expr_vector& var);
  void enforce_precondition(const Variable& action, const z3::expr_vector& var);
  void enforce_effect(const Variable& action, const z3::expr_vector& var);

  auto frame_satisfied(const Variable& var, bool value) -> bool;
  auto try_enforce_frame(const Variable& var, bool value, const z3::expr_vector& var_vec) -> bool;
  auto appropriate_action_set(unsigned int step,
                              const std::vector<ActionTypeT>& appropriate_actions) -> bool;
  auto added_by_action(const Variable& var) -> bool;
  auto deleted_by_action(const Variable& var) -> bool;
  auto frame_axiom_expr(const Variable& var, bool value) -> const z3::expr&;

  auto check_goal() -> std::vector<unsigned int>;
  auto last_set_value(unsigned int idx) -> boost::tribool;

  z3::solver* solver;
  const unsigned int state_dim;
  const unsigned int num_actions;

  robin_hood::unordered_map<z3::expr, Variable> vars;
  z3::expr_vector state_exprs;
  z3::expr_vector negated_state_exprs;
  std::vector<z3::expr_vector> actions_by_step;
  robin_hood::unordered_map<FrameAxiomKey, z3::expr> frame_axioms;

  std::vector<boost::tribool> state;
  std::vector<std::optional<z3::expr>> plan;

  std::vector<std::size_t> scopes;
  z3::expr_vector fixed_exprs;

  robin_hood::unordered_map<PredicateTypeT, std::vector<ActionTypeT>> predicate_adding_actions;
  robin_hood::unordered_map<PredicateTypeT, std::vector<ActionTypeT>> predicate_deleting_actions;
  // NOTE: This precondition model is only valid for postive-only conjunctive preconditions
  const std::vector<std::vector<PredicateTypeT>> preconditions;
  const std::vector<Effect> effects;
  // NOTE: This goal model is only valid for positive-only conjunctive goals
  const std::vector<PredicateTypeT> goal;
};

inline std::ostream& operator<<(std::ostream& out, const NewTaskPlanPropagator::Variable& v) {
  out << "{expr: " << v.expr << ", idx: " << v.idx << ", step: " << v.step
      << ", is_action:" << v.is_action << "}";
  return out;
}

struct TaskPlanPropagator : public z3::user_propagator_base {
  z3::solver* solver;
  const unsigned int state_dim;
  const unsigned int num_actions;

  struct VariableInfo {
    VariableInfo(const z3::expr& expr, unsigned int idx, unsigned int step, bool is_action)
    : idx(idx), step(step), is_action(is_action), expr(expr), negated_expr(!expr) {}
    unsigned int idx;
    unsigned int step;
    bool is_action;
    z3::expr expr;
    z3::expr negated_expr;
  };

  robin_hood::unordered_map<z3::expr, VariableInfo> expr_info;
  std::vector<z3::expr> idx_to_expr;
  robin_hood::unordered_map<const Action*, unsigned int> action_ptr_to_type;
  robin_hood::unordered_map<FrameAxiomKey, z3::expr> frame_axiom_exprs;
  std::vector<z3::expr_vector> step_action_exprs;
  robin_hood::unordered_map<unsigned int, robin_hood::unordered_set<unsigned int>>
  predicate_adding_actions;
  robin_hood::unordered_map<unsigned int, robin_hood::unordered_set<unsigned int>>
  predicate_deleting_actions;
  const std::vector<std::vector<unsigned int>> action_preconditions;

  struct SymbolicEffect {
    std::vector<unsigned int> positive_predicates;
    std::vector<unsigned int> negative_predicates;
  };

  const std::vector<SymbolicEffect> action_effects;
  const std::vector<unsigned int> goal;

  std::vector<z3::expr_vector> banned_prefixes;
  std::vector<std::size_t> scope_stack;
  z3::expr_vector fixed_exprs;
  std::vector<boost::tribool> plan_states;
  std::vector<std::optional<z3::expr>> plan_actions;
  mutable std::vector<z3::expr> prefix_actions;

  auto process_action_effects(const std::vector<Action>& actions);
  auto get_var_info(const z3::expr& var) const -> const VariableInfo&;
  auto get_state_var_at_step(unsigned int step, unsigned int state_var_idx) const -> unsigned int;

  auto added_by_action(unsigned int pred_idx, unsigned int step) const -> bool;
  auto deleted_by_action(unsigned int pred_idx, unsigned int step) const -> bool;
  auto action_set_contains(unsigned int pred_idx, unsigned int step, const auto& action_set) const
  -> bool;
  void get_frame_modifying_actions(const VariableInfo& var_info,
                                   z3::expr_vector& exprs,
                                   bool fixed_value);
  auto get_frame_axiom_expr(const z3::expr& expr, bool fixed_value, bool include_prev_step)
  -> const z3::expr&;
  void add_step();
  TaskPlanPropagator(z3::solver* solver,
                     unsigned int state_dim,
                     const std::vector<Action>& actions,
                     const Constraint& goal);
  auto fresh(z3::context& ctx) -> z3::user_propagator_base* override;
  void push() override;
  void pop(unsigned int num_scopes) override;
  void add_action(const Action* ptr, unsigned int step, unsigned int type, const z3::expr& expr);
  void add_state_var(unsigned int idx, unsigned int step, const z3::expr& expr);
  void block_prefix(const std::vector<const Action*>& action_sequence, unsigned int prefix_length);
  auto check_banned_prefix() const -> bool;
  void final() override;
  void fixed(const z3::expr& var, const z3::expr& value_expr) override;
};

}  // namespace planner::symbolic

#endif
