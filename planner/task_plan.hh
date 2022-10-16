#ifndef TASK_PLAN_HH
#define TASK_PLAN_HH
#include <z3++.h>

#include <memory>
#include <optional>
#include <vector>

#include "plan_context.hh"
#include "predicate.hh"
#include "symbolic.hh"
#include "task_plan_propagator.hh"

namespace planner {
namespace symbolic {
  struct PlannerState {
    PlannerState(const Domain& domain, const Problem& problem);
    const std::vector<std::shared_ptr<Predicate>>& predicates;
    const std::vector<Action>& actions;
    const Constraint& goal;
    unsigned int steps;
    PlanContext ctx;
    z3::solver solver;
    NewTaskPlanPropagator propagator;
    const std::vector<int> action_coeffs;
  };
}  // namespace symbolic

std::optional<std::vector<const symbolic::Action*>> plan(symbolic::PlannerState& state);
void forbid_plan_prefix(symbolic::PlannerState& state,
                        const std::vector<const symbolic::Action*>& actions,
                        unsigned int prefix_length);
}  // namespace planner
#endif
