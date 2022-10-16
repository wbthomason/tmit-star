#ifndef PLAN_CONTEXT_HH
#define PLAN_CONTEXT_HH
#include <fmt/format.h>
#include <z3++.h>

#include <vector>

#include "bidirectional_map.hh"
#include "predicate.hh"
#include "robin_hood.h"
#include "symbolic.hh"

namespace planner::symbolic {
struct PlanContext {
  PlanContext(unsigned int min_steps = 0, unsigned int max_steps = 100)
  : ctx(), predicate_exprs(ctx), action_exprs(ctx), min_steps{min_steps}, max_steps{max_steps} {}

  z3::context ctx;
  std::vector<robin_hood::unordered_map<unsigned int, unsigned int>> predicate_expr_map;
  z3::expr_vector predicate_exprs;
  std::vector<utils::BidirectionalMap<unsigned int, const Action*>> action_expr_map;
  z3::expr_vector action_exprs;

  const unsigned int min_steps{0};
  const unsigned int max_steps{100};
};

namespace util {
  z3::expr predicate_var_at_step(PlanContext& plan_context,
                                 const Predicate* predicate,
                                 unsigned int step,
                                 fmt::memory_buffer& name_buf);
}  // namespace util
}  // namespace planner::symbolic
#endif
