#ifndef PDDL_HH
#define PDDL_HH

#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "robin_hood.h"
#include "sexplib.hh"
#include "symbolic.hh"

namespace input::symbolic {
namespace {
  struct PredicateTemplate {
    std::string name;
    bool discrete;
    std::vector<std::string> parameter_names;
    std::vector<std::string> parameter_types;
    std::vector<std::pair<unsigned int, unsigned int>> links;
    explicit PredicateTemplate(const sexp::Sexp& predicate_sexp);
  };

  struct ActionTemplate {
    std::string name;
    std::vector<std::string> parameter_names;
    std::vector<std::string> parameter_types;
    sexp::Sexp precondition;
    sexp::Sexp effect;
    explicit ActionTemplate(const sexp::Sexp& action_sexp);
  };

  struct PddlDomain {
    std::string name;
    robin_hood::unordered_map<std::string, PredicateTemplate> predicates;
    std::vector<ActionTemplate> actions;
    explicit PddlDomain(const sexp::Sexp& domain_sexp);
  };

  struct PddlProblem {
    std::string name;
    robin_hood::unordered_map<std::string, std::vector<std::string>> typed_objects;
    std::vector<sexp::Sexp> initial_state;
    sexp::Sexp goal_formula;
    explicit PddlProblem(const sexp::Sexp& problem_sexp);
  };
}  // namespace

std::optional<std::pair<planner::symbolic::Domain, planner::symbolic::Problem>>
load(const std::filesystem::path& domain_path, const std::filesystem::path& problem_path);
}  // namespace input::symbolic
#endif
