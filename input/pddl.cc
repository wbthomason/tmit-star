#include "pddl.hh"

#include <fmt/core.h>

#include <algorithm>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "expression_tree.hh"
#include "predicate.hh"
#include "robin_hood.h"
#include "sexplib.hh"
#include "state.hh"
#include "symbolic.hh"
#include "utils.hh"

namespace input::symbolic {
namespace {
  using PredicateTemplateMap = robin_hood::unordered_map<std::string, PredicateTemplate>;
  using PredicateMap =
  robin_hood::unordered_map<std::string, std::shared_ptr<planner::symbolic::Predicate>>;

  std::optional<unsigned int>
  find_idx(const std::vector<std::string>& list, const std::string& value) {
    for (unsigned int i = 0; i < list.size(); ++i) {
      if (list[i] == value) {
        return i;
      }
    }

    return std::nullopt;
  }

  PredicateTemplate::PredicateTemplate(const sexp::Sexp& predicate_sexp) : discrete(false) {
    name = predicate_sexp.head.value();
    if (predicate_sexp.tail) {
      for (const auto& param : predicate_sexp.tail.value()) {
        const auto& param_value = param.head.value();
        if (param_value == "kinematic") {
          // TODO: Handle cases where unlink is different than link
          const auto* link_sexp = param.find_first("kinematic/link").value();
          const auto arg_1 =
          find_idx(parameter_names, std::string(link_sexp->tail->at(0).head.value())).value();
          const auto arg_2 =
          find_idx(parameter_names, std::string(link_sexp->tail->at(1).head.value())).value();
          links.emplace_back(arg_1, arg_2);
          discrete = true;
        } else {
          switch (param_value[0]) {
            case '?':
              parameter_names.emplace_back(param_value);
              break;
            case ':':
              if (param_value == ":discrete") {
                discrete = true;
              }
              break;

            case '-':
              const auto multiplicity = parameter_names.size() - parameter_types.size();
              const auto type_name    = param_value.substr(1);
              parameter_types.insert(parameter_types.end(), multiplicity, std::string(type_name));
              break;
          }
        }
      }
    }
  }

  ActionTemplate::ActionTemplate(const sexp::Sexp& action_sexp) {
    const auto& action_spec = action_sexp.tail.value();
    name                    = action_spec[0].head.value();
    // Find and process the parameter spec, precondition formula, and effect formula
    for (size_t idx = 0; idx < action_spec.size(); ++idx) {
      const auto& elem = action_spec[idx].head.value();
      if (elem == ":parameters") {
        const auto& parameter_sexp = action_spec[idx + 1];
        parameter_names.emplace_back(parameter_sexp.head.value());
        for (const auto& param : parameter_sexp.tail.value()) {
          const auto& param_value = param.head.value();
          if (param_value[0] == '?') {
            parameter_names.emplace_back(param_value);
          } else if (param_value[0] == '-') {
            const auto multiplicity = parameter_names.size() - parameter_types.size();
            const auto type_name    = param_value.substr(1);
            parameter_types.insert(parameter_types.end(), multiplicity, std::string(type_name));
          }
        }
      } else if (elem == ":precondition") {
        precondition = action_spec[idx + 1];
      } else if (elem == ":effect") {
        effect = action_spec[idx + 1];
      }
    }
  }

  PddlDomain::PddlDomain(const sexp::Sexp& domain_sexp) {
    name = domain_sexp.find_first("define/domain").value()->tail.value()[0].head.value();
    const auto& predicates_sexp_list =
    domain_sexp.find_first("define/:predicates").value()->tail.value();
    for (const auto& predicate_sexp : predicates_sexp_list) {
      PredicateTemplate predicate(predicate_sexp);
      predicates.emplace(predicate.name, predicate);
    }

    auto actions_sexp = domain_sexp.find_all("define/:action").value();
    while (!actions_sexp.done()) {
      actions.emplace_back(*actions_sexp);
      ++actions_sexp;
    }
  }

  template <typename T>
  std::vector<std::vector<std::string>> generate_argument_combinations(
  const T& t,
  const robin_hood::unordered_map<std::string, std::vector<std::string>>& typed_objects) {
    std::vector<const std::vector<std::string>*> object_choices;
    unsigned int num_combinations = 1;
    for (const auto& type_name : t.parameter_types) {
      const auto& object_group = typed_objects.at(type_name);
      num_combinations *= object_group.size();
      object_choices.push_back(&object_group);
    }

    std::vector<std::vector<std::string>> argument_combos;
    if (num_combinations == 0) {
      return argument_combos;
    }

    argument_combos.reserve(num_combinations);
    num_combinations /= object_choices[0]->size();
    for (const auto& obj : *object_choices[0]) {
      argument_combos.insert(argument_combos.end(), num_combinations, {obj});
      argument_combos.back().reserve(object_choices.size());
    }

    for (size_t i = 1; i < object_choices.size(); ++i) {
      num_combinations /= object_choices[i]->size();
      for (size_t c = 0; c < argument_combos.size(); ++c) {
        argument_combos[c].push_back(
        (*object_choices[i])[(c / num_combinations) % object_choices[i]->size()]);
      }
    }

    return argument_combos;
  }

  const planner::symbolic::Predicate*
  ensure_predicate(const robin_hood::unordered_map<std::string, std::string>* const parameter_map,
                   const sexp::Sexp& predicate_sexp,
                   PredicateMap& predicates,
                   const PredicateTemplateMap& predicate_templates) {
    std::vector<std::string> arguments;
    arguments.reserve(predicate_sexp.tail.value().size());
    if (parameter_map != nullptr) {
      for (const auto& argument : predicate_sexp.tail.value()) {
        arguments.emplace_back(parameter_map->at(std::string(argument.head.value())));
      }
    } else {
      for (const auto& argument : predicate_sexp.tail.value()) {
        arguments.emplace_back(argument.head.value());
      }
    }

    const auto grounded_name =
    utils::make_grounded_name(std::string(predicate_sexp.head.value()), arguments);
    auto existing_predicate_itr = predicates.find(grounded_name);
    if (existing_predicate_itr != predicates.end()) {
      return existing_predicate_itr->second.get();
    }

    const auto& predicate_template =
    predicate_templates.at(std::string(predicate_sexp.head.value()));
    const auto& [predicate_itr, _] =
    predicates.emplace(grounded_name,
                       std::make_unique<planner::symbolic::Predicate>(predicate_template.discrete,
                                                                      predicate_template.name,
                                                                      arguments,
                                                                      predicate_template.links,
                                                                      predicates.size()));
    return predicate_itr->second.get();
  }

  std::shared_ptr<planner::symbolic::ExpressionNode> make_expression_tree_from_template(
  const sexp::Sexp& formula_template,
  const robin_hood::unordered_map<std::string, std::string>* const parameter_map,
  PredicateMap& predicates,
  const PredicateTemplateMap& predicate_templates) {
    const auto& formula_operator = formula_template.head.value();
    if (formula_operator == "and") {
      auto node = std::make_shared<planner::symbolic::AndNode>();
      for (const auto& argument : formula_template.tail.value()) {
        node->operands.emplace_back(make_expression_tree_from_template(
        argument, parameter_map, predicates, predicate_templates));
      }

      return node;
    } else if (formula_operator == "or") {
      auto node = std::make_shared<planner::symbolic::OrNode>();
      for (const auto& argument : formula_template.tail.value()) {
        node->operands.emplace_back(make_expression_tree_from_template(
        argument, parameter_map, predicates, predicate_templates));
      }

      return node;
    } else {
      auto node = std::make_shared<planner::symbolic::PredicateNode>();
      node->predicate =
      ensure_predicate(parameter_map, formula_template, predicates, predicate_templates);
      return node;
    }
  }

  planner::symbolic::Formula make_formula_from_template(
  const sexp::Sexp& formula_template,
  const robin_hood::unordered_map<std::string, std::string>* const parameter_map,
  PredicateMap& predicates,
  const PredicateTemplateMap& predicate_templates) {
    return planner::symbolic::Formula(make_expression_tree_from_template(
    formula_template, parameter_map, predicates, predicate_templates));
  }

  std::pair<const planner::symbolic::Predicate*, bool>
  process_effect_predicate(const sexp::Sexp& predicate_sexp,
                           const robin_hood::unordered_map<std::string, std::string>& parameter_map,
                           PredicateMap& predicates,
                           const PredicateTemplateMap& predicate_templates) {
    if (predicate_sexp.head.value() == "not") {
      const auto predicate_ptr = ensure_predicate(
      &parameter_map, predicate_sexp.tail.value()[0], predicates, predicate_templates);
      return {predicate_ptr, false};
    } else {
      const auto predicate_ptr =
      ensure_predicate(&parameter_map, predicate_sexp, predicates, predicate_templates);
      return {predicate_ptr, true};
    }
  }

  planner::symbolic::Effect
  make_effect_from_template(const sexp::Sexp& formula_template,
                            const robin_hood::unordered_map<std::string, std::string>& parameter_map,
                            PredicateMap& predicates,
                            const PredicateTemplateMap& predicate_templates) {
    std::vector<const planner::symbolic::Predicate*> add_list;
    std::vector<const planner::symbolic::Predicate*> del_list;
    for (const auto& predicate : formula_template.tail.value()) {
      const auto& [predicate_ptr, is_added] =
      process_effect_predicate(predicate, parameter_map, predicates, predicate_templates);
      if (is_added) {
        add_list.emplace_back(predicate_ptr);
      } else {
        del_list.emplace_back(predicate_ptr);
      }
    }

    return planner::symbolic::Effect(std::move(add_list), std::move(del_list));
  }

  std::vector<planner::symbolic::Action> ground_action(
  const ActionTemplate& action_template,
  PredicateMap& predicates,
  const PredicateTemplateMap& predicate_templates,
  const robin_hood::unordered_map<std::string, std::vector<std::string>>& typed_objects) {
    const auto argument_combos = generate_argument_combinations(action_template, typed_objects);
    std::vector<planner::symbolic::Action> result;
    if (argument_combos.empty()) {
      return result;
    }

    for (const auto& arguments : argument_combos) {
      robin_hood::unordered_map<std::string, std::string> parameter_map;
      parameter_map.reserve(action_template.parameter_names.size());
      for (size_t i = 0; i < action_template.parameter_names.size(); ++i) {
        parameter_map.emplace(action_template.parameter_names[i], arguments[i]);
      }

      auto precondition = make_formula_from_template(
      action_template.precondition, &parameter_map, predicates, predicate_templates);
      auto effect = make_effect_from_template(
      action_template.effect, parameter_map, predicates, predicate_templates);
      result.emplace_back(action_template.name,
                          arguments,
                          planner::symbolic::Constraint(std::move(precondition)),
                          effect);
    }

    return result;
  }

  PddlProblem::PddlProblem(const sexp::Sexp& problem_sexp) {
    name = problem_sexp.find_first("define/problem").value()->tail.value()[0].head.value();
    const auto objects = problem_sexp.find_first("define/:objects").value()->tail.value();
    std::vector<std::string> object_names;
    for (const auto& object_sexp : objects) {
      const auto& elem = object_sexp.head.value();
      if (elem[0] == '-') {
        auto& object_group = typed_objects[std::string(elem.substr(1))];
        object_group.insert(object_group.end(), object_names.begin(), object_names.end());
        object_names.clear();
      } else {
        object_names.emplace_back(elem);
      }
    }

    initial_state = problem_sexp.find_first("define/:init").value()->tail.value();
    goal_formula  = problem_sexp.find_first("define/:goal").value()->tail.value()[0];
  }

  std::pair<planner::symbolic::Domain, planner::symbolic::Problem>
  ground(const PddlDomain& lifted_domain, const PddlProblem& lifted_problem) {
    planner::symbolic::Domain grounded_domain;
    grounded_domain.name = lifted_domain.name;
    PredicateMap predicates;
    for (const auto& action_template : lifted_domain.actions) {
      auto grounded_actions = ground_action(
      action_template, predicates, lifted_domain.predicates, lifted_problem.typed_objects);
      grounded_domain.actions.insert(grounded_domain.actions.end(),
                                     grounded_actions.begin(),
                                     grounded_actions.end());
    }

    planner::symbolic::Constraint goal(make_formula_from_template(
    lifted_problem.goal_formula, nullptr, predicates, lifted_domain.predicates));
    std::vector<const planner::symbolic::Predicate*> grounded_initial_predicates;
    grounded_initial_predicates.reserve(lifted_problem.initial_state.size());
    for (const auto& predicate_sexp : lifted_problem.initial_state) {
      grounded_initial_predicates.push_back(
      ensure_predicate(nullptr, predicate_sexp, predicates, lifted_domain.predicates));
    }

    // Now we have all the possible predicates
    grounded_domain.predicates.reserve(predicates.size());
    for (auto& predicate_elem : predicates) {
      grounded_domain.predicates.emplace_back(std::move(predicate_elem.second));
    }

    std::sort(grounded_domain.predicates.begin(),
              grounded_domain.predicates.end(),
              [](const auto& pred_a, const auto& pred_b) { return pred_a->id < pred_b->id; });

    auto num_discrete_predicates = 0;
    for (auto& predicate : grounded_domain.predicates) {
      if (predicate->discrete) {
        predicate->mode_idx = num_discrete_predicates;
        ++num_discrete_predicates;
      } else {
        // NOTE: Intentionally setting an unsigned int to a constant negative value such that this
        // will produce more obvious errors if we try to access this property on non-discrete
        // predicates
        predicate->mode_idx = -1;
      }
    }

    // We have to go back and fix the bitsets in the action effects, since we didn't know the total
    // number of grounded predicates when we constructed effects, and all the bitsets need to be the
    // same length
    for (auto& action : grounded_domain.actions) {
      action.effect.set_bits(num_discrete_predicates);
    }

    planner::symbolic::State initial_mode(num_discrete_predicates);
    for (const auto& initial_predicate : grounded_initial_predicates) {
      if (initial_predicate->discrete) {
        initial_mode[initial_predicate->mode_idx] = true;
      }
    }

    planner::symbolic::Problem grounded_problem(lifted_problem.name, initial_mode, goal);
    return {grounded_domain, grounded_problem};
  }

}  // namespace

std::optional<std::pair<planner::symbolic::Domain, planner::symbolic::Problem>>
load(const std::filesystem::path& domain_path, const std::filesystem::path& problem_path) {
  auto log = utils::get_logger("tmit-star::input::pddl");
  log->debug("Loading domain s-expression from {}", domain_path.c_str());
  if (!std::filesystem::exists(domain_path)) {
    log->error("No domain file at provided path!");
    return std::nullopt;
  }

  const auto domain_string_result = utils::read_file_to_string(std::move(domain_path));
  log->debug("Loading problem s-expression from {}", problem_path.c_str());
  if (!std::filesystem::exists(problem_path)) {
    log->error("No problem file at provided path!");
    return std::nullopt;
  }

  const auto problem_string_result = utils::read_file_to_string(std::move(problem_path));
  if (!domain_string_result) {
    log->error("Failed to read domain file!");
    return std::nullopt;
  }

  if (!problem_string_result) {
    log->error("Failed to read problem file!");
    return std::nullopt;
  }

  const auto domain_string  = *domain_string_result;
  const auto problem_string = *problem_string_result;
  const auto lifted_domain  = PddlDomain(sexp::parse<sexp::Sexp>(domain_string));
  const auto lifted_problem = PddlProblem(sexp::parse<sexp::Sexp>(problem_string));
  return ground(lifted_domain, lifted_problem);
}
}  // namespace input::symbolic
