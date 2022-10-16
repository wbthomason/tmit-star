#ifndef PREDICATE_HH
#define PREDICATE_HH
#include <Eigen/Core>
#include <algorithm>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "autodiff/forward/real.hpp"
#include "lua_env.hh"
#include "scenegraph.hh"
#include "state.hh"
#include "utils.hh"

namespace planner::symbolic {
struct Predicate {
  Predicate(bool discrete,
            const std::string& template_name,
            const std::vector<std::string>& arguments,
            const std::vector<std::pair<unsigned int, unsigned int>> kinematic_links,
            unsigned int id)
  : id(id)
  , mode_idx(-1)
  , fn_idx(-1)
  , argspec_idx(-1)
  , name(utils::make_grounded_name(template_name, arguments))
  , arguments(arguments)
  , kinematic_links(std::move(kinematic_links))
  , discrete(discrete)
  , template_name(template_name) {}

  Predicate(const Predicate& o) = default;
  Predicate(Predicate&&)        = default;

  double
  distance_at(LuaEnv& lua_env, const PosedScene<planner::geometric::Transform3<>>& state) const;
  double gradient_at(LuaEnv& lua_env,
                     const PosedScene<planner::geometric::PoseSetPartialDerivative>& state,
                     Eigen::Ref<Eigen::VectorXd> gradient) const;

  unsigned int id;

  // NOTE: Meaningless if discrete == false
  unsigned int mode_idx;

  // NOTE: Meaningless if discrete == true
  LuaRef fn_idx;
  int argspec_idx;

  std::string name;
  std::vector<std::string> arguments;
  std::vector<std::pair<unsigned int, unsigned int>> kinematic_links;
  bool discrete;
  std::string template_name;
};
}  // namespace planner::symbolic
#endif
