#ifndef LUA_ENV_HH
#define LUA_ENV_HH

extern "C" {
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>

#include "luajit.h"
}

#include <Eigen/Core>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "scenegraph.hh"
#include "spdlog/spdlog.h"
#include "state.hh"

namespace planner::symbolic {
template <typename PoseT> struct PosedScene {
  PosedScene(const planner::geometric::SceneGraph& sg, const std::vector<PoseT>& poses)
  : sg(sg), poses(poses) {}
  const planner::geometric::SceneGraph& sg;
  const std::vector<PoseT>& poses;
};

using LuaRef = int;

struct Dual {
  double v;
  double d;
};

template <typename NumT> struct LuaObject {
  NumT px;
  NumT py;
  NumT pz;
  NumT rw;
  NumT rx;
  NumT ry;
  NumT rz;
};

template <typename NumT> struct LuaSurface {
  LuaObject<NumT> pose;

  double x_low;
  double x_high;

  double y_low;
  double y_high;

  double z_low;
  double z_high;
};

struct PredicateArgSpec {
  std::vector<size_t> object_indices;
  std::vector<size_t> surface_indices;
};

struct LuaEnv {
  LuaEnv(std::filesystem::path predicate_file_path,
         size_t gradient_size,
         size_t num_objects,
         size_t num_surfaces);
  lua_State* l;
  std::shared_ptr<spdlog::logger> log;
  std::vector<PredicateArgSpec> predicate_fn_specs;
  std::vector<LuaSurface<double>> distance_surfaces;
  std::vector<LuaObject<double>> distance_objects;
  std::unique_ptr<LuaSurface<Dual>[]> gradient_surfaces;
  std::unique_ptr<LuaSurface<Dual>*[]> gradient_surfaces_array;
  std::unique_ptr<LuaObject<Dual>[]> gradient_objects;
  std::unique_ptr<LuaObject<Dual>*[]> gradient_objects_array;
};

std::pair<int, std::vector<const char*>>
generate_predicate_argspec(LuaEnv& lua_env,
                           const std::vector<std::string>& arguments,
                           const geometric::SceneInfo& scene_info);

void load_predicate(LuaEnv& lua_env,
                    const std::string& name,
                    const std::vector<const char*>& argument_types,
                    const geometric::SceneInfo& scene_info);

double call_distance_function(LuaEnv& lua_env,
                              const PosedScene<planner::geometric::Transform3<>>& state,
                              LuaRef fn_idx,
                              unsigned int argspec_idx);

double call_gradient_function(LuaEnv& lua_env,
                              const PosedScene<planner::geometric::PoseSetPartialDerivative>& state,
                              Eigen::Ref<Eigen::VectorXd> gradient,
                              LuaRef fn_idx,
                              unsigned int argspec_idx);
}  // namespace planner::symbolic
#endif
