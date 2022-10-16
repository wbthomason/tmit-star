#include "lua_env.hh"

extern "C" {
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>

#include "luajit.h"
}

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "autodiff/forward/real.hpp"
#include "scenegraph.hh"
#include "state.hh"
#include "utils.hh"

namespace planner::symbolic {

constexpr char DIST_FN_KEY[]           = "exoplanet_dist_fn";
constexpr char GRAD_FN_KEY[]           = "exoplanet_grad_fn";
constexpr char PREDICATES_MODULE_KEY[] = "exoplanet_predicates_mod";

constexpr char TRACEBACK_FN_NAME[] = "exoplanet_err_func";
static int traceback(lua_State* L) {
  // 'message' not a string?
  if (!lua_isstring(L, 1)) {
    return 1;  // Keep it intact
  }

  lua_getfield(L, LUA_GLOBALSINDEX, "debug");
  if (!lua_istable(L, -1)) {
    lua_pop(L, 1);
    return 1;
  }

  lua_getfield(L, -1, "traceback");
  if (!lua_isfunction(L, -1)) {
    lua_pop(L, 2);
    return 1;
  }

  lua_pushvalue(L, 1);    // Pass error message
  lua_pushinteger(L, 2);  // Skip this function and traceback
  lua_call(L, 2, 1);      // Call debug.traceback
  return 1;
}

#ifdef DEBUG_LUA
static int wrap_exceptions(lua_State* L, lua_CFunction f) {
  try {
    return f(L);             // Call wrapped function and return result.
  } catch (const char* s) {  // Catch and convert exceptions.
    lua_pushstring(L, s);
  } catch (std::exception& e) {
    lua_pushstring(L, e.what());
  } catch (...) {
    lua_pushliteral(L, "caught (...)");
  }

  return lua_error(L);  // Rethrow as a Lua error.
}
#endif

void load_c_fns(lua_State* l) {
  lua_pushcfunction(l, traceback);
  lua_setglobal(l, TRACEBACK_FN_NAME);
#ifdef DEBUG_LUA
  lua_pushlightuserdata(l, reinterpret_cast<void*>(wrap_exceptions));
  luaJIT_setmode(l, -1, LUAJIT_MODE_WRAPCFUNC | LUAJIT_MODE_ON);
  lua_pop(l, 1);
#endif
}

void set_package_path(lua_State* l) {
  lua_getglobal(l, "package");
  lua_getfield(l, -1, "path");
  std::string path = lua_tostring(l, -1);
  path.append(";lua/?.lua");
  lua_pop(l, 1);
  lua_pushstring(l, path.c_str());
  lua_setfield(l, -2, "path");
  lua_pop(l, 1);
}

LuaEnv::LuaEnv(std::filesystem::path predicate_file_path,
               size_t gradient_size,
               size_t num_objects,
               size_t num_surfaces)
: l(luaL_newstate())
, log(utils::get_logger("predicate env"))
, predicate_fn_specs()
, distance_surfaces()
, distance_objects() {
  // Setup surface, object, and gradient object buffers
  distance_surfaces.reserve(num_surfaces);
  for (size_t i = 0; i < num_surfaces; ++i) {
    distance_surfaces.emplace_back();
  }

  distance_objects.reserve(num_objects);
  for (size_t i = 0; i < num_objects; ++i) {
    distance_objects.emplace_back();
  }

  gradient_objects       = std::make_unique<LuaObject<Dual>[]>(gradient_size * num_objects);
  gradient_objects_array = std::make_unique<LuaObject<Dual>*[]>(gradient_size);
  for (size_t i = 0; i < gradient_size; ++i) {
    gradient_objects_array[i] = &gradient_objects[i * num_objects];
  }

  gradient_surfaces       = std::make_unique<LuaSurface<Dual>[]>(gradient_size * num_surfaces);
  gradient_surfaces_array = std::make_unique<LuaSurface<Dual>*[]>(gradient_size);
  for (size_t i = 0; i < gradient_size; ++i) {
    gradient_surfaces_array[i] = &gradient_surfaces[i * num_surfaces];
  }

  // Setup environment with Lua standard libraries and traceback function
  luaL_openlibs(l);
  load_c_fns(l);
  set_package_path(l);

  // Get the exoplanet module
  lua_getglobal(l, "require");
  lua_pushstring(l, "exoplanet");
  lua_call(l, 1, 1);

  // Get and store the exoplanet.dist function
  lua_pushstring(l, DIST_FN_KEY);
  lua_pushstring(l, "dist");
  lua_rawget(l, -3);
  lua_rawset(l, LUA_REGISTRYINDEX);

  // Get and store the exoplanet.grad function
  lua_pushstring(l, GRAD_FN_KEY);
  lua_pushstring(l, "grad");
  lua_rawget(l, -3);
  lua_rawset(l, LUA_REGISTRYINDEX);

  // Load the predicate module
  lua_pushstring(l, PREDICATES_MODULE_KEY);
  if (luaL_dofile(l, predicate_file_path.c_str()) != 0) {
    auto err_msg = lua_tostring(l, -1);
    log->error("Loading predicates from {} failed: {}", predicate_file_path.c_str(), err_msg);
    throw std::runtime_error("Failed to load predicates!");
  }

  // Store the predicate module in the registry
  lua_rawset(l, LUA_REGISTRYINDEX);

  // Create array for predicate functions
  lua_newtable(l);
}

void call_fn_with_spec(lua_State* l,
                       const char* exoplanet_fn_name,
                       const std::optional<int> gradient_size,
                       const std::vector<const char*>& argument_spec) {
  // Get the function
  lua_pushstring(l, exoplanet_fn_name);
  lua_rawget(l, LUA_REGISTRYINDEX);

  // Copy the function it takes as its first argument
  lua_pushvalue(l, -2);

  // If we were given a gradient size, push it too
  if (gradient_size) {
    lua_pushinteger(l, *gradient_size);
  }

  // Push on the argument spec
  for (const auto* arg : argument_spec) {
    lua_pushstring(l, arg);
  }

  // Call the function
  lua_call(l, argument_spec.size() + 1 + (gradient_size ? 1 : 0), 1);
}

std::pair<int, std::vector<const char*>>
generate_predicate_argspec(LuaEnv& lua_env,
                           const std::vector<std::string>& arguments,
                           const geometric::SceneInfo& scene_info) {
  // Build the argument spec
  std::vector<const char*> argument_types;
  PredicateArgSpec spec;
  for (const auto& arg : arguments) {
    // This is not super efficient, but I'm too lazy to change the type of arguments all the way
    // through and this is not a performance-critical function
    const auto object_idx   = scene_info.index.at(arg);
    const auto& object_info = scene_info.joint_info[object_idx];
    if (object_info.surface_bounds) {
      spec.surface_indices.push_back(object_idx);
      argument_types.emplace_back("surface");
    } else {
      spec.object_indices.push_back(object_idx);
      argument_types.emplace_back("object");
    }
  }

  lua_env.predicate_fn_specs.push_back(spec);

  // Return the index into the fn_specs vector and the argument types vector
  return {static_cast<int>(lua_env.predicate_fn_specs.size() - 1), argument_types};
}

void load_predicate(LuaEnv& lua_env,
                    const std::string& name,
                    const std::vector<const char*>& argument_types,
                    const geometric::SceneInfo& scene_info) {
  // Get the predicates module from the registry
  auto* l = lua_env.l;
  lua_pushstring(l, PREDICATES_MODULE_KEY);
  lua_rawget(l, LUA_REGISTRYINDEX);

  // Get the requested predicate function
  lua_pushstring(l, name.data());
  lua_rawget(l, -2);

  // Call exoplanet.dist on the predicate function
  call_fn_with_spec(l, DIST_FN_KEY, std::nullopt, argument_types);

  // Store the result in the predicate function array
  lua_rawseti(l, 2, 2 * lua_env.predicate_fn_specs.size());

  // Call exoplanet.grad on the predicate function
  call_fn_with_spec(l, GRAD_FN_KEY, 3 + scene_info.num_controllable_joints, argument_types);

  // Store the result in the predicate function array
  lua_rawseti(l, 2, 2 * lua_env.predicate_fn_specs.size() + 1);

  lua_pop(l, 2);
}

#define COPY_BOUNDS(var_name)                              \
  surface.var_name##_low  = surface_bounds.var_name##_low; \
  surface.var_name##_high = surface_bounds.var_name##_high;

void make_surface(LuaEnv& lua_env,
                  size_t idx,
                  const geometric::Transform3<>& pose,
                  const geometric::AreaBounds& surface_bounds) {
  auto& surface           = lua_env.distance_surfaces[idx];
  const auto& translation = pose.translation();
  const auto& rotation    = Eigen::Quaterniond(pose.linear());

  surface.pose.px = translation.x();
  surface.pose.py = translation.y();
  surface.pose.pz = translation.z();

  surface.pose.rw = rotation.w();
  surface.pose.rx = rotation.x();
  surface.pose.ry = rotation.y();
  surface.pose.rz = rotation.z();

  COPY_BOUNDS(x);
  COPY_BOUNDS(y);
  COPY_BOUNDS(z);
}

#define COPY_AD_SURFACE_TRANSLATION(dim_name)             \
  surface.pose.p##dim_name.v = translation.dim_name()[0]; \
  surface.pose.p##dim_name.d = translation.dim_name()[1];

#define COPY_AD_SURFACE_ROTATION(dim_name)             \
  surface.pose.r##dim_name.v = rotation.dim_name()[0]; \
  surface.pose.r##dim_name.d = rotation.dim_name()[1];

void make_surface(LuaEnv& lua_env,
                  size_t grad_idx,
                  size_t idx,
                  const geometric::Transform3<autodiff::real>& pose,
                  const geometric::AreaBounds& surface_bounds) {
  auto& surface           = lua_env.gradient_surfaces_array[grad_idx][idx];
  const auto& translation = pose.translation();
  Eigen::Quaternion<autodiff::real> rotation(pose.linear());

  COPY_AD_SURFACE_TRANSLATION(x);
  COPY_AD_SURFACE_TRANSLATION(y);
  COPY_AD_SURFACE_TRANSLATION(z);

  COPY_AD_SURFACE_ROTATION(w);
  COPY_AD_SURFACE_ROTATION(x);
  COPY_AD_SURFACE_ROTATION(y);
  COPY_AD_SURFACE_ROTATION(z);

  surface.x_high = surface_bounds.x_high;
  surface.x_low  = surface_bounds.x_low;
  surface.y_high = surface_bounds.y_high;
  surface.y_low  = surface_bounds.y_low;
  surface.z_high = surface_bounds.z_high;
  surface.z_low  = surface_bounds.z_low;
}

void make_object(LuaEnv& lua_env, size_t idx, const geometric::Transform3<>& pose) {
  auto& object            = lua_env.distance_objects[idx];
  const auto& translation = pose.translation();
  Eigen::Quaterniond rotation(pose.linear());

  object.px = translation.x();
  object.py = translation.y();
  object.pz = translation.z();

  object.rw = rotation.w();
  object.rx = rotation.x();
  object.ry = rotation.y();
  object.rz = rotation.z();
}

#define COPY_AD_OBJECT_TRANSLATION(dim_name)        \
  object.p##dim_name.v = translation.dim_name()[0]; \
  object.p##dim_name.d = translation.dim_name()[1];

#define COPY_AD_OBJECT_ROTATION(dim_name)        \
  object.r##dim_name.v = rotation.dim_name()[0]; \
  object.r##dim_name.d = rotation.dim_name()[1];

void make_object(LuaEnv& lua_env,
                 size_t grad_idx,
                 size_t idx,
                 const geometric::Transform3<autodiff::real>& pose) {
  auto& object            = lua_env.gradient_objects_array[grad_idx][idx];
  const auto& translation = pose.translation();
  const auto& rotation    = Eigen::Quaternion<autodiff::real>(pose.linear());

  COPY_AD_OBJECT_TRANSLATION(x);
  COPY_AD_OBJECT_TRANSLATION(y);
  COPY_AD_OBJECT_TRANSLATION(z);

  COPY_AD_OBJECT_ROTATION(w);
  COPY_AD_OBJECT_ROTATION(x);
  COPY_AD_OBJECT_ROTATION(y);
  COPY_AD_OBJECT_ROTATION(z);
}

template <typename PoseT>
void load_objects(LuaEnv& lua_env,
                  const PosedScene<PoseT>& state,
                  const PredicateArgSpec& arg_spec) {
  if constexpr (std::is_same<PoseT, geometric::Transform3<>>::value) {
    for (size_t i = 0; i < arg_spec.surface_indices.size(); ++i) {
      const auto surface_idx = arg_spec.surface_indices[i];
      const auto& bounds     = state.sg.scene_info->joint_info[surface_idx].surface_bounds;
      make_surface(lua_env, i, state.poses[surface_idx], *bounds);
    }

    for (size_t i = 0; i < arg_spec.object_indices.size(); ++i) {
      const auto object_idx = arg_spec.object_indices[i];
      make_object(lua_env, i, state.poses[object_idx]);
    }
  } else {
    for (size_t i = 0; i < state.poses.size(); ++i) {
      for (size_t j = 0; j < arg_spec.surface_indices.size(); ++j) {
        const auto surface_idx = arg_spec.surface_indices[j];
        const auto& bounds     = state.sg.scene_info->joint_info[surface_idx].surface_bounds;
        make_surface(lua_env, i, j, state.poses[i][surface_idx], *bounds);
      }
    }

    for (size_t i = 0; i < state.poses.size(); ++i) {
      for (size_t j = 0; j < arg_spec.object_indices.size(); ++j) {
        const auto object_idx = arg_spec.object_indices[j];
        make_object(lua_env, i, j, state.poses[i][object_idx]);
      }
    }
  }
}

double call_distance_function(LuaEnv& lua_env,
                              const PosedScene<geometric::Transform3<>>& state,
                              LuaRef fn_idx,
                              unsigned int argspec_idx) {
  auto* l = lua_env.l;
  lua_getglobal(l, TRACEBACK_FN_NAME);
  const int err_func_idx = lua_gettop(l);

  // Fetch the distance function for this predicate
  lua_rawgeti(l, 2, 2 * fn_idx + 2);

  // Load the posed objects
  const auto& arg_spec = lua_env.predicate_fn_specs[argspec_idx];
  load_objects(lua_env, state, arg_spec);

  // Push the pointers to the lists of objects and surfaces and call the function
  lua_pushlightuserdata(l, lua_env.distance_objects.data());
  lua_pushlightuserdata(l, lua_env.distance_surfaces.data());
  if (lua_pcall(l, 2, 1, err_func_idx) != 0) {
    spdlog::get("predicate env")
    ->error("Error running Lua distance function: {}", lua_tostring(l, -1));
  }

  // Get the result and clean up
  const auto result = lua_tonumber(l, -1);
  lua_pop(l, 2);
  return result;
}

double call_gradient_function(LuaEnv& lua_env,
                              const PosedScene<geometric::PoseSetPartialDerivative>& state,
                              Eigen::Ref<Eigen::VectorXd> gradient,
                              LuaRef fn_idx,
                              unsigned int argspec_idx) {
  auto* l = lua_env.l;

  lua_getglobal(l, TRACEBACK_FN_NAME);
  const int err_func_idx = lua_gettop(l);

  // Fetch the gradient function for this predicate
  lua_rawgeti(l, 2, 2 * fn_idx + 3);

  // Load the posed objects
  const auto& arg_spec = lua_env.predicate_fn_specs[argspec_idx];
  load_objects(lua_env, state, arg_spec);

  // Push the pointers to the lists of objects and surfaces, as well as the pointer to the
  // gradient, and call the function
  lua_pushlightuserdata(l, lua_env.gradient_objects_array.get());
  lua_pushlightuserdata(l, lua_env.gradient_surfaces_array.get());
  lua_pushlightuserdata(l, gradient.data());
  if (lua_pcall(l, 3, 1, err_func_idx) != 0) {
    spdlog::get("predicate env")
    ->error("Error running Lua distance function: {}", lua_tostring(l, -1));
  }

  // Get the result and clean up
  const auto result = lua_tonumber(l, -1);
  lua_pop(l, 2);

  return result;
}
}  // namespace planner::symbolic
