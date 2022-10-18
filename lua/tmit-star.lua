local ffi = require 'ffi'

local M = {}
-- Dual numbers for forward-mode autodiff. Adapted from sci-lua/diff with some further
-- simplification and use-specific optimization

local dual

local abs = math.abs
local max = math.max
local min = math.min
local floor = math.floor
local sin = math.sin
local asin = math.asin
local cos = math.cos
local acos = math.acos
local tan = math.tan
local atan = math.atan
local log = math.log
local sqrt = math.sqrt
local sinh = math.sinh
local cosh = math.cosh
local tanh = math.tanh
local exp = math.exp

local function step(x)
  return max(0, min(floor(x) + 1, 1))
end

local function sign(x)
  return -1 + step(x) * 2
end

local function dual_pow(x, y)
  if type(y) == 'number' then
    return dual(x.v ^ y, y * x.v ^ (y - 1) * x.d)
  elseif type(x) == 'number' then
    return dual(x ^ y.v, x ^ y.v * log(x) * y.d)
  else
    return dual(x.v ^ y.v, x.v ^ y.v * (log(x.v) * y.d + y.v / x.v * x.d))
  end
end

local dual_mt = {
  __unm = function(x)
    return dual(-x.v, -x.d)
  end,
  __add = function(x, y)
    x, y = dual(x), dual(y)
    return dual(x.v + y.v, x.d + y.d)
  end,
  __sub = function(x, y)
    x, y = dual(x), dual(y)
    return dual(x.v - y.v, x.d - y.d)
  end,
  __mul = function(x, y)
    x, y = dual(x), dual(y)
    return dual(x.v * y.v, x.d * y.v + y.d * x.v)
  end,
  __div = function(x, y)
    x, y = dual(x), dual(y)
    return dual(x.v / y.v, (x.d * y.v - y.d * x.v) / y.v ^ 2)
  end,
  __pow = dual_pow,
  __eq = function(x, y)
    x, y = dual(x), dual(y)
    return x.v == y.v
  end,
  __lt = function(x, y)
    x, y = dual(x), dual(y)
    return x.v < y.v
  end,
  __le = function(x, y)
    x, y = dual(x), dual(y)
    return x.v <= y.v
  end,
  __tostring = function(x)
    return '(v: ' .. tostring(x.v) .. ', d: ' .. tostring(x.d) .. ')'
  end,
  __tonumber = function(x)
    return tonumber(x.v)
  end,
}

dual_mt.__index = dual_mt

ffi.cdef [[
typedef struct { double v; double d; } Dual;
]]

dual = ffi.metatype('Dual', dual_mt)

M.math = {
  sin = function(x)
    return type(x) ~= 'number' and dual(sin(x.v), x.d * cos(x.v)) or sin(x)
  end,
  cos = function(x)
    return type(x) ~= 'number' and dual(cos(x.v), x.d * (-sin(x.v))) or cos(x)
  end,
  tan = function(x)
    return type(x) ~= 'number' and dual(tan(x.v), x.d * (1 + tan(x.v) ^ 2)) or tan(x)
  end,
  asin = function(x)
    return type(x) ~= 'number' and dual(asin(x.v), x.d / sqrt(1 - x.v ^ 2)) or asin(x)
  end,
  acos = function(x)
    return type(x) ~= 'number' and dual(acos(x.v), -x.d / sqrt(1 - x.v ^ 2)) or acos(x)
  end,
  atan = function(x)
    return type(x) ~= 'number' and dual(atan(x.v), x.d / (1 + x.v ^ 2)) or atan(x)
  end,
  sinh = function(x)
    return type(x) ~= 'number' and dual(sinh(x.v), x.d * cosh(x.v)) or sinh(x)
  end,
  cosh = function(x)
    return type(x) ~= 'number' and dual(cosh(x.v), x.d * sinh(x.v)) or cosh(x)
  end,
  tanh = function(x)
    return type(x) ~= 'number' and dual(tanh(x.v), x.d * (1 - tanh(x.v) ^ 2)) or tanh(x)
  end,
  exp = function(x)
    return type(x) ~= 'number' and dual(exp(x.v), x.d * exp(x.v)) or exp(x)
  end,
  log = function(x)
    return type(x) ~= 'number' and dual(log(x.v), x.d / x.v) or log(x)
  end,
  sqrt = function(x)
    return type(x) ~= 'number' and dual(sqrt(x.v), x.d / (2 * sqrt(x.v))) or sqrt(x)
  end,
  abs = function(x)
    return type(x) ~= 'number' and dual(abs(x.v), x.d * sign(x.v)) or abs(x)
  end,
  pow = dual_pow,
  max = function(x, y)
    x, y = dual(x), dual(y)
    if max(abs(x.v), abs(y.v)) == 1 / 0 then
      return x >= y and x or y
    else
      local z = step(y.v - x.v)
      return dual(z * y.v + (1 - z) * x.v, z * y.d + (1 - z) * x.d)
    end
  end,
  min = function(x, y)
    x, y = dual(x), dual(y)
    if max(abs(x.v), abs(y.v)) == 1 / 0 then
      return x <= y and x or y
    else
      local z = step(x.v - y.v)
      return dual(z * y.v + (1 - z) * x.v, z * y.d + (1 - z) * x.d)
    end
  end,
}

ffi.cdef [[
typedef struct {
  double px;
  double py;
  double pz;

  double rw;
  double rx;
  double ry;
  double rz;
} Object;

typedef struct {
  Dual px;
  Dual py;
  Dual pz;

  Dual rw;
  Dual rx;
  Dual ry;
  Dual rz;
} DiffObject;

typedef struct {
  Object pose;
  
  double x_low;
  double x_high;

  double y_low;
  double y_high;

  double z_low;
  double z_high;
} Surface;

typedef struct {
  DiffObject pose;
  
  double x_low;
  double x_high;

  double y_low;
  double y_high;

  double z_low;
  double z_high;
} DiffSurface;
]]

-- Utility function to handle casting light userdata to proper FFI type
local function generate_dist_setup(...)
  local arg_spec = { ... }
  local object_indices = {}
  local surface_indices = {}
  local args = {}
  for i = 1, #arg_spec do
    if arg_spec[i] == 'object' then
      object_indices[#object_indices + 1] = i
    else
      surface_indices[#surface_indices + 1] = i
    end

    args[#args + 1] = true
  end

  local objects_arr_t = ffi.typeof('Object(&)[' .. tostring(#object_indices) .. ']')
  local surfaces_arr_t = ffi.typeof('Surface(&)[' .. tostring(#surface_indices) .. ']')

  return function(objects_ptr, surfaces_ptr)
    local objects = ffi.cast(objects_arr_t, objects_ptr)
    local surfaces = ffi.cast(surfaces_arr_t, surfaces_ptr)
    for i = 1, #object_indices do
      args[object_indices[i]] = objects[i - 1]
    end

    for i = 1, #surface_indices do
      args[surface_indices[i]] = surfaces[i - 1]
    end

    return args
  end
end

-- Utility function to coordinate casting args and calling predicate for distance
M.dist = function(predicate, ...)
  local setup_fn = generate_dist_setup(...)
  return function(objects, surfaces)
    local args = setup_fn(objects, surfaces)
    local result = predicate(unpack(args))
    if type(result) == 'number' then
      return result
    else
      return result.v
    end
  end
end

local function make_arg_array(num_args)
  local result = {}
  for _ = 1, num_args do
    result[#result + 1] = true
  end

  return result
end

local function generate_grad_setup(grad_len, ...)
  local arg_spec = { ... }
  local object_indices = {}
  local surface_indices = {}
  for i = 1, #arg_spec do
    if arg_spec[i] == 'object' then
      object_indices[#object_indices + 1] = i
    else
      surface_indices[#surface_indices + 1] = i
    end
  end

  local objects_arr_t = ffi.typeof 'DiffObject**'
  local surfaces_arr_t = ffi.typeof 'DiffSurface**'
  local grad_t = ffi.typeof('double(&)[' .. tostring(grad_len) .. ']')
  local args = {}
  for _ = 1, grad_len do
    args[#args + 1] = make_arg_array(#arg_spec)
  end

  return function(objects_ptr, surfaces_ptr, grad_ptr)
    local objects = ffi.cast(objects_arr_t, objects_ptr)
    local surfaces = ffi.cast(surfaces_arr_t, surfaces_ptr)
    local grad = ffi.cast(grad_t, grad_ptr)
    for i = 1, grad_len do
      for j = 1, #object_indices do
        args[i][object_indices[j]] = objects[i - 1][j - 1]
      end

      for j = 1, #surface_indices do
        args[i][surface_indices[j]] = surfaces[i - 1][j - 1]
      end
    end

    return args, grad
  end
end

-- Utility function to coordinate casting args and iterating over all components
-- for gradient
M.grad = function(predicate, grad_len, ...)
  local setup_fn = generate_grad_setup(grad_len, ...)
  return function(objects, surfaces, grad_ptr)
    local args, grad = setup_fn(objects, surfaces, grad_ptr)
    local result = 0.0
    for i = 1, grad_len do
      local pred_result = predicate(unpack(args[i]))
      result = pred_result.v
      grad[i - 1] = pred_result.d
    end

    return result
  end
end

M.dual = dual

return M
