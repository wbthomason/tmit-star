local dual = require('exoplanet').dual
local ad_math = require('exoplanet').math
local max, sqrt = ad_math.max, ad_math.sqrt

function And(a, b)
  local m = max(a, 0) ^ 2.0 + max(b, 0) ^ 2.0
  if m == 0.0 then
    return dual(0)
  end

  return sqrt(m)
end

function Or(a, b)
  return -1.0 * max(-a, -b)
end

local DISTANCE_THRESHOLD = 0.05
local function at(manipulator, object)
  -- print('at: object pose', object.px, object.py, object.pz)
  -- print('at: manip pose', manipulator.px, manipulator.py, manipulator.pz)
  return max(
    (
      (object.px - manipulator.px) ^ 2.0 + (object.py - manipulator.py) ^ 2.0 + (object.pz + 0.2 - manipulator.pz) ^ 2.0
    ) - DISTANCE_THRESHOLD ^ 2.0,
    0.0
  )
end

local function above(manipulator, surface)
  -- print('surface pose', surface.pose.px, surface.pose.py, surface.pose.pz)
  -- print('surface bounds', surface.x_low, surface.x_high, surface.y_low, surface.y_high, surface.z_low, surface.z_high)
  -- print('above: object pose', object.px, object.py, object.pz)
  local x_low = surface.pose.px + surface.x_low
  local x_high = surface.pose.px + surface.x_high
  local y_low = surface.pose.py + surface.y_low
  local y_high = surface.pose.py + surface.y_high
  local z_low = surface.pose.pz + surface.z_low + 0.01
  local z_high = surface.pose.pz + surface.z_high + 0.01
  -- print('above: surface bounds: x: (', x_low, x_high, ') y: (', y_low, y_high, ') z: (', z_low, z_high, ')')

  local in_x = And(max(x_low - manipulator.px, 0.0), max(manipulator.px - x_high, 0.0))
  local in_y = And(max(y_low - manipulator.py, 0.0), max(manipulator.py - y_high, 0.0))
  local in_z = And(max(z_low - manipulator.pz, 0.0), max(manipulator.pz - z_high, 0.0))
  -- print('above: in_x', in_x)
  -- print('above: in_y', in_y)
  -- print('above: in_z', in_z)

  return And(in_x, And(in_y, in_z))
end

local function upright(object)
  return object.rx ^ 2.0 + object.ry ^ 2.0 + object.rz ^ 2.0 + (object.rw - 1.0) ^ 2.0
end

local x_bounds = { low = -0.35, high = 0.35 }
local y_bounds = { low = -0.25, high = 0.25 }
local z_bounds_lower = { low = 0.44, high = 0.54 }
local z_bounds_upper = { low = 0.84, high = 0.94 }
local z_bounds_top = { low = 1.24, high = 1.34 }

local function in_lower_region(manipulator)
  local in_x = And(max(x_bounds.low - manipulator.px, 0.0), max(manipulator.px - x_bounds.high, 0.0))
  local in_y = And(max(y_bounds.low - manipulator.py, 0.0), max(manipulator.py - y_bounds.high, 0.0))
  local in_z = And(max(z_bounds_lower.low - manipulator.pz, 0.0), max(manipulator.pz - z_bounds_lower.high, 0.0))

  return And(in_x, And(in_y, in_z))
end

local function in_upper_region(manipulator)
  local in_x = And(max(x_bounds.low - manipulator.px, 0.0), max(manipulator.px - x_bounds.high, 0.0))
  local in_y = And(max(y_bounds.low - manipulator.py, 0.0), max(manipulator.py - y_bounds.high, 0.0))
  local in_z = And(max(z_bounds_upper.low - manipulator.pz, 0.0), max(manipulator.pz - z_bounds_upper.high, 0.0))

  return And(in_x, And(in_y, in_z))
end

local function in_top_region(manipulator)
  local in_x = And(max(x_bounds.low - manipulator.px, 0.0), max(manipulator.px - x_bounds.high, 0.0))
  local in_y = And(max(y_bounds.low - manipulator.py, 0.0), max(manipulator.py - y_bounds.high, 0.0))
  local in_z = And(max(z_bounds_top.low - manipulator.pz, 0.0), max(manipulator.pz - z_bounds_top.high, 0.0))

  return And(in_x, And(in_y, in_z))
end

return {
  above = above,
  at = at,
  upright = upright,
  ['in-lower-region'] = in_lower_region,
  ['in-upper-region'] = in_upper_region,
  ['in-top-region'] = in_top_region,
}
