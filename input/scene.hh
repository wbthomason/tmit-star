#ifndef SCENE_HH
#define SCENE_HH
#include <array>
#include <filesystem>
#include <optional>
#include <vector>

#include "bounds.hh"
#include "scenegraph.hh"
#include "state.hh"

namespace input::geometric {
struct SceneInfo {
  std::array<planner::geometric::Bounds, 3> bounds;
  planner::geometric::SceneGraph sg;
  planner::geometric::State<> initial_state;
};

std::optional<SceneInfo> load(const std::filesystem::path& scene_path);
}  // namespace input::geometric
#endif
