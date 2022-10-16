#ifndef OUTPUT_HH
#define OUTPUT_HH

#include <ompl/geometric/PathGeometric.h>

#include <filesystem>

#include "mode_atlas.hh"
#include "nlohmann/json_fwd.hpp"

namespace output {
void output_plan(const ompl::geometric::PathGeometric* const plan,
                 planner::ModeAtlas* mode_atlas,
                 const std::filesystem::path output_filepath);
struct PlannerSolution {
  double cost;
  long time_ms;
};

void to_json(nlohmann::json& j, const PlannerSolution& p);

void action_aware_interpolate(ompl::geometric::PathGeometric* plan);
}  // namespace output
#endif
