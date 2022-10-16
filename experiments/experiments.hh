#pragma once
#include <chrono>
#include <utility>
#include <vector>

#include "input.hh"
#include "output.hh"

namespace experiments {
std::vector<output::PlannerSolution> run_experiment(const input::PlanningProblem& problem,
                                                    const input::ExperimentParameters& experiment,
                                                    bool should_optimize,
                                                    unsigned int trial_number,
                                                    bool should_interpolate = false,
                                                    bool save_plans         = false);
}
