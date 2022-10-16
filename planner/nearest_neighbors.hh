#ifndef NEAREST_NEIGHBORS_HH
#define NEAREST_NEIGHBORS_HH
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

#include "cspace.hh"
#include "mode.hh"
#include "mode_atlas.hh"
#include "ompl/base/State.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "robin_hood.h"

namespace planner {
template <typename MotionT, typename NearestNeighborsT>
struct MultimodalNearestNeighbors : public ompl::NearestNeighbors<MotionT> {
  explicit MultimodalNearestNeighbors(const ModeAtlas* const mode_atlas)
  : ompl::NearestNeighbors<MotionT>(), mode_atlas(mode_atlas), counter{0} {}
  ~MultimodalNearestNeighbors() override = default;
  constexpr bool reportsSortedResults() const override { return true; }
  void clear() override {
    local_nns.clear();
    counter = 0;
  }

  size_t size() const override { return counter; }

  void add(const MotionT& motion) override {
    const auto* cstate = motion->getState()->template as<CompositeStateSpace::StateType>();
    const auto& mode   = cstate->get_mode();
    if (!local_nns.contains(mode)) {
      auto [mode_nn_it, _] = local_nns.emplace(mode, std::make_unique<NearestNeighborsT>());
      mode_nn_it->second->setDistanceFunction(continuous_distance);
    }

    local_nns[mode]->add(motion);
    ++counter;

    if (cstate->transition) {
      const auto& transition_mode = cstate->transition_mode;
      entry_motions[transition_mode].emplace_back(motion);
    }
  }

  void add(const std::vector<MotionT>& motions) override {
    // TODO: Is there a more efficient way to do this?
    for (const auto& motion : motions) {
      add(motion);
    }
  }

  bool remove(const MotionT& motion) override {
    const auto* cstate = motion->getState()->template as<CompositeStateSpace::StateType>();
    const auto& mode   = cstate->get_mode();
    auto mode_nn_it    = local_nns.find(mode);
    if (mode_nn_it != local_nns.end()) {
      const auto remove_result = mode_nn_it->second->remove(motion);
      counter -= static_cast<unsigned int>(remove_result);
      return remove_result;
    }

    return false;
  }

  void list(std::vector<MotionT>& motions) const override {
    motions.reserve(counter);
    std::vector<MotionT> local_motions;
    for (const auto& [_, mode_nn] : local_nns) {
      local_motions.clear();
      local_motions.reserve(mode_nn->size());
      mode_nn->list(local_motions);
      motions.insert(motions.end(), local_motions.begin(), local_motions.end());
    }
  }

  MotionT nearest(const MotionT& motion) const override {
    const auto* cstate = motion->getState()->template as<CompositeStateSpace::StateType>();
    const auto& mode   = cstate->get_mode();
    auto mode_nn_it    = local_nns.find(mode);
    return mode_nn_it->second->nearest(motion);
  }

  void permute(std::vector<MotionT>& neighbors, std::vector<int>& permutation) const {
    const unsigned int num_neighbors = neighbors.size();
    unsigned int idx                 = 0;
    unsigned int swap_start          = 0;
    unsigned int swapped_count       = 0;
    unsigned int permutation_idx     = 0;
    while (swapped_count < num_neighbors) {
      while (swap_start < num_neighbors && permutation[swap_start] < 0) {
        ++swap_start;
      }

      idx             = swap_start;
      permutation_idx = permutation[swap_start];
      ++swapped_count;
      while (permutation_idx != swap_start) {
        std::swap(neighbors[idx], neighbors[permutation_idx]);
        permutation[idx] = -1;
        idx              = permutation_idx;
        permutation_idx  = permutation[permutation_idx];
        ++swapped_count;
      }

      permutation[idx] = -1;
      ++swap_start;
    }
  }

  void sort_neighbors_by_distance(const MotionT& motion, std::vector<MotionT>& neighbors) const {
    // NOTE: This is gross - we're having to run the distance function again for potentially a lot
    // of samples! But I'm not sure how to do better under the constraints of OMPL's NN API, which
    // does not appear to provide a means of returning distances
    std::vector<double> distances;
    distances.reserve(neighbors.size());
    for (const auto& neighbor : neighbors) {
      distances.emplace_back(continuous_distance(motion, neighbor));
    }

    std::vector<int> indices(neighbors.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(), [&distances](const auto a, const auto b) -> bool {
      return distances[a] < distances[b];
    });

    // Apply the permutation
    permute(neighbors, indices);

    // Ensure we return only one of each state
    auto last = std::unique(neighbors.begin(), neighbors.end());
    neighbors.erase(last, neighbors.end());
  }

  void nearestK(const MotionT& motion, size_t k, std::vector<MotionT>& neighbors) const override {
    k                     = std::min(k, counter);
    const auto* cstate    = motion->getState()->template as<CompositeStateSpace::StateType>();
    const auto& mode      = cstate->get_mode();
    const auto mode_nn_it = local_nns.find(mode);
    neighbors.reserve(k);
    if (mode_nn_it != local_nns.end()) {
      mode_nn_it->second->nearestK(motion, cstate->transition ? k / 2 : k, neighbors);
    }

    if (cstate->transition) {
      const auto& transition_mode      = cstate->transition_mode;
      const auto transition_mode_nn_it = local_nns.find(transition_mode);
      if (transition_mode_nn_it != local_nns.end()) {
        std::vector<MotionT> transition_neighbors;
        transition_neighbors.reserve(k / 2);
        transition_mode_nn_it->second->nearestK(motion, k / 2, transition_neighbors);
        neighbors.insert(neighbors.end(), transition_neighbors.begin(), transition_neighbors.end());
      }
    }

    // NOTE: Assumes a sorted result from the local NN structure
    const auto& mode_entry_motions = entry_motions[mode];
    neighbors.reserve(neighbors.size() + mode_entry_motions.size());
    neighbors.insert(neighbors.end(), mode_entry_motions.begin(), mode_entry_motions.end());

    sort_neighbors_by_distance(motion, neighbors);

    // Drop all but the k closest
    // TODO: What do we do about neighbors that may be infinitely far away? Can this happen?
    if (neighbors.size() > k) {
      neighbors.resize(k);
    }
  }

  void
  nearestR(const MotionT& motion, double radius, std::vector<MotionT>& neighbors) const override {
    const auto* cstate    = motion->getState()->template as<CompositeStateSpace::StateType>();
    const auto& mode      = cstate->get_mode();
    const auto mode_nn_it = local_nns.find(mode);
    if (mode_nn_it != local_nns.end()) {
      mode_nn_it->second->nearestR(motion, radius, neighbors);
    }

    if (cstate->transition) {
      const auto& transition_mode      = cstate->transition_mode;
      const auto transition_mode_nn_it = local_nns.find(transition_mode);
      if (transition_mode_nn_it != local_nns.end()) {
        std::vector<MotionT> transition_neighbors;
        transition_mode_nn_it->second->nearestR(motion, radius, transition_neighbors);
        neighbors.insert(neighbors.end(), transition_neighbors.begin(), transition_neighbors.end());
      }
    }

    const auto& mode_entry_motions = entry_motions[mode];
    neighbors.reserve(neighbors.size() + mode_entry_motions.size());
    neighbors.insert(neighbors.end(), mode_entry_motions.begin(), mode_entry_motions.end());

    sort_neighbors_by_distance(motion, neighbors);
  }

  void
  set_continuous_distance(const std::function<double(const MotionT& a, const MotionT& b)>& cdist) {
    continuous_distance = cdist;
  }

 protected:
  mutable robin_hood::unordered_map<Mode, std::unique_ptr<NearestNeighborsT>> local_nns;
  mutable robin_hood::unordered_map<Mode, std::vector<MotionT>> entry_motions;
  std::function<double(const MotionT& a, const MotionT& b)> continuous_distance;
  const ModeAtlas* const mode_atlas;
  size_t counter;
};
}  // namespace planner
#endif
