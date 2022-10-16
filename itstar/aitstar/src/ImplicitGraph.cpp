/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Oxford
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the names of the copyright holders nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Authors: Marlin Strub, modified by Wil Thomason
#include "aitstar/ImplicitGraph.h"

#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <memory>
#include <stdexcept>

#include "collisions.hh"
#include "cspace.hh"
#include "mode_atlas.hh"
#include "mode_sampler.hh"
#include "ompl/util/GeometricEquations.h"
#include "optimization_objective.hh"
#include "sampler.hh"

namespace ompl {
namespace geometric {
  namespace aitstar {
    MultimodalImplicitGraph::MultimodalImplicitGraph(const ompl::base::Cost& solutionCost,
                                                     planner::ModeAtlas* mode_atlas,
                                                     unsigned int batch_budget)
    : batchId_(1u)
    , solutionCost_(solutionCost)
    , vertices_(mode_atlas)
    , mode_atlas(mode_atlas)
    , batch_budget(batch_budget) {}

    void MultimodalImplicitGraph::setup(const ompl::base::SpaceInformationPtr& spaceInformation,
                                        const ompl::base::ProblemDefinitionPtr& problemDefinition,
                                        ompl::base::PlannerInputStates* inputStates) {
      vertices_.setDistanceFunction(
      [this](const std::shared_ptr<Vertex>& a, const std::shared_ptr<Vertex>& b) {
        return spaceInformation_->distance(a->getState(), b->getState());
      });
      vertices_.set_continuous_distance(
      [this](const std::shared_ptr<Vertex>& a, const std::shared_ptr<Vertex>& b) {
        return spaceInformation_->getStateSpace()
        ->as<planner::CompositeStateSpace>()
        ->continuous_distance(a->getState()->as<planner::CompositeStateSpace::StateType>(),
                              b->getState()->as<planner::CompositeStateSpace::StateType>());
      });
      spaceInformation_  = spaceInformation;
      problemDefinition_ = problemDefinition;
      objective_         = problemDefinition->getOptimizationObjective();
      k_rgg_             = boost::math::constants::e<double>() +
               (boost::math::constants::e<double>() / spaceInformation->getStateDimension());
      updateStartAndGoalStates(ompl::base::plannerAlwaysTerminatingCondition(), inputStates);
      mode_sampler = std::make_unique<planner::ModeSampler>(
      spaceInformation_,
      problemDefinition_,
      sampler_,
      solutionCost_,
      [this](std::size_t num_states) -> double { return computeConnectionRadius(num_states); },
      batchId_,
      mode_atlas,
      numSampledStates_,
      numValidSamples_,
      batch_budget);
    }

    void MultimodalImplicitGraph::clear() {
      batchId_      = 1u;
      radius_       = std::numeric_limits<double>::infinity();
      numNeighbors_ = std::numeric_limits<std::size_t>::max();
      vertices_.clear();
      startVertices_.clear();
      goalVertices_.clear();
      prunedStartVertices_.clear();
      prunedGoalVertices_.clear();
      numSampledStates_ = 0u;
      numValidSamples_  = 0u;
    }

    void MultimodalImplicitGraph::setRewireFactor(double rewireFactor) {
      rewireFactor_ = rewireFactor;
    }

    double MultimodalImplicitGraph::getRewireFactor() const { return rewireFactor_; }

    void MultimodalImplicitGraph::setMaxNumberOfGoals(unsigned int maxNumberOfGoals) {
      maxNumGoals_ = maxNumberOfGoals;
    }

    unsigned int MultimodalImplicitGraph::getMaxNumberOfGoals() const { return maxNumGoals_; }

    void MultimodalImplicitGraph::setUseKNearest(bool useKNearest) { useKNearest_ = useKNearest; }

    bool MultimodalImplicitGraph::getUseKNearest() const { return useKNearest_; }

    void MultimodalImplicitGraph::registerStartState(const ompl::base::State* const startState) {
      // Create a vertex corresponding to this state.
      auto startVertex = std::make_shared<Vertex>(spaceInformation_, problemDefinition_, batchId_);

      // Copy the state into the vertex's state.
      spaceInformation_->copyState(startVertex->getState(), startState);

      // By definition, this has identity cost-to-come.
      startVertex->setCostToComeFromStart(objective_->identityCost());

      // Add the start vertex to the set of vertices.
      vertices_.add(startVertex);

      // Remember it as a start vertex.
      startVertices_.emplace_back(startVertex);
    }

    void MultimodalImplicitGraph::registerGoalState(const ompl::base::State* const goalState) {
      // Create a vertex corresponding to this state.
      auto goalVertex = std::make_shared<Vertex>(spaceInformation_, problemDefinition_, batchId_);

      // Copy the state into the vertex's state.
      spaceInformation_->copyState(goalVertex->getState(), goalState);

      // Add the goal vertex to the set of vertices.
      vertices_.add(goalVertex);

      // Remember it as a goal vertex.
      goalVertices_.emplace_back(goalVertex);
    }

    bool MultimodalImplicitGraph::hasAStartState() const { return !startVertices_.empty(); }

    bool MultimodalImplicitGraph::hasAGoalState() const { return !goalVertices_.empty(); }

    void MultimodalImplicitGraph::updateStartAndGoalStates(
    const ompl::base::PlannerTerminationCondition& terminationCondition,
    ompl::base::PlannerInputStates* inputStates) {
      // We need to keep track whether a new goal and/or a new start has been added.
      bool addedNewGoalState  = false;
      bool addedNewStartState = false;

      // First update the goals
      if (reached_goal_mode) {
        if (goalVertices_.size() < maxNumGoals_) {
          // NOTE: Removed the terminationCondition check here because it had unexpected values.
          while (newGoalSamples_.size() > 0) {
            // Get a new goal. If there are none, or the underlying state is invalid this will be a
            // nullptr.
            auto newGoalState = newGoalSamples_.back();
            newGoalSamples_.pop_back();

            // If there was a new valid goal, register it as such and remember that a goal has been
            // added.
            registerGoalState(newGoalState);
            addedNewGoalState = true;
          }
        }
      }

      // Having updated the goals, we now update the starts.
      while (inputStates->haveMoreStartStates()) {
        // Get the next start. The returned pointer can be a nullptr (if the state is invalid).
        auto newStartState = inputStates->nextStart();

        // If there is a new valid start, register it as such and remember that a start has been
        // added.
        if (static_cast<bool>(newStartState)) {
          registerStartState(newStartState);
          addedNewStartState = true;
        }
      }

      // If we added a new start and have previously pruned goals, we might want to add the goals
      // back.
      if (addedNewStartState && !prunedGoalVertices_.empty()) {
        // Keep track of the pruned goal vertices that have been revived.
        std::vector<std::vector<std::shared_ptr<Vertex>>::iterator> revivedGoals;

        // Let's see if the pruned goal is close enough to any new start to revive it..
        for (auto it = prunedGoalVertices_.begin(); it != prunedGoalVertices_.end(); ++it) {
          // Loop over all start states to get the best cost.
          auto heuristicCost = objective_->infiniteCost();
          for (const auto& start : startVertices_) {
            heuristicCost = objective_->betterCost(
            heuristicCost, objective_->motionCostHeuristic(start->getState(), (*it)->getState()));
          }

          // If this goal can possibly improve the current solution, add it back to the graph.
          if (objective_->isCostBetterThan(heuristicCost, solutionCost_)) {
            registerGoalState((*it)->getState());
            addedNewGoalState = true;
            revivedGoals.emplace_back(it);
          }
        }

        // Remove all revived goals from the pruned goals.
        for (const auto& revivedGoal : revivedGoals) {
          std::iter_swap(revivedGoal, prunedGoalVertices_.rbegin());
          prunedGoalVertices_.pop_back();
        }
      }

      // If we added a new goal and have previously pruned starts, we might want to add the starts
      // back.
      if (addedNewGoalState && !prunedStartVertices_.empty()) {
        // Keep track of the pruned goal vertices that have been revived.
        std::vector<std::vector<std::shared_ptr<Vertex>>::iterator> revivedStarts;

        // Let's see if the pruned start is close enough to any new goal to revive it..
        for (auto it = prunedStartVertices_.begin(); it != prunedStartVertices_.end(); ++it) {
          // Loop over all start states to get the best cost.
          auto heuristicCost = objective_->infiniteCost();
          for (const auto& goal : goalVertices_) {
            heuristicCost = objective_->betterCost(
            heuristicCost, objective_->motionCostHeuristic(goal->getState(), (*it)->getState()));
          }

          // If this goal can possibly improve the current solution, add it back to the graph.
          if (objective_->isCostBetterThan(heuristicCost, solutionCost_)) {
            registerStartState((*it)->getState());
            addedNewStartState = true;
            revivedStarts.emplace_back(it);
          }
        }

        // Remove all revived goals from the pruned goals.
        for (const auto& revivedStart : revivedStarts) {
          std::iter_swap(revivedStart, prunedStartVertices_.rbegin());
          prunedStartVertices_.pop_back();
        }
      }

      if (addedNewGoalState || addedNewStartState) {
        // Allocate a state sampler if we have at least one start state
        if (!startVertices_.empty() && sampler_ == nullptr) {
          sampler_ =
          std::make_shared<planner::UninformedSampler>(problemDefinition_,
                                                       std::numeric_limits<unsigned int>::max());
        }
      }

      if (!goalVertices_.empty() && startVertices_.empty()) {
        OMPL_WARN(
        "AIT* (ImplicitGraph): The problem has a goal but not a start. AIT* can not find a "
        "solution since PlannerInputStates provides no method to wait for a valid start state to "
        "appear.");
      }
    }

    std::size_t MultimodalImplicitGraph::computeNumberOfSamplesInInformedSet() const {
      // Loop over all vertices and count the ones in the informed set.
      std::size_t numberOfSamplesInInformedSet{0u};
      for (const auto& vertex : getVertices()) {
        // Get the best cost to come from any start.
        auto costToCome = objective_->infiniteCost();
        for (const auto& start : startVertices_) {
          costToCome = objective_->betterCost(
          costToCome, objective_->motionCostHeuristic(start->getState(), vertex->getState()));
        }

        // Get the best cost to go to any goal.
        auto costToGo = objective_->infiniteCost();
        for (const auto& goal : goalVertices_) {
          costToGo = objective_->betterCost(
          costToCome, objective_->motionCostHeuristic(vertex->getState(), goal->getState()));
        }

        // If this can possibly improve the current solution, it is in the informed set.
        if (objective_->isCostBetterThan(objective_->combineCosts(costToCome, costToGo),
                                         solutionCost_)) {
          ++numberOfSamplesInInformedSet;
        }
      }

      return numberOfSamplesInInformedSet;
    }

    bool MultimodalImplicitGraph::addSamples(
    std::size_t numNewSamples,
    const ompl::base::PlannerTerminationCondition& terminationCondition) {
      // If there are no states to be added, then there's nothing to do.
      if (numNewSamples == 0u) {
        return true;
      }

      newGoalSamples_.clear();
      const auto result = mode_sampler->add_samples<Vertex>(
      numNewSamples, terminationCondition, newSamples_, newGoalSamples_);

      if (!newGoalSamples_.empty()) {
        reached_goal_mode = true;
      }

      if (result) {
        // First get the number of samples inside the informed set.
        auto numSamplesInInformedSet = computeNumberOfSamplesInInformedSet();

        if (useKNearest_) {
          numNeighbors_ = computeNumberOfNeighbors(numSamplesInInformedSet + numNewSamples -
                                                   startVertices_.size() - goalVertices_.size());
        } else {
          radius_ = computeConnectionRadius(numSamplesInInformedSet + numNewSamples -
                                            startVertices_.size() - goalVertices_.size());
        }
      }

      // Add all new vertices to the nearest neighbor structure.
      vertices_.add(newSamples_);
      newSamples_.clear();

      // Update the batch id.
      // ++batchId_;

      return result;
    }

    std::size_t MultimodalImplicitGraph::getNumVertices() const { return vertices_.size(); }

    double MultimodalImplicitGraph::getConnectionRadius() const { return radius_; }

    std::vector<std::shared_ptr<Vertex>>
    MultimodalImplicitGraph::getNeighbors(const std::shared_ptr<Vertex>& vertex) const {
      // Return cached neighbors if available.
      if (vertex->hasCachedNeighbors()) {
        return vertex->getNeighbors();
      } else {
        ++numNearestNeighborsCalls_;
        std::vector<std::shared_ptr<Vertex>> neighbors{};
        if (useKNearest_) {
          vertices_.nearestK(vertex, numNeighbors_, neighbors);
        } else {
          vertices_.nearestR(vertex, radius_, neighbors);
        }

        vertex->cacheNeighbors(neighbors);
        return neighbors;
      }
    }

    bool MultimodalImplicitGraph::isStart(const std::shared_ptr<Vertex>& vertex) const {
      for (const auto& start : startVertices_) {
        if (vertex->getId() == start->getId()) {
          return true;
        }
      }
      return false;
    }

    bool MultimodalImplicitGraph::isGoal(const std::shared_ptr<Vertex>& vertex) const {
      for (const auto& goal : goalVertices_) {
        if (vertex->getId() == goal->getId()) {
          return true;
        }
      }
      return false;
    }

    const std::vector<std::shared_ptr<Vertex>>& MultimodalImplicitGraph::getStartVertices() const {
      return startVertices_;
    }

    const std::vector<std::shared_ptr<Vertex>>& MultimodalImplicitGraph::getGoalVertices() const {
      return goalVertices_;
    }

    std::vector<std::shared_ptr<Vertex>> MultimodalImplicitGraph::getVertices() const {
      std::vector<std::shared_ptr<Vertex>> vertices;
      vertices_.list(vertices);
      return vertices;
    }

    void MultimodalImplicitGraph::prune() {
      if (!objective_->isFinite(solutionCost_)) {
        return;
      }

      std::vector<std::shared_ptr<Vertex>> vertices;
      vertices_.list(vertices);

      // Prepare the vector of vertices to be pruned.
      std::vector<std::shared_ptr<Vertex>> verticesToBePruned;

      // Check each vertex whether it can be pruned.
      for (const auto& vertex : vertices) {
        // Check if the combination of the admissible costToCome and costToGo estimates results in a
        // path that is more expensive than the current solution.
        if (!canPossiblyImproveSolution(vertex)) {
          // We keep track of pruned start and goal vertices. This is because if the user adds start
          // or goal states after we have pruned start or goal states, we might want to reconsider
          // pruned start or goal states.
          if (isGoal(vertex)) {
            prunedGoalVertices_.emplace_back(vertex);
          } else if (isStart(vertex)) {
            prunedStartVertices_.emplace_back(vertex);
          }
          verticesToBePruned.emplace_back(vertex);
        }
      }

      // Remove all vertices to be pruned.
      for (const auto& vertex : verticesToBePruned) {
        // Remove it from both search trees.
        if (vertex->hasReverseParent()) {
          vertex->getReverseParent()->removeFromReverseChildren(vertex->getId());
          vertex->resetReverseParent();
        }
        vertex->invalidateReverseBranch();
        if (vertex->hasForwardParent()) {
          vertex->getForwardParent()->removeFromForwardChildren(vertex->getId());
          vertex->resetForwardParent();
        }
        vertex->invalidateForwardBranch();

        // Remove it from the nearest neighbor struct.
        vertices_.remove(vertex);
      }

      // Assert that the forward and reverse queue are empty?
    }

    std::size_t MultimodalImplicitGraph::getNumberOfSampledStates() const {
      return numSampledStates_;
    }

    std::size_t MultimodalImplicitGraph::getNumberOfValidSamples() const {
      return numValidSamples_;
    }

    std::size_t MultimodalImplicitGraph::getNumberOfStateCollisionChecks() const {
      // Each sampled state is checked for collision. Only sampled states are checked for collision
      // (number of collision checked edges don't count here.)
      return numSampledStates_;
    }

    std::size_t MultimodalImplicitGraph::getNumberOfNearestNeighborCalls() const {
      return numNearestNeighborsCalls_;
    }

    double MultimodalImplicitGraph::computeConnectionRadius(std::size_t numSamples) const {
      // Define the dimension as a helper variable.
      const auto dimension = static_cast<double>(spaceInformation_->getStateDimension());

      // // Compute the RRT* factor.
      // return rewireFactor_ *
      //        std::pow(2.0 * (1.0 + 1.0 / dimension) *
      //                     (sampler_->getInformedMeasure(solutionCost_) /
      //                      unitNBallMeasure(spaceInformation_->getStateDimension())) *
      //                     (std::log(static_cast<double>(numSamples)) /
      //                     static_cast<double>(numSamples)),
      //                 1.0 / dimension);

      // // Compute the FMT* factor.
      // return 2.0 * rewireFactor_ *
      //        std::pow((1.0 / dimension) *
      //                     (sampler_->getInformedMeasure(solutionCost_) /
      //                      unitNBallMeasure(spaceInformation_->getStateDimension())) *
      //                     (std::log(static_cast<double>(numSamples)) / numSamples),
      //                 1.0 / dimension);

      // PRM*
      return rewireFactor_ * 2.0 *
             std::pow((1.0 + 1.0 / dimension) *
                      (sampler_->getInformedMeasure(solutionCost_) /
                       unitNBallMeasure(spaceInformation_->getStateDimension())) *
                      (std::log(static_cast<double>(numSamples)) / numSamples),
                      1.0 / dimension);
    }

    std::size_t MultimodalImplicitGraph::computeNumberOfNeighbors(std::size_t numSamples) const {
      return std::ceil(rewireFactor_ * k_rgg_ * std::log(static_cast<double>(numSamples)));
    }

    bool MultimodalImplicitGraph::canPossiblyImproveSolution(
    const std::shared_ptr<Vertex>& vertex) const {
      // Get the preferred start for this vertex.
      auto bestCostToCome = objective_->infiniteCost();
      for (const auto& start : startVertices_) {
        auto costToCome = objective_->motionCostHeuristic(start->getState(), vertex->getState());
        if (objective_->isCostBetterThan(costToCome, bestCostToCome)) {
          bestCostToCome = costToCome;
        }
      }

      // Check if the combination of the admissible costToCome and costToGo estimates results in a
      // path that is more expensive than the current solution.
      return objective_->isCostBetterThan(
      objective_->combineCosts(bestCostToCome,
                               objective_->costToGo(vertex->getState(),
                                                    problemDefinition_->getGoal().get())),
      solutionCost_);
    }

  }  // namespace aitstar

}  // namespace geometric

}  // namespace ompl
