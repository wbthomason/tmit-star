#ifndef MODE_ATLAS_HH
#define MODE_ATLAS_HH
#include <ompl/base/State.h>
#include <ompl/util/RandomNumbers.h>

#include <boost/dynamic_bitset.hpp>
#include <optional>
#include <ostream>
#include <utility>
#include <vector>

#include "mode.hh"
#include "robin_hood.h"
#include "scenegraph.hh"
#include "state.hh"
#include "symbolic.hh"
#include "task_plan.hh"

namespace planner {
struct Transition {
  Transition(const Mode::SymbolicStateT& from_symbolic_state,
             const Mode::SymbolicStateT& to_symbolic_state,
             const symbolic::Action* const action)
  : from_symbolic_state{from_symbolic_state}
  , to_symbolic_state{to_symbolic_state}
  , action{action}
  , states{}
  , successes{0}
  , failures{0}
  , steps{}
  , blocked_steps{} {}
  const Mode::SymbolicStateT from_symbolic_state;
  const Mode::SymbolicStateT to_symbolic_state;
  const symbolic::Action* action;
  std::vector<const ompl::base::State*> states;
  unsigned int successes;
  unsigned int failures;
  std::vector<unsigned int> steps;
  std::vector<unsigned int> blocked_steps;
};

struct SymbolicStateInfo {
  SymbolicStateInfo(const Mode::SymbolicStateT& signature)
  : signature{signature}, transitions{}, num_modes{0} {}
  const boost::dynamic_bitset<> signature;
  std::vector<std::unique_ptr<Transition>> transitions;
  unsigned int num_modes;
};

struct ModeInfo {
  ModeInfo(const Mode& mode, geometric::SceneGraph&& sg)
  : mode{mode}, sg{std::move(sg)}, entry_states{} {}
  bool reached{false};
  int reachable_mode_idx{-1};
  std::size_t num_samples{0};
  Mode mode;
  geometric::SceneGraph sg;
  std::vector<const ompl::base::State*> entry_states;
};

struct ModeAtlas {
  ModeAtlas(const symbolic::Domain& domain,
            const symbolic::Problem& problem,
            geometric::SceneGraph&& initial_sg);

  SymbolicStateInfo& get_symbolic_state_info(const Mode::SymbolicStateT& symbolic_state);
  ModeInfo& get_mode_info(const Mode& mode);
  Mode transition_succeeded(Transition* const transition,
                            const ompl::base::State* const state,
                            const std::vector<geometric::Transform3<>>& poses);
  void transition_failed(Transition* const transition);
  std::size_t num_known_modes() const;
  bool update_task_plan();
  void add_symbolic_state(const Mode::SymbolicStateT& symbolic_state);
  Mode add_mode(const Mode::SymbolicStateT& symbolic_state, geometric::SceneGraph&& sg);
  std::vector<Mode> reachable_modes;
  std::vector<bool> sampled_modes;
  std::size_t mode_dims = 0;

 protected:
  geometric::SceneGraph
  update_scenegraph_for_action(const geometric::SceneGraph& sg,
                               const symbolic::Action* const action,
                               const std::vector<geometric::Transform3<>>& poses);

  const Mode::SymbolicStateT start_symbolic_state;
  symbolic::PlannerState planner_state;
  // Mutable to let us use operator[]
  mutable robin_hood::unordered_map<Mode::SymbolicStateT, SymbolicStateInfo> symbolic_state_info;
  mutable robin_hood::unordered_map<Mode, ModeInfo> mode_info;
  std::vector<const symbolic::Action*> last_plan_actions;
  std::vector<const Transition*> last_plan_transitions;
  const std::size_t first_movable_object_idx;
};
};  // namespace planner
#endif
