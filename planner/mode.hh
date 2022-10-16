#ifndef MODE_HH
#define MODE_HH

#include <boost/container_hash/extensions.hpp>
#include <boost/dynamic_bitset.hpp>
#include <cstddef>
#include <ostream>

namespace planner {
struct Mode {
  using SymbolicStateT = boost::dynamic_bitset<>;
  using PoseSetT       = std::size_t;

  SymbolicStateT symbols;
  PoseSetT pose_set;
};

std::ostream& operator<<(std::ostream& os, const Mode& mode);

bool operator==(const Mode& mode_1, const Mode& mode_2);

std::size_t hash_value(const Mode& m);

}  // namespace planner

template <> struct std::hash<planner::Mode> {
  std::size_t operator()(const planner::Mode& p) const noexcept;
};
#endif
