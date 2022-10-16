#include "mode.hh"

#include <iostream>

namespace planner {
std::ostream& operator<<(std::ostream& os, const Mode& mode) {
  os << "(symbols: " << mode.symbols << "\tposes: " << mode.pose_set << ")";
  return os;
}

bool operator==(const Mode& mode_1, const Mode& mode_2) {
  return mode_1.symbols == mode_2.symbols && mode_1.pose_set == mode_2.pose_set;
}

std::size_t hash_value(const Mode& m) {
  std::size_t seed = 0;
  boost::hash_combine(seed, m.symbols);
  boost::hash_combine(seed, m.pose_set);
  return seed;
}
}  // namespace planner

std::size_t std::hash<planner::Mode>::operator()(const planner::Mode& p) const noexcept {
  std::size_t seed = 0;
  boost::hash_combine(seed, p.symbols);
  boost::hash_combine(seed, p.pose_set);
  return seed;
}
