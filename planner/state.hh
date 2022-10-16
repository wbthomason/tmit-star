#ifndef STATE_HH
#define STATE_HH
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/dynamic_bitset.hpp>

namespace planner::symbolic {
using State = boost::dynamic_bitset<>;
};  // namespace planner::symbolic


namespace planner::geometric {
template <typename NumT = double> using Transform3 = Eigen::Transform<NumT, 3, Eigen::Isometry>;
template <typename NumT = double> struct State {
  NumT base_x;
  NumT base_y;
  NumT base_yaw;
  std::vector<NumT> joint_poses;
};

template <typename NumT> using Vector = Eigen::Matrix<NumT, 3, 1>;
template <typename NumT> Transform3<NumT> make_base_pose(const State<NumT>& state) {
  return Eigen::Translation<NumT, 3>(state.base_x, state.base_y, 0.0) *
         Eigen::AngleAxis<NumT>(state.base_yaw, Vector<NumT>::UnitZ());
}

template <typename NumT> void set_base_pose(const State<NumT>& state, Transform3<NumT>& base_pose) {
  auto& translation    = base_pose.translation();
  translation.x()      = state.base_x;
  translation.y()      = state.base_y;
  translation.z()      = state.base_z;
  base_pose.rotation() = Eigen::AngleAxis<NumT>(state.base_yaw, Vector<NumT>::UnitZ());
}
}  // namespace planner::geometric

namespace planner {
struct State {
  symbolic::State mode;
  geometric::State<> configuration;
};
}  // namespace planner
#endif
