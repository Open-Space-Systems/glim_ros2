#include <glim/util/trajectory_manager.hpp>
#include <stdexcept>
#include <iostream>
using glim::TrajectoryManager;
Eigen::Isometry3d pose(double x) {
  auto p=Eigen::Isometry3d::Identity(); p.translation().x()=x; return p;
}
void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
int main() {
  TrajectoryManager t;
  t.update_anchor(1,pose(10));  // Empty history must not dereference end().
  check(t.current_pose().isApprox(pose(0)),"empty trajectory changed");
  t.add_odom(1,pose(1)); t.add_odom(3,pose(3)); t.add_odom(2,pose(2));
  t.add_odom(2,pose(2.5));
  t.update_anchor(2,pose(12.5));
  check(t.get_T_world_odom().isApprox(pose(10)),"duplicate/reordered stamp broke anchor");
  t.update_anchor(4,pose(100)); t.update_anchor(.5,pose(100));
  check(t.get_T_world_odom().isApprox(pose(10)),"anchor extrapolated");
  t.update_anchor(2.5,pose(22.75));
  check(t.get_T_world_odom().isApprox(pose(20)),"anchor interpolation failed");
  auto global=pose(5);
  global.linear()=(Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitX()) *
                   Eigen::AngleAxisd(-.4,Eigen::Vector3d::UnitZ())).toRotationMatrix();
  t.update_anchor(3,global*pose(3));
  check(t.odom2world(pose(3)).isApprox(global*pose(3)),"full SE3 correction lost");
  t.add_odom(200,pose(200));
  t.update_anchor(1,pose(99));
  check(t.get_T_world_odom().isApprox(global),"expired anchor changed frame");
  t.update_anchor(global*pose(2),pose(2));
  check(t.get_T_world_odom().isApprox(global),"authoritative endpoint depends on expired history");
  std::cout << "trajectory contract tests passed\n";
}
