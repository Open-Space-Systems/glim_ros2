// Real ROS publisher/callback test: graph-corrected poses and point coordinates
// must agree, including when the historical endpoint changed in the smoother.
#include <glim_ros/rviz_viewer.hpp>
#include <glim/util/config.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/mapping/callbacks.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <cstring>
#include <stdexcept>

void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
Eigen::Isometry3d at(double x) {
  auto p = Eigen::Isometry3d::Identity(); p.translation().x() = x; return p;
}
double coordinate(const sensor_msgs::msg::PointCloud2& cloud, const std::string& name) {
  for (const auto& field : cloud.fields) if (field.name == name) {
    check(field.datatype == sensor_msgs::msg::PointField::FLOAT32, "unexpected point type");
    float value; std::memcpy(&value, cloud.data.data() + field.offset, sizeof(value)); return value;
  }
  throw std::runtime_error("coordinate missing");
}
Eigen::Isometry3d pose(const geometry_msgs::msg::Pose& p) {
  auto out = at(p.position.x); out.translation().y()=p.position.y; out.translation().z()=p.position.z;
  out.linear()=Eigen::Quaterniond(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z).toRotationMatrix();
  return out;
}
int main(int argc, char** argv) {
  check(argc > 1, "config path required");
  glim::GlobalConfig::instance(argv[1], true);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rviz_frame_contract");
  auto viewer = std::make_unique<glim::RvizViewer>();
  viewer->create_subscriptions(*node);
  geometry_msgs::msg::PoseStamped::SharedPtr current_pose;
  sensor_msgs::msg::PointCloud2::SharedPtr current_cloud;
  auto poses = node->create_subscription<geometry_msgs::msg::PoseStamped>("~/pose_corrected", 10,
    [&](geometry_msgs::msg::PoseStamped::SharedPtr p) { current_pose = p; });
  auto clouds = node->create_subscription<sensor_msgs::msg::PointCloud2>("~/aligned_points_corrected", 10,
    [&](sensor_msgs::msg::PointCloud2::SharedPtr p) { current_cloud = p; });
  auto points = std::make_shared<gtsam_points::PointCloudCPU>();
  points->points_storage = {Eigen::Vector4d(.25,.5,.75,1)};
  points->points = points->points_storage.data(); points->num_points = 1;
  auto frame = std::make_shared<glim::EstimationFrame>();
  frame->id=0; frame->stamp=1; frame->frame=points;
  frame->T_lidar_imu=at(.3); frame->T_world_imu=at(1);
  frame->T_world_lidar=frame->T_world_imu*frame->T_lidar_imu.inverse();
  frame->frame_id=glim::FrameID::IMU; frame->v_world_imu.setZero();
  glim::OdometryEstimationCallbacks::on_update_new_frame(frame);

  auto correction = at(5);
  correction.translation().z()=2;
  correction.linear()=(Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitX()) *
                       Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitZ())).toRotationMatrix();
  auto submap = std::make_shared<glim::SubMap>();
  auto endpoint=frame->clone(); endpoint->T_world_imu=at(2); // Later fixed-lag revision.
  submap->id=0; submap->odom_frames={endpoint}; submap->frame=points;
  submap->T_world_origin=correction*endpoint->T_world_imu;
  submap->T_origin_endpoint_R=Eigen::Isometry3d::Identity();
  glim::GlobalMappingCallbacks::on_update_submaps(std::vector<glim::SubMap::Ptr>{submap});
  // Repeated graph updates must not append duplicate submap geometry.
  glim::GlobalMappingCallbacks::on_update_submaps(std::vector<glim::SubMap::Ptr>{submap});
  for (auto kind : {glim::FrameID::IMU, glim::FrameID::LIDAR, glim::FrameID::WORLD}) {
    frame->frame_id=kind; frame->stamp+=1;
    current_pose.reset(); current_cloud.reset();
    auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(4);
    const auto received_current = [&] {
      return current_pose && current_cloud && current_pose->header.stamp.sec == frame->stamp &&
             current_cloud->header.stamp.sec == frame->stamp;
    };
    while (!received_current() && std::chrono::steady_clock::now()<deadline) {
      glim::OdometryEstimationCallbacks::on_update_new_frame(frame);
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    check(received_current(), "ROS pose/cloud publication missing");
    check(current_pose->header.frame_id=="map" && current_cloud->header.frame_id=="map", "wrong map frame");
    check(current_pose->header.stamp==current_cloud->header.stamp, "pose/cloud stamp mismatch");
    check(pose(current_pose->pose).isApprox(correction*frame->T_world_imu,1e-6), "stale endpoint used for correction");
    const Eigen::Vector4d expected=correction*frame->T_world_sensor()*points->points[0];
    const Eigen::Vector3d actual(coordinate(*current_cloud,"x"),coordinate(*current_cloud,"y"),coordinate(*current_cloud,"z"));
    check(actual.isApprox(expected.head<3>(),1e-6), "aligned cloud disagrees with global pose");
  }
  viewer.reset();
  check(!glim::OdometryEstimationCallbacks::on_update_new_frame, "destroyed viewer left a dangling callback");
  rclcpp::shutdown();
}
