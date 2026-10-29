#pragma once

#include <deque>
#include <mutex>
#include <memory>
#include <string>
#include <map>
#include <set>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <glim/util/extension_module_ros2.hpp>

namespace gtsam {
class Values;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace gtsam_points {
class IncrementalFixedLagSmootherExtWithFallback;
}  // namespace gtsam_points

namespace spdlog {
class logger;
}

namespace glim {

/**
 * @brief Absolute-position constraint module.
 *
 * Fuses an external absolute-position measurement (synthetic RTK / surface
 * beacon, see the synthetic_rtk package) into GLIM's odometry fixed-lag
 * smoother as a position-only prior (gtsam::GPSFactor) on the live IMU pose
 * key X(current), bounding the catastrophic pose jumps that pure LiDAR-IMU
 * odometry suffers in feature-degenerate lunar terrain.
 *
 * Assumes GLIM's world frame is pinned to the real/measurement world frame via
 * odometry_estimation.init_T_world_imu, so the measurement is a direct prior.
 *
 * Architecture: the ROS subscription (executor thread) buffers measurements;
 * the on_smoother_update callback (odometry thread) drains the buffer, matches
 * by timestamp to the newly created pose key, gates outliers, and appends a
 * robust GPSFactor. Dropout => no factor. This is the AbsolutePositionProvider;
 * future multi-robot range/relative providers plug into the same hook.
 */
class AbsConstraintModule : public ExtensionModuleROS2 {
public:
  AbsConstraintModule();
  ~AbsConstraintModule();

  virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node& node) override;

private:
  struct Sample {
    double stamp;          // measurement time (sec)
    Eigen::Vector3d pos;   // world-frame position
    Eigen::Vector3d sigma; // per-axis std-dev (from msg covariance)
    double yaw;            // absolute heading of the pose key's frame (rad)
    double yaw_sigma;      // heading std-dev (from msg yaw covariance)
    bool has_yaw;          // true if the message carried a usable heading
  };

  void set_callbacks();
  void rtk_callback(const std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped>& msg);
  void on_smoother_update(
    gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
    gtsam::NonlinearFactorGraph& new_factors,
    gtsam::Values& new_values,
    std::map<std::uint64_t, double>& new_stamps);

  std::shared_ptr<spdlog::logger> logger;

  // Config
  bool enabled;
  int smoother_callback_id{-1};
  std::string rtk_topic;
  double assoc_tolerance;   // s, max |stamp - frame_stamp| to associate
  bool require_closed_window; // opt-in causal association on a strictly ordered source
  double latest_ordered_stamp{-1.0};
  double gate_dist;         // m, reject if |meas - predicted| exceeds (<=0 disables)
  double huber_k;           // robust kernel threshold (<=0 disables robustification)
  double sigma_floor;       // m, lower bound on per-axis sigma
  bool use_msg_covariance;  // use covariance from msg; else use sigma_floor isotropically
  bool use_yaw;             // fuse absolute heading (GNSS-compass / dual-antenna) when present
  double yaw_huber_k;       // robust kernel for the yaw prior (<=0 disables; default off so a
                            // trustworthy heading can bootstrap-correct a wrong frame yaw)
  double yaw_sigma_floor;   // rad, lower bound on heading std-dev
  double yaw_cov_threshold; // rad^2, msg yaw variance above this = "heading not measured"

  // Measurement buffer (shared between ROS thread and odometry thread)
  mutable std::mutex buffer_mutex;
  std::deque<Sample> buffer;
  std::map<std::uint64_t, double> live_pose_keys;
  std::set<std::uint64_t> constrained_keys;
  std::set<double> consumed_stamps;

  // Diagnostics
  long n_msgs;
  long n_added;
  long n_yaw_added;
  long n_no_match;
  long n_gated;
  double last_log_stamp;
  double last_added_dist;  // |meas - predicted| of the most recent added factor
};

}  // namespace glim
