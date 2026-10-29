#include "ros2_bot_estimator_lidar_vshape/tracker.hpp"
#include "ros2_bot_estimator_lidar_vshape/output.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <optional>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
using PoseMessage = geometry_msgs::msg::PoseWithCovarianceStamped;

class VShapeNode : public rclcpp::Node {
 public:
  VShapeNode() : Node("vshape_estimator") {
    vshape::Config c;
#define PARAM(name) c.name = parameter(#name, c.name)
    PARAM(arm_length); PARAM(opening_angle_deg);
    PARAM(association_distance); PARAM(endpoint_margin); PARAM(min_support);
    PARAM(target_x); PARAM(target_y); PARAM(target_yaw);
    PARAM(target_position_tolerance); PARAM(target_yaw_tolerance);
    PARAM(min_points); PARAM(max_points_per_arm);
#undef PARAM
#define IEKF(field, name) c.field = parameter(name, c.field)
    IEKF(range_sigma, "iekf_measurement_noise_pos");
    IEKF(angle_sigma, "iekf_measurement_noise_angle");
    IEKF(model_sigma, "iekf_arm_model_sigma");
    IEKF(odom_timeout, "iekf_odom_extrapolation_max_dt");
    IEKF(unknown_linear, "iekf_odom_gap_linear_noise_density");
    IEKF(unknown_angular, "iekf_odom_gap_angular_noise_density");
    IEKF(gate_sigma, "iekf_chi2_gate_sigma");
    IEKF(huber_delta, "iekf_huber_threshold");
    IEKF(history_sec, "iekf_state_history_duration");
    IEKF(init_position_sigma, "iekf_initial_position_sigma");
    IEKF(init_yaw_sigma, "iekf_initial_yaw_sigma");
    IEKF(max_iterations, "iekf_max_iterations");
#undef IEKF
    // Match wall/reflector's Q convention: ROS parameters are covariance
    // densities; the standalone core API stores their square-root densities.
    const double q_lin = parameter("iekf_process_noise_lin", c.q_linear * c.q_linear);
    const double q_ang = parameter("iekf_process_noise_ang", c.q_angular * c.q_angular);
    if (!std::isfinite(q_lin) || !std::isfinite(q_ang) || q_lin <= 0 || q_ang <= 0)
      throw std::invalid_argument("iekf_process_noise_lin/ang must be positive covariance densities");
    c.q_linear = std::sqrt(q_lin); c.q_angular = std::sqrt(q_ang);
    c.max_events = boundedSize("iekf_state_history_size", c.max_events, 100000);
    c.max_scan_points = boundedSize("max_scan_points", c.max_scan_points, 200000);
    c.lidar_in_base = vshape::SE2(parameter("lidar_in_base.x", 0.0),
                                parameter("lidar_in_base.y", 0.0),
                                parameter("lidar_in_base.yaw", 0.0));
    c.validate();
    face_depth_ = c.arm_length * std::cos(c.opening_angle_deg * 3.141592653589793 / 360.);
    tracker_ = std::make_unique<vshape::Tracker>(c);
    base_frame_ = parameter<std::string>("base_frame", "base_link");
    lidar_frame_ = parameter<std::string>("lidar_frame", "laser");
    marker_height_ = parameter("marker_height", .2);
    if (base_frame_.empty() || lidar_frame_.empty() || !std::isfinite(marker_height_) || marker_height_ < 0) {
      throw std::invalid_argument("base/lidar frames and a finite nonnegative marker_height are required");
    }
    const auto scan_topic = parameter<std::string>("scan_topic", "/scan");
    const auto odom_topic = parameter<std::string>("iekf_odom_topic", "/bot_sensor/odometer/odometry");
    const double rate = parameter("output_rate", 30.0);
    measurement_timeout_ = parameter("iekf_no_measurement_timeout_sec", 0.5);
    if (!std::isfinite(rate) || rate < 1.0 || rate > 200.0 ||
        !std::isfinite(measurement_timeout_) || measurement_timeout_ <= 0) {
      throw std::invalid_argument("invalid output_rate or iekf_no_measurement_timeout_sec");
    }
    pose_pub_ = create_publisher<PoseMessage>("/vshape/pose", 10);
    deskew_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/vshape/deskewed_points", rclcpp::SensorDataQoS());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/vshape/diagnostics", 10);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr message) { scan(*message); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) { odometry(*message); });
    reset_service_ = create_service<std_srvs::srv::Trigger>(
        "/vshape/reset", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          reset("reset requested");
          response->success = true;
          response->message = "V marker tracker reset";
        });
    // A wall timer also emits health diagnostics when simulation time is paused.
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate)), [this]() { publish(); });
    rcl_jump_threshold_t threshold{};
    threshold.on_clock_change = true;
    threshold.min_backward.nanoseconds = -1;
    clock_jump_ = get_clock()->create_jump_callback(nullptr,
        [this](const rcl_time_jump_t& jump) {
          if (jump.delta.nanoseconds < 0 || jump.clock_change == RCL_ROS_TIME_ACTIVATED ||
              jump.clock_change == RCL_ROS_TIME_DEACTIVATED) clock_reset_pending_ = true;
        }, threshold);
    RCLCPP_INFO(get_logger(), "V marker tracker: scan %s (%s), body twist %s (%s)",
                scan_topic.c_str(), lidar_frame_.c_str(), odom_topic.c_str(), base_frame_.c_str());
  }

 private:
  template<class T> T parameter(const std::string& name, const T& value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = "Startup configuration; restart node to change";
    return declare_parameter<T>(name, value, descriptor);
  }

  std::size_t boundedSize(const std::string& name, std::size_t value, std::int64_t limit) {
    const auto result = parameter<std::int64_t>(name, static_cast<std::int64_t>(value));
    if (result < 1 || result > limit) throw std::invalid_argument(name + " out of bounded range");
    return static_cast<std::size_t>(result);
  }

  void reset(const std::string& reason) {
    tracker_->reset();
    convention_.reset(); last_scan_.reset(); deskew_status_ = "waiting for scan";
    deskew_points_ = 0; scan_duration_ = 0;
    input_reason_ = reason;
    last_now_ = now().seconds();
  }

  rclcpp::Time currentTime() {
    const auto stamp = now();
    if (clock_reset_pending_ || (std::isfinite(last_now_) && stamp.seconds() < last_now_ - 1e-9)) {
      clock_reset_pending_ = false;
      reset("ROS clock changed or rewound; tracker reset");
    }
    last_now_ = stamp.seconds();
    return stamp;
  }

  bool validStamp(const builtin_interfaces::msg::Time& stamp, double current) {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000u ||
        double(stamp.sec) + double(stamp.nanosec) * 1e-9 > current + 1e-9) {
      input_reason_ = "invalid or future sensor timestamp";
      return false;
    }
    return true;
  }

  void scan(const sensor_msgs::msg::LaserScan& message) {
    const double current = currentTime().seconds();
    if (message.header.frame_id != lidar_frame_) {
      input_reason_ = "scan frame mismatch: expected " + lidar_frame_;
      return;
    }
    if (!validStamp(message.header.stamp, current)) return;
    if (message.ranges.size() > tracker_->config().max_scan_points) {
      input_reason_ = "scan exceeds max_scan_points";
      return;
    }
    if (message.ranges.empty() || !std::isfinite(message.angle_min) ||
        !std::isfinite(message.angle_increment) || message.angle_increment == 0 ||
        !std::isfinite(message.time_increment) || message.time_increment < 0 ||
        !std::isfinite(message.range_min) || !std::isfinite(message.range_max) ||
        message.range_min < 0 || message.range_max <= message.range_min) {
      input_reason_ = "invalid scan metadata";
      return;
    }
    vshape::Scan s;
    s.stamp = rclcpp::Time(message.header.stamp).seconds();
    s.angle_min = message.angle_min;
    s.angle_increment = message.angle_increment;
    s.time_increment = message.time_increment;
    s.range_min = message.range_min;
    s.range_max = message.range_max;
    s.ranges.assign(message.ranges.begin(), message.ranges.end());
    input_reason_.clear();
    if (tracker_->addScan(s)) {
      last_scan_ = std::move(s);
      publishDeskewedScan();
    }
  }

  void publishDeskewedScan() {
    if (!last_scan_) return;
    vshape::Points points; std::string reason;
    scan_duration_ = last_scan_->time_increment * double(last_scan_->ranges.size() - 1);
    if (!tracker_->deskewScan(*last_scan_, points, reason)) {
      deskew_status_ = "rejected: " + reason; deskew_points_ = 0; return;
    }
    deskew_status_ = scan_duration_ > 0 ? "applied" : "instantaneous";
    deskew_points_ = points.size();
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = base_frame_;
    cloud.header.stamp = rclcpp::Time(static_cast<int64_t>(std::llround(last_scan_->stamp * 1e9)), RCL_ROS_TIME);
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz"); modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (const auto& point : points) {
      *x = float(point.q.x()); *y = float(point.q.y()); *z = float(marker_height_); ++x; ++y; ++z;
    }
    cloud.is_dense = true;
    deskew_pub_->publish(cloud);
  }

  void odometry(const nav_msgs::msg::Odometry& message) {
    const double current = currentTime().seconds();
    if (message.child_frame_id != base_frame_) {
      input_reason_ = "odometry twist frame mismatch: expected child_frame_id " + base_frame_;
      return;
    }
    if (!validStamp(message.header.stamp, current)) return;
    vshape::Odom o;
    o.stamp = rclcpp::Time(message.header.stamp).seconds();
    o.twist = {message.twist.twist.linear.x, message.twist.twist.linear.y,
               message.twist.twist.angular.z};
    if (!o.twist.allFinite()) {
      input_reason_ = "nonfinite odometry body twist";
      return;
    }
    if (tracker_->addOdometry(o) && last_scan_ &&
        o.stamp <= last_scan_->stamp + scan_duration_ + 1e-9)
      publishDeskewedScan();
  }

  static PoseMessage pose(const rclcpp::Time& stamp, const std::string& frame,
                          const vshape::SE2& transform, const vshape::Mat3& covariance, double height) {
    PoseMessage message;
    message.header.stamp = stamp;
    message.header.frame_id = frame;
    message.pose.pose.position.x = transform.p.x();
    message.pose.pose.position.y = transform.p.y();
    message.pose.pose.position.z = height;
    message.pose.pose.orientation.z = std::sin(transform.yaw / 2);
    message.pose.pose.orientation.w = std::cos(transform.yaw / 2);
    constexpr int index[3] = {0, 1, 5};
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 3; ++col)
        message.pose.covariance[index[row] * 6 + index[col]] = covariance(row, col);
    for (int unsupported : {2, 3, 4}) message.pose.covariance[unsupported * 6 + unsupported] = 1e6;
    return message;
  }

  static std::string number(double value) {
    std::ostringstream stream;
    stream << std::setprecision(10) << value;
    return stream.str();
  }

  void publish() {
    const auto stamp = currentTime();
    const double current = stamp.seconds();
    const auto state = tracker_->estimate(current);
    if (state.initialized) {
      const auto geometric = state.robot_in_marker.inverse();
      const auto target = convention_.apply(vshape::openingFacePose(geometric, face_depth_));
      const auto covariance = vshape::openingFaceCovariance(geometric,
          vshape::inversePoseCovariance(state.robot_in_marker, state.covariance), face_depth_);
      pose_pub_->publish(pose(stamp, base_frame_, target,
          covariance, marker_height_));
    }
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus diagnostic;
    diagnostic.name = "vshape_estimator/tracking";
    diagnostic.hardware_id = lidar_frame_;
    const auto& d = state.diagnostics;
    const double age = std::max(0.0, current - d.last_measurement);
    const bool stale_odom = state.initialized && d.odom_age > tracker_->config().odom_timeout;
    const bool stale_scan = state.initialized && age > measurement_timeout_;
    const std::string mode = !state.initialized ? "UNINITIALIZED" : stale_odom ?
        "ODOMETRY_STALE" : stale_scan ? "PROPAGATION_ONLY" : d.mode;
    diagnostic.level = mode == "TRACKING" && input_reason_.empty() ?
        diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diagnostic.message = input_reason_.empty() ? (stale_odom ? "odometry stale: motion unknown" :
        stale_scan ? "prediction only: no recent V marker measurement" : d.reason) : input_reason_;
    auto add = [&diagnostic](const std::string& key, const std::string& value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key; item.value = value; diagnostic.values.push_back(item);
    };
    add("mode", mode); add("reason", diagnostic.message);
    add("last_measurement_age", number(age)); add("odom_age", number(d.odom_age));
    add("deskew_status", deskew_status_); add("scan_duration", number(scan_duration_));
    add("deskew_points", std::to_string(deskew_points_)); add("marker_height", number(marker_height_));
    add("output_convention", "V opening-face midpoint in base; initial heading away from AMR");
    add("opening_face_depth", number(face_depth_));
    add("nis", number(d.nis)); add("dof", std::to_string(d.dof));
    add("points_arm_0", std::to_string(d.points[0])); add("points_arm_1", std::to_string(d.points[1]));
    add("updates", std::to_string(d.updates)); add("rejected", std::to_string(d.rejected));
    add("replays", std::to_string(d.replays)); add("update_ms", number(d.update_ms));
    add("processing_ms", number(d.processing_ms)); add("candidate_score", number(d.candidate_score));
    add("support_fraction", number(d.support_fraction)); add("normal_condition", number(d.normal_condition));
    for (int i = 0; i < 3; ++i)
      add("information_eigenvalue_" + std::to_string(i), number(d.information_eigenvalues[i]));
    array.status.push_back(std::move(diagnostic));
    diagnostics_pub_->publish(array);
  }

  std::unique_ptr<vshape::Tracker> tracker_;
  vshape::DockingConvention convention_;
  std::optional<vshape::Scan> last_scan_;
  std::string base_frame_, lidar_frame_, input_reason_, deskew_status_ = "waiting for scan";
  double marker_height_ = .2, scan_duration_ = 0;
  double face_depth_ = 0.;
  std::size_t deskew_points_ = 0;
  double measurement_timeout_ = 0.5;
  double last_now_ = std::numeric_limits<double>::quiet_NaN();
  rclcpp::Publisher<PoseMessage>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskew_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool clock_reset_pending_ = false;
  rclcpp::JumpHandler::SharedPtr clock_jump_;
};
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    auto node = std::make_shared<VShapeNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("vshape_estimator"), "%s", error.what());
    code = 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return code;
}
