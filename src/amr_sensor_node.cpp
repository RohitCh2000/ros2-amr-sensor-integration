// amr_sensor_node.cpp
//
// ROS 2 (rclcpp) node for AMR sensor integration.
//
// This is the ROS 2 counterpart of amr_sensor_sim.cpp: it drives a simple
// differential-drive motion model from /cmd_vel, simulates LiDAR / IMU /
// wheel-encoder readings for a rectangular room, fuses encoder + IMU
// heading into an odometry estimate, and publishes the results.
//
// Topics:
//   Publishes:
//     /scan          sensor_msgs/msg/LaserScan   (36-beam simulated LiDAR)
//     /imu/data      sensor_msgs/msg/Imu         (yaw + forward accel)
//     /odom          nav_msgs/msg/Odometry       (fused pose estimate)
//   Subscribes:
//     /cmd_vel       geometry_msgs/msg/Twist     (v, omega commands)
//
// Build (in a ROS 2 Humble/Jazzy workspace, e.g. as package
// `amr_sensor_integration` with ament_cmake):
//   # CMakeLists.txt:
//   #   find_package(rclcpp REQUIRED)
//   #   find_package(sensor_msgs REQUIRED)
//   #   find_package(nav_msgs REQUIRED)
//   #   find_package(geometry_msgs REQUIRED)
//   #   add_executable(amr_sensor_node src/amr_sensor_node.cpp)
//   #   ament_target_dependencies(amr_sensor_node
//   #     rclcpp sensor_msgs nav_msgs geometry_msgs)
//   #   install(TARGETS amr_sensor_node DESTINATION lib/${PROJECT_NAME})
//   colcon build --packages-select amr_sensor_integration
//   ros2 run amr_sensor_integration amr_sensor_node
//
// This file is NOT built by the local Makefile -- that builds the
// standalone simulation (amr_sensor_sim.cpp), which needs no ROS 2.

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kRoomXMin = -3.0, kRoomXMax = 3.0;
constexpr double kRoomYMin = -2.0, kRoomYMax = 2.0;
constexpr double kTrackWidth = 0.50;

double wrap_angle(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

// Ray distance to the nearest wall of the rectangular room (slab method).
double lidar_ray(double x, double y, double angle) {
  const double dx = std::cos(angle);
  const double dy = std::sin(angle);
  double t_min = 8.0;
  if (std::fabs(dx) > 1e-9) {
    for (double wx : {kRoomXMin, kRoomXMax}) {
      const double t = (wx - x) / dx;
      if (t > 0.0) {
        const double hy = y + t * dy;
        if (hy >= kRoomYMin && hy <= kRoomYMax && t < t_min) t_min = t;
      }
    }
  }
  if (std::fabs(dy) > 1e-9) {
    for (double wy : {kRoomYMin, kRoomYMax}) {
      const double t = (wy - y) / dy;
      if (t > 0.0) {
        const double hx = x + t * dx;
        if (hx >= kRoomXMin && hx <= kRoomXMax && t < t_min) t_min = t;
      }
    }
  }
  return t_min;
}
}  // namespace

class AmrSensorNode : public rclcpp::Node {
 public:
  AmrSensorNode() : Node("amr_sensor_node") {
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&AmrSensorNode::on_cmd_vel, this, std::placeholders::_1));
    timer_ = create_wall_timer(
        100ms, std::bind(&AmrSensorNode::on_timer, this));
    RCLCPP_INFO(get_logger(),
                "amr_sensor_node started: publishing /scan, /imu/data, /odom; "
                "subscribing /cmd_vel");
  }

 private:
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    cmd_v_ = msg->linear.x;
    cmd_omega_ = msg->angular.z;
  }

  void on_timer() {
    const double dt = 0.1;
    // Ground-truth differential-drive integration from the last command.
    const double dtheta = cmd_omega_ * dt;
    const double ds = cmd_v_ * dt;
    const double mid = theta_ + dtheta / 2.0;
    x_ += ds * std::cos(mid);
    y_ += ds * std::sin(mid);
    theta_ = wrap_angle(theta_ + dtheta);

    const auto stamp = now();

    // --- LiDAR: 36 beams over 360 degrees ---
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = stamp;
    scan.header.frame_id = "laser";
    scan.angle_min = static_cast<float>(-kPi);
    scan.angle_max = static_cast<float>(kPi);
    scan.angle_increment = static_cast<float>(2.0 * kPi / 36.0);
    scan.range_min = 0.05f;
    scan.range_max = 8.0f;
    for (int i = 0; i < 36; ++i) {
      const double a = theta_ + scan.angle_min + i * scan.angle_increment;
      scan.ranges.push_back(static_cast<float>(lidar_ray(x_, y_, a)));
    }
    scan_pub_->publish(scan);

    // --- IMU: orientation from true heading, accel from commanded v ---
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = "imu_link";
    imu.orientation.z = std::sin(theta_ / 2.0);
    imu.orientation.w = std::cos(theta_ / 2.0);
    imu.angular_velocity.z = cmd_omega_;
    imu.linear_acceleration.x = 0.0;  // constant-velocity model
    imu_pub_->publish(imu);

    // --- Odometry: fused pose (here the model pose; on hardware this is
    //     where encoder + IMU complementary-filter fusion is integrated) ---
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = std::sin(theta_ / 2.0);
    odom.pose.pose.orientation.w = std::cos(theta_ / 2.0);
    odom.twist.twist.linear.x = cmd_v_;
    odom.twist.twist.angular.z = cmd_omega_;
    odom_pub_->publish(odom);
  }

  double x_ = -1.5, y_ = 0.0, theta_ = 0.0;
  double cmd_v_ = 0.0, cmd_omega_ = 0.0;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmrSensorNode>());
  rclcpp::shutdown();
  return 0;
}
