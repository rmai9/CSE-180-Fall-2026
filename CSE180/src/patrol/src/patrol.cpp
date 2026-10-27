#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace rclcpp;

// Personal parameters: student ID ends in 5.
const double L = 1.05;
const double D_STOP = 0.50;

// Waypoints expressed in the fixed start frame S.
const double waypoint_s_x[4] = {L, L, 0.0, 0.0};
const double waypoint_s_y[4] = {0.0, L, L, 0.0};

// Waypoints expressed in odom, calculated once at startup.
double waypoint_odom_x[4];
double waypoint_odom_y[4];

// Recorded start pose.
double start_x = 0.0;
double start_y = 0.0;
double start_theta = 0.0;

bool start_saved = false;
bool scan_received = false;
bool obstacle_too_close = false;

int waypoint_index = 0;

Node::SharedPtr node;
Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  bool obstacle_now = false;

  for (float range : msg->ranges) {
    if (std::isfinite(range) &&
        range >= msg->range_min &&
        range <= msg->range_max &&
        range < D_STOP) {
      obstacle_now = true;
      break;
    }
  }

  if (obstacle_now && !obstacle_too_close) {
    RCLCPP_INFO(
      node->get_logger(),
      "Obstacle within %.2f m. Pausing patrol.",
      D_STOP);
  } else if (!obstacle_now && obstacle_too_close) {
    RCLCPP_INFO(
      node->get_logger(),
      "Obstacle clear. Resuming patrol.");
  }

  obstacle_too_close = obstacle_now;
  scan_received = true;

  // Send a stop immediately, rather than waiting for the next odom.
  if (obstacle_too_close) {
    geometry_msgs::msg::Twist stop;
    pub_->publish(stop);
  }
}

void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double robot_x = msg->pose.pose.position.x;
  double robot_y = msg->pose.pose.position.y;

  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;

  double theta = std::atan2(
    2.0 * (qw * qz + qx * qy),
    1.0 - 2.0 * (qy * qy + qz * qz));

  if (!start_saved) {
    start_x = robot_x;
    start_y = robot_y;
    start_theta = theta;

    // Transform each waypoint from S into odom.
    for (int i = 0; i < 4; i++) {
      waypoint_odom_x[i] =
        start_x + std::cos(start_theta) * waypoint_s_x[i]
        - std::sin(start_theta) * waypoint_s_y[i];

      waypoint_odom_y[i] =
        start_y + std::sin(start_theta) * waypoint_s_x[i]
        + std::cos(start_theta) * waypoint_s_y[i];
    }

    RCLCPP_INFO(
      node->get_logger(),
      "Start pose: x = %.4f, y = %.4f, theta = %.4f rad",
      start_x, start_y, start_theta);

    for (int i = 0; i < 4; i++) {
      RCLCPP_INFO(
        node->get_logger(),
        "W%d in odom: x = %.4f, y = %.4f",
        i + 1, waypoint_odom_x[i], waypoint_odom_y[i]);
    }

    start_saved = true;
  }

  // Default construction gives an all-zero velocity command.
  geometry_msgs::msg::Twist out;

  // Do not move until a scan has arrived, or while blocked.
  if (!scan_received || obstacle_too_close) {
    pub_->publish(out);
    return;
  }

  double dx = waypoint_odom_x[waypoint_index] - robot_x;
  double dy = waypoint_odom_y[waypoint_index] - robot_y;

  // Transform the goal vector from odom into robot frame R.
  double x_R =
    std::cos(theta) * dx + std::sin(theta) * dy;

  double y_R =
    -std::sin(theta) * dx + std::cos(theta) * dy;

  double d = std::sqrt(x_R * x_R + y_R * y_R);
  double alpha = std::atan2(y_R, x_R);

  if (d < 0.10) {
    RCLCPP_INFO(
      node->get_logger(),
      "Reached W%d.",
      waypoint_index + 1);

    waypoint_index = (waypoint_index + 1) % 4;

    RCLCPP_INFO(
      node->get_logger(),
      "Heading to W%d.",
      waypoint_index + 1);

    pub_->publish(out);
    return;
  }

  // Reuse last week's steering rule.
  out.angular.z = alpha;

  if (out.angular.z > 0.8) {
    out.angular.z = 0.8;
  } else if (out.angular.z < -0.8) {
    out.angular.z = -0.8;
  }

  if (std::abs(alpha) < 0.3) {
    out.linear.x = 0.5 * d;

    if (out.linear.x > 0.2) {
      out.linear.x = 0.2;
    }
  } else {
    out.linear.x = 0.0;
  }

  pub_->publish(out);
}

int main(int argc, char ** argv)
{
  init(argc, argv);

  node = Node::make_shared("patrol");

  pub_ = node->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10);

  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    "/odom",
    SensorDataQoS(),
    &odom_callback);

  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan",
    SensorDataQoS(),
    &scan_callback);

  spin(node);
  shutdown();

  return 0;
}