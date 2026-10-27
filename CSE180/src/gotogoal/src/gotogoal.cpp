#include <algorithm>
#include <cmath>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace rclcpp;

// Goal coordinates in the start frame S.
const double a = 1.00;
const double b = 0.50;

// True after the first odometry message has established frame S.
bool start_saved = false;

// True after the robot has reached the goal.
// This prevents printing "Goal reached" on every odometry callback.
bool goal_reached = false;

// Fixed goal coordinates, expressed in the odom frame.
double goal_odom_x = 0.0;
double goal_odom_y = 0.0;

Node::SharedPtr node;
Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

double clamp(double value, double minimum, double maximum)
{
  return std::max(minimum, std::min(value, maximum));
}

double yaw_from_quaternion(double qx, double qy, double qz, double qw)
{
  double sin_yaw = 2.0 * (qw * qz + qx * qy);
  double cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz);

  return std::atan2(sin_yaw, cos_yaw);
}

void callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Current robot pose, expressed in odom.
  double robot_x = msg->pose.pose.position.x;
  double robot_y = msg->pose.pose.position.y;

  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;

  double theta = yaw_from_quaternion(qx, qy, qz, qw);

  // On the first message, the robot's current pose defines start frame S.
  if (!start_saved) {
    goal_odom_x =
      robot_x + std::cos(theta) * a - std::sin(theta) * b;

    goal_odom_y =
      robot_y + std::sin(theta) * a + std::cos(theta) * b;

    RCLCPP_INFO(
      node->get_logger(),
      "Start pose: x = %.4f, y = %.4f, theta = %.4f rad",
      robot_x, robot_y, theta);

    RCLCPP_INFO(
      node->get_logger(),
      "Goal in odom: x = %.4f, y = %.4f",
      goal_odom_x, goal_odom_y);

    start_saved = true;
  }

  // Goal relative to robot, first as an odom-frame vector.
  double dx = goal_odom_x - robot_x;
  double dy = goal_odom_y - robot_y;

  // Rotate the odom-frame vector into the current robot frame R.
  double x_R =
    std::cos(theta) * dx + std::sin(theta) * dy;

  double y_R =
    -std::sin(theta) * dx + std::cos(theta) * dy;

  double d = std::sqrt(x_R * x_R + y_R * y_R);
  double alpha = std::atan2(y_R, x_R);

  geometry_msgs::msg::Twist out;

  // A default Twist has every field set to zero.
  if (d < 0.10) {
    pub_->publish(out);

    if (!goal_reached) {
      RCLCPP_INFO(node->get_logger(), "Goal reached. Stopping robot.");
      goal_reached = true;
    }

    return;
  }

  // angular.z = 1.0 * alpha, limited to +/- 0.8 rad/s.
  out.angular.z = clamp(1.0 * alpha, -0.8, 0.8);

  // linear.x = 0.5 * d, capped at 0.2 m/s,
  // only when the goal is approximately in front of the robot.
  if (std::abs(alpha) < 0.3) {
    out.linear.x = clamp(0.5 * d, 0.0, 0.2);
  } else {
    out.linear.x = 0.0;
  }

  pub_->publish(out);
}

int main(int argc, char ** argv)
{
  init(argc, argv);

  node = Node::make_shared("gotogoal");

  sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    "/odom",
    10,
    &callback);

  pub_ = node->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    10);

  spin(node);
  shutdown();

  return 0;
}