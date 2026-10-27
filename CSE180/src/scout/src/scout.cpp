#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace rclcpp;

const double PI = 3.14159265358979323846;
const double D_PARK = 0.40;
const double BETA = 30.0 * PI / 180.0;

const std::string ODOM_FRAME = "odom";
const std::string ROBOT_FRAME = "base_link";

enum Phase { DETECT, APPROACH, RETURN, DONE };
Phase phase = DETECT;

Node::SharedPtr node;
Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
TimerBase::SharedPtr timer_;

std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;

sensor_msgs::msg::LaserScan::SharedPtr latest_scan;

geometry_msgs::msg::PointStamped target_odom;
geometry_msgs::msg::PointStamped home_odom;

bool tf_failing = false;
bool no_target_reported = false;

double clamp(double value, double minimum, double maximum)
{
  return std::max(minimum, std::min(value, maximum));
}

void publish_stop()
{
  geometry_msgs::msg::Twist out;
  pub_->publish(out);
}

void report_tf_recovery()
{
  if (tf_failing) {
    RCLCPP_WARN(node->get_logger(), "TF lookups recovered.");
    tf_failing = false;
  }
}

// The scan callback only stores the newest scan.
void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  latest_scan = msg;
}

void timer_callback()
{
  if (phase == DONE) {
    publish_stop();
    return;
  }

  if (phase == DETECT && !latest_scan) {
    publish_stop();
    return;
  }

  try {
    if (phase == DETECT) {
      auto scan = latest_scan;
      rclcpp::Time scan_time(scan->header.stamp);

      // Target frame first, source frame second.
      // Detection uses the transform at the scan timestamp.
      auto scanner_to_robot = tf_buffer->lookupTransform(
        ROBOT_FRAME,
        scan->header.frame_id,
        scan_time);

      auto scanner_to_odom = tf_buffer->lookupTransform(
        ODOM_FRAME,
        scan->header.frame_id,
        scan_time);

      auto robot_to_odom = tf_buffer->lookupTransform(
        ODOM_FRAME,
        ROBOT_FRAME,
        scan_time);

      report_tf_recovery();

      bool found = false;
      double closest_distance = std::numeric_limits<double>::infinity();

      geometry_msgs::msg::PointStamped best_scanner;
      geometry_msgs::msg::PointStamped best_robot;

      for (std::size_t i = 0; i < scan->ranges.size(); i++) {
        double range = scan->ranges[i];

        if (!std::isfinite(range) ||
            range < scan->range_min ||
            range > scan->range_max) {
          continue;
        }

        double angle =
          scan->angle_min +
          static_cast<double>(i) * scan->angle_increment;

        geometry_msgs::msg::PointStamped point_scanner;
        point_scanner.header = scan->header;

        // Polar-to-Cartesian conversion, still in the scanner frame.
        point_scanner.point.x = range * std::cos(angle);
        point_scanner.point.y = range * std::sin(angle);
        point_scanner.point.z = 0.0;

        geometry_msgs::msg::PointStamped point_robot;
        tf2::doTransform(
          point_scanner, point_robot, scanner_to_robot);

        double x_R = point_robot.point.x;
        double y_R = point_robot.point.y;

        double bearing = std::atan2(y_R, x_R);
        double distance = std::hypot(x_R, y_R);

        // Apply the sector test in the ROBOT frame.
        if (x_R > 0.0 &&
            std::abs(bearing) <= BETA &&
            distance < closest_distance) {
          found = true;
          closest_distance = distance;
          best_scanner = point_scanner;
          best_robot = point_robot;
        }
      }

      if (!found) {
        publish_stop();

        if (!no_target_reported) {
          RCLCPP_INFO(
            node->get_logger(),
            "No valid point in the forward sector. Retrying.");
          no_target_reported = true;
        }

        return;
      }

      no_target_reported = false;

      tf2::doTransform(
        best_scanner, target_odom, scanner_to_odom);

      home_odom.header.frame_id = ODOM_FRAME;
      home_odom.header.stamp = scan->header.stamp;
      home_odom.point.x = robot_to_odom.transform.translation.x;
      home_odom.point.y = robot_to_odom.transform.translation.y;
      home_odom.point.z = robot_to_odom.transform.translation.z;

      RCLCPP_INFO(
        node->get_logger(),
        "Target in scanner (%s): x = %.6f, y = %.6f, z = %.6f",
        scan->header.frame_id.c_str(),
        best_scanner.point.x,
        best_scanner.point.y,
        best_scanner.point.z);

      RCLCPP_INFO(
        node->get_logger(),
        "Target in robot (%s): x = %.6f, y = %.6f, z = %.6f",
        ROBOT_FRAME.c_str(),
        best_robot.point.x,
        best_robot.point.y,
        best_robot.point.z);

      RCLCPP_INFO(
        node->get_logger(),
        "Target in odom: x = %.6f, y = %.6f, z = %.6f",
        target_odom.point.x,
        target_odom.point.y,
        target_odom.point.z);

      RCLCPP_INFO(
        node->get_logger(),
        "Home in odom: x = %.6f, y = %.6f, z = %.6f",
        home_odom.point.x,
        home_odom.point.y,
        home_odom.point.z);

      phase = APPROACH;

      // No movement on the detection tick.
      publish_stop();
      return;
    }

    // APPROACH and RETURN use the latest available transform.
    auto odom_to_robot = tf_buffer->lookupTransform(
      ROBOT_FRAME,
      ODOM_FRAME,
      tf2::TimePointZero);

    report_tf_recovery();

    geometry_msgs::msg::PointStamped goal_robot;

    if (phase == APPROACH) {
      tf2::doTransform(
        target_odom, goal_robot, odom_to_robot);
    } else {
      tf2::doTransform(
        home_odom, goal_robot, odom_to_robot);
    }

    double x_R = goal_robot.point.x;
    double y_R = goal_robot.point.y;

    // Planar distance: ignore the scanner mounting height.
    double d = std::hypot(x_R, y_R);
    double alpha = std::atan2(y_R, x_R);

    if (phase == APPROACH && d < D_PARK) {
      publish_stop();

      RCLCPP_INFO(
        node->get_logger(),
        "Parked at %.3f m from target. Returning home.",
        d);

      phase = RETURN;
      return;
    }

    if (phase == RETURN && d < 0.10) {
      publish_stop();

      RCLCPP_INFO(
        node->get_logger(),
        "Returned home within %.3f m. DONE.",
        d);

      phase = DONE;
      return;
    }

    geometry_msgs::msg::Twist out;

    out.angular.z = clamp(alpha, -0.8, 0.8);

    if (std::abs(alpha) < 0.3) {
      out.linear.x = clamp(0.5 * d, 0.0, 0.2);
    }

    pub_->publish(out);
  } catch (const tf2::TransformException & ex) {
    publish_stop();

    if (!tf_failing) {
      RCLCPP_WARN(
        node->get_logger(),
        "TF lookup failed; stopping until recovery: %s",
        ex.what());

      tf_failing = true;
    }
  }
}

int main(int argc, char ** argv)
{
  init(argc, argv);

  node = Node::make_shared("scout");

  pub_ = node->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10);

  tf_buffer = std::make_shared<tf2_ros::Buffer>(
    node->get_clock());

  // The listener handles TF subscriptions in its dedicated thread.
  tf_listener = std::make_shared<tf2_ros::TransformListener>(
    *tf_buffer, node, true);

  sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan",
    SensorDataQoS(),
    &scan_callback);

  timer_ = node->create_wall_timer(
    std::chrono::milliseconds(100),
    &timer_callback);

  spin(node);

  tf_listener.reset();
  tf_buffer.reset();

  shutdown();
  return 0;
}