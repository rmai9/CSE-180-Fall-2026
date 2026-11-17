// search -- final project: find the people in the warehouse map and report them.
//
// Stage 3 (this file): navigation-only search state machine. The robot drives a
// short list of map-frame viewpoints, turns once at each of them, and drives back
// to the pose it saved at startup. There is no perception yet: RGB/depth
// processing, person merging and markers arrive in the later stages (README.md,
// section G).
//
// Shape: one node named "search" created in main(), plus the course Navigator
// created in main() and handed by reference to the free functions below. That is
// the same structure as gotogoal/patrol/scout -- explicit state, no node subclass.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <navigation/navigation.hpp>

// ---------------------------------------------------------------------------
// Search plan, in the map frame.
//
// Both viewpoints are the poses used to photograph the two people in the stage-2
// detector experiment, so they are known free space 3.5-4 m from a person, facing
// it. Map-derived viewpoints replace this list in stage 8.
// ---------------------------------------------------------------------------
struct MapWaypoint
{
  double x;
  double y;
  double yaw;
  const char * name;
};

const MapWaypoint WAYPOINTS[] = {
  {1.00, 2.50, -1.5708, "3.5 m north of person 1, facing -y"},
  {-12.83, 11.09, 1.3620, "4.0 m south of person 2, facing +y"},
};

const std::size_t WAYPOINT_COUNT = sizeof(WAYPOINTS) / sizeof(WAYPOINTS[0]);

const double SPIN_ANGLE = 1.57;      // rad, a quarter turn at each viewpoint
const double SPIN_TIMEOUT = 60.0;    // s of wall clock

// Goal deadline. Nav2's own progress checker already aborts a goal that stops
// making progress (0.5 m in 10 s in params/nav2_params_wh_tb4.yaml), so this
// deadline is a backstop for a result that never arrives, not the primary
// stuck detector -- it only has to be generous. Measured in this map, the path
// is up to three times the straight-line distance: 16.2 m straight needed ~48 m
// of driving, because the aisles force a detour around the shelf rows.
const double CRUISE_SPEED = 0.25;    // m/s, about half the planner's top speed
const double DETOUR_FACTOR = 3.0;    // path length / straight-line distance
const double GOAL_OVERHEAD = 30.0;   // s
const double MIN_GOAL_TIMEOUT = 120.0;
const double MAX_GOAL_TIMEOUT = 600.0;

const double NAV2_READY_TIMEOUT = 60.0;   // s
const double TF_POSE_TIMEOUT = 30.0;      // s

const std::string MAP_FRAME = "map";
const std::string ROBOT_FRAME = "base_link";

// Robot pose in the map frame, refreshed from TF. /amcl_pose is not a continuous
// stream (it is published on AMCL updates, not periodically), so TF is the only
// always-available source.
geometry_msgs::msg::Pose robot_pose;
bool robot_pose_valid = false;
bool tf_failing = false;

std::shared_ptr<tf2_ros::Buffer> tf_buffer;

// Quaternion to yaw. Same formula as the odometry callback in patrol.cpp, just
// moved into a free function.
double yaw_of(const geometry_msgs::msg::Quaternion & orientation)
{
  const double x = orientation.x;
  const double y = orientation.y;
  const double z = orientation.z;
  const double w = orientation.w;

  return std::atan2(
    2.0 * (w * z + x * y),
    1.0 - 2.0 * (y * y + z * z));
}

double planar_distance(const geometry_msgs::msg::Pose & pose, double x, double y)
{
  return std::hypot(pose.position.x - x, pose.position.y - y);
}

// Build a map-frame pose from x, y and yaw, the form Navigator::GoToPose wants.
geometry_msgs::msg::Pose::SharedPtr pose_at(double x, double y, double yaw)
{
  auto pose = std::make_shared<geometry_msgs::msg::Pose>();

  pose->position.x = x;
  pose->position.y = y;
  pose->position.z = 0.0;

  pose->orientation.z = std::sin(yaw / 2.0);
  pose->orientation.w = std::cos(yaw / 2.0);

  return pose;
}

double goal_timeout_for(double distance)
{
  const double needed = GOAL_OVERHEAD + DETOUR_FACTOR * std::abs(distance) / CRUISE_SPEED;

  return std::max(MIN_GOAL_TIMEOUT, std::min(needed, MAX_GOAL_TIMEOUT));
}

// One lookup of map <- base_link at the newest available time, with the
// warn-once-on-failure behaviour from scout.cpp (there the consequence was a stop
// command; here it is skipping the update of the last known pose).
//
// Spin this node first: the TF buffer is only filled while it is being spun, and
// a Navigator call spins its own node, not this one. wait_for_task() spins on
// every iteration, so a pose read straight after a task ends is current to about
// one poll (100 ms).
void refresh_robot_pose(const rclcpp::Node::SharedPtr & node)
{
  try {
    const geometry_msgs::msg::TransformStamped map_to_robot =
      tf_buffer->lookupTransform(MAP_FRAME, ROBOT_FRAME, tf2::TimePointZero);

    robot_pose.position.x = map_to_robot.transform.translation.x;
    robot_pose.position.y = map_to_robot.transform.translation.y;
    robot_pose.position.z = map_to_robot.transform.translation.z;
    robot_pose.orientation = map_to_robot.transform.rotation;
    robot_pose_valid = true;

    if (tf_failing) {
      RCLCPP_WARN(node->get_logger(), "TF lookups recovered.");
      tf_failing = false;
    }
  } catch (const tf2::TransformException & ex) {
    if (!tf_failing) {
      RCLCPP_WARN(
        node->get_logger(), "TF lookup %s <- %s failed: %s",
        MAP_FRAME.c_str(), ROBOT_FRAME.c_str(), ex.what());
      tf_failing = true;
    }
  }
}

void update_robot_pose(const rclcpp::Node::SharedPtr & node)
{
  rclcpp::spin_some(node);
  refresh_robot_pose(node);
}

// ---------------------------------------------------------------------------
// Task bookkeeping.
//
// The Navigator has no goal timeout of its own, so wait_for_task() polls
// IsTaskComplete() (which spins the Navigator node for 100 ms itself) against a
// deadline and cancels on expiry. A timed-out goal is reported separately from
// one Nav2 aborted: the robot has to know whether it gave up or Nav2 did.
// ---------------------------------------------------------------------------
enum class Outcome { SUCCEEDED, ABORTED, CANCELED, UNKNOWN, TIMED_OUT, SHUTDOWN };

const char * outcome_name(Outcome outcome)
{
  switch (outcome) {
    case Outcome::SUCCEEDED: return "succeeded";
    case Outcome::ABORTED: return "aborted by Nav2";
    case Outcome::CANCELED: return "canceled";
    case Outcome::UNKNOWN: return "unknown result";
    case Outcome::TIMED_OUT: return "deadline expired";
    case Outcome::SHUTDOWN: return "interrupted";
  }
  return "unknown";
}

Outcome wait_for_task(
  Navigator & navigator, const rclcpp::Node::SharedPtr & node, double timeout_s)
{
  // Wall clock, not the node clock: simulation time can pause or stall (and is
  // zero before the first /clock), and a deadline that never fires is a hang.
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(static_cast<long>(timeout_s * 1000.0));

  while (rclcpp::ok()) {
    if (navigator.IsTaskComplete()) {
      switch (navigator.GetResult()) {
        case rclcpp_action::ResultCode::SUCCEEDED: return Outcome::SUCCEEDED;
        case rclcpp_action::ResultCode::ABORTED: return Outcome::ABORTED;
        case rclcpp_action::ResultCode::CANCELED: return Outcome::CANCELED;
        default: return Outcome::UNKNOWN;
      }
    }

    // Keep this node's subscriptions and the TF buffer serviced while waiting.
    rclcpp::spin_some(node);

    if (std::chrono::steady_clock::now() >= deadline) {
      RCLCPP_WARN(
        node->get_logger(),
        "Task deadline of %.0f s expired; cancelling it.", timeout_s);
      navigator.CancelTask();
      return Outcome::TIMED_OUT;
    }
  }

  return Outcome::SHUTDOWN;
}

// Bounded replacement for the unchecked part of Navigator::WaitUntilNav2Active(),
// which waits for lifecycle services with no timeout and would hang forever if
// the stack is not running. Once both services are visible, its remaining waits
// have a server to talk to.
bool wait_for_nav2_services(const rclcpp::Node::SharedPtr & node, double timeout_s)
{
  const auto start = std::chrono::steady_clock::now();
  const auto deadline = start +
    std::chrono::milliseconds(static_cast<long>(timeout_s * 1000.0));

  bool amcl_ready = false;
  bool bt_ready = false;

  while (rclcpp::ok()) {
    const auto services = node->get_service_names_and_types();
    amcl_ready = services.find("/amcl/get_state") != services.end();
    bt_ready = services.find("/bt_navigator/get_state") != services.end();

    if (amcl_ready && bt_ready) {
      const double waited = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
      RCLCPP_INFO(
        node->get_logger(),
        "Nav2 lifecycle services visible after %.1f s.", waited);
      return true;
    }

    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if (std::chrono::steady_clock::now() >= deadline) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Nav2 did not appear within %.0f s (/amcl/get_state %s, "
        "/bt_navigator/get_state %s). Is the simulation with Nav2 running?",
        timeout_s,
        amcl_ready ? "present" : "missing",
        bt_ready ? "present" : "missing");
      return false;
    }
  }

  return false;
}

bool wait_for_robot_pose(const rclcpp::Node::SharedPtr & node, double timeout_s)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(static_cast<long>(timeout_s * 1000.0));

  while (rclcpp::ok() && !robot_pose_valid) {
    rclcpp::spin_some(node);
    refresh_robot_pose(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
  }

  return robot_pose_valid;
}

// ---------------------------------------------------------------------------
// The search state machine.
//
// One goal per state transition, exactly like the switch in scout.cpp: send,
// then wait, then advance. A failed waypoint does not stop the search -- it is
// counted and the robot moves on -- but a failed return home is reported.
// ---------------------------------------------------------------------------
void run_search(
  Navigator & navigator, const rclcpp::Node::SharedPtr & node,
  const geometry_msgs::msg::Pose & home)
{
  enum State { SEND_GOAL, WAIT_GOAL, SEND_SPIN, WAIT_SPIN, SEND_HOME, WAIT_HOME, DONE };

  State state = SEND_GOAL;
  std::size_t waypoint_index = 0;
  std::size_t waypoints_reached = 0;
  std::size_t waypoint_failures = 0;
  std::size_t spins_completed = 0;
  double goal_timeout = MIN_GOAL_TIMEOUT;
  std::string home_status = "not attempted";

  while (rclcpp::ok() && state != DONE) {
    switch (state) {
      case SEND_GOAL: {
        const MapWaypoint & waypoint = WAYPOINTS[waypoint_index];
        auto goal = pose_at(waypoint.x, waypoint.y, waypoint.yaw);

        // Refresh first: after a failed goal the last known pose is the one from
        // before that goal started, and that misjudges the next deadline.
        update_robot_pose(node);
        const double distance = planar_distance(robot_pose, waypoint.x, waypoint.y);
        goal_timeout = goal_timeout_for(distance);

        RCLCPP_INFO(
          node->get_logger(),
          "Waypoint %zu/%zu (%s): map-frame goal x=%.2f y=%.2f yaw=%.2f, "
          "%.1f m away, deadline %.0f s",
          waypoint_index + 1, WAYPOINT_COUNT, waypoint.name,
          waypoint.x, waypoint.y, waypoint.yaw, distance, goal_timeout);

        if (navigator.GoToPose(goal)) {
          state = WAIT_GOAL;
        } else {
          RCLCPP_ERROR(
            node->get_logger(),
            "Waypoint %zu/%zu: goal was not accepted, skipping it.",
            waypoint_index + 1, WAYPOINT_COUNT);
          ++waypoint_failures;
          ++waypoint_index;
          state = (waypoint_index < WAYPOINT_COUNT) ? SEND_GOAL : SEND_HOME;
        }
        break;
      }

      case WAIT_GOAL: {
        const MapWaypoint & waypoint = WAYPOINTS[waypoint_index];
        const Outcome outcome = wait_for_task(navigator, node, goal_timeout);

        if (outcome == Outcome::SHUTDOWN) {
          RCLCPP_WARN(node->get_logger(), "Shutdown requested; abandoning the search.");
          state = DONE;
          break;
        }

        if (outcome == Outcome::SUCCEEDED) {
          ++waypoints_reached;
          update_robot_pose(node);
          RCLCPP_INFO(
            node->get_logger(),
            "Waypoint %zu/%zu reached; robot is %.2f m from the requested pose.",
            waypoint_index + 1, WAYPOINT_COUNT,
            planar_distance(robot_pose, waypoint.x, waypoint.y));
          state = SEND_SPIN;
          break;
        }

        ++waypoint_failures;
        RCLCPP_WARN(
          node->get_logger(),
          "Waypoint %zu/%zu failed (%s); continuing with the next waypoint.",
          waypoint_index + 1, WAYPOINT_COUNT, outcome_name(outcome));
        ++waypoint_index;
        state = (waypoint_index < WAYPOINT_COUNT) ? SEND_GOAL : SEND_HOME;
        break;
      }

      case SEND_SPIN: {
        RCLCPP_INFO(
          node->get_logger(),
          "Looking around: sending Spin %.2f rad, deadline %.0f s.",
          SPIN_ANGLE, SPIN_TIMEOUT);

        if (navigator.Spin(SPIN_ANGLE)) {
          state = WAIT_SPIN;
        } else {
          RCLCPP_ERROR(node->get_logger(), "Spin request was not accepted.");
          ++waypoint_index;
          state = (waypoint_index < WAYPOINT_COUNT) ? SEND_GOAL : SEND_HOME;
        }
        break;
      }

      case WAIT_SPIN: {
        const Outcome outcome = wait_for_task(navigator, node, SPIN_TIMEOUT);

        if (outcome == Outcome::SHUTDOWN) {
          RCLCPP_WARN(node->get_logger(), "Shutdown requested; abandoning the search.");
          state = DONE;
          break;
        }

        if (outcome == Outcome::SUCCEEDED) {
          ++spins_completed;
          update_robot_pose(node);
          RCLCPP_INFO(
            node->get_logger(),
            "Spin finished at waypoint %zu/%zu; robot yaw is now %.2f rad.",
            waypoint_index + 1, WAYPOINT_COUNT, yaw_of(robot_pose.orientation));
        } else {
          RCLCPP_WARN(
            node->get_logger(),
            "Spin at waypoint %zu/%zu did not finish (%s); continuing.",
            waypoint_index + 1, WAYPOINT_COUNT, outcome_name(outcome));
        }

        ++waypoint_index;
        state = (waypoint_index < WAYPOINT_COUNT) ? SEND_GOAL : SEND_HOME;
        break;
      }

      case SEND_HOME: {
        update_robot_pose(node);
        const double distance = planar_distance(
          robot_pose, home.position.x, home.position.y);
        goal_timeout = goal_timeout_for(distance);

        RCLCPP_INFO(
          node->get_logger(),
          "No more viewpoints: returning to the saved home pose x=%.2f y=%.2f "
          "yaw=%.2f, %.1f m away, deadline %.0f s.",
          home.position.x, home.position.y, yaw_of(home.orientation),
          distance, goal_timeout);

        if (navigator.GoToPose(std::make_shared<geometry_msgs::msg::Pose>(home))) {
          state = WAIT_HOME;
        } else {
          RCLCPP_ERROR(node->get_logger(), "Home goal was not accepted.");
          home_status = "rejected";
          state = DONE;
        }
        break;
      }

      case WAIT_HOME: {
        const Outcome outcome = wait_for_task(navigator, node, goal_timeout);

        if (outcome == Outcome::SHUTDOWN) {
          home_status = "interrupted";
          state = DONE;
          break;
        }

        if (outcome == Outcome::SUCCEEDED) {
          home_status = "reached";
          update_robot_pose(node);
          RCLCPP_INFO(
            node->get_logger(),
            "Home reached; robot is %.2f m from the saved pose.",
            planar_distance(robot_pose, home.position.x, home.position.y));
        } else {
          home_status = outcome_name(outcome);
          RCLCPP_ERROR(
            node->get_logger(), "Returning home failed (%s).", outcome_name(outcome));
        }

        state = DONE;
        break;
      }

      case DONE:
        break;
    }
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Search finished: %zu/%zu waypoints reached, %zu failed, %zu spin(s) done, home %s.",
    waypoints_reached, WAYPOINT_COUNT, waypoint_failures, spins_completed,
    home_status.c_str());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Same shape as the lab programs: one node created in main(), used by free
  // functions. This is the usual rclcpp structure, not code from the labs.
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("search");

  // TF buffer and listener, as in scout.cpp, but with the listener on this node's
  // own executor: the node is spun by hand below, so a dedicated listener thread
  // would put the node in a second executor.
  tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer, node, false);

  int exit_code = 0;

  {
    // The course Navigator is also a node named "navigator". Every one of its
    // blocking calls spins its own node, so it is never added to an executor and
    // never spun from another thread; it is only driven by the calls below.
    //
    // Debug logging stays off: with it on, the library reports a successful goal
    // at ERROR severity (print_result_diagnostic), so the state machine logs its
    // own results instead of relying on the library's text.
    Navigator navigator(false, false);

    if (!wait_for_nav2_services(node, NAV2_READY_TIMEOUT)) {
      RCLCPP_ERROR(node->get_logger(), "Nav2 is not usable; exiting.");
      exit_code = 1;
    } else if (!wait_for_robot_pose(node, TF_POSE_TIMEOUT)) {
      RCLCPP_ERROR(
        node->get_logger(), "No %s <- %s transform; cannot save the home pose.",
        MAP_FRAME.c_str(), ROBOT_FRAME.c_str());
      exit_code = 1;
    } else {
      const geometry_msgs::msg::Pose home = robot_pose;

      RCLCPP_INFO(
        node->get_logger(),
        "Home pose saved in map frame: x=%.2f y=%.2f yaw=%.2f",
        home.position.x, home.position.y, yaw_of(home.orientation));

      // WaitUntilNav2Active() republishes the Navigator's stored initial pose, and
      // that member is all zeros unless SetInitialPose was called first. Left
      // alone it overwrites AMCL's launch-provided pose with (0,0,0), which was
      // observed in the first stage-3 run: every DWB plan was empty and all three
      // goals were aborted. Passing the true pose back makes that republish a
      // no-op for localisation.
      navigator.SetInitialPose(std::make_shared<geometry_msgs::msg::Pose>(home));
      navigator.WaitUntilNav2Active();
      RCLCPP_INFO(node->get_logger(), "Nav2 is active.");

      if (rclcpp::ok()) {
        run_search(navigator, node, home);
      }
    }
  }
  // Navigator is destroyed here, while the context is still valid: its destructor
  // calls CancelTask(), which spins the navigator node and is what cancels a goal
  // still running when Ctrl+C ends the state machine.

  RCLCPP_INFO(node->get_logger(), "search finished, exiting.");
  rclcpp::shutdown();
  return exit_code;
}
