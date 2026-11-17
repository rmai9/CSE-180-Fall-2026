// search -- final project: find the people in the warehouse map and report them.
//
// Stage 2 (this file): package skeleton only. It creates the node and the
// course Navigator, then idles until shutdown. Nothing here moves the robot,
// subscribes to images, or processes depth yet.
//
// Where the rest will go (see README.md, "Where the remaining functionality
// will go" -- deliberately not written as unused scaffolding):
//   - the state machine replaces rclcpp::spin(node) below
//   - camera/depth subscriptions: callback that only stores the latest frame
//   - TF2 map <- camera optical frame lookups, inside that state machine
//   - person merging, markers, detector integration: new code

#include <rclcpp/rclcpp.hpp>

#include <navigation/navigation.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Same shape as the lab programs (gotogoal, patrol, scout): one node created
  // in main(), used by free functions. This is the usual rclcpp structure, not
  // code taken from those labs.
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("search");

  RCLCPP_INFO(
    node->get_logger(),
    "search skeleton: stage 2 -- no waypoints, perception or merging yet");

  // The course Navigator is itself an rclcpp node named "navigator". Every one
  // of its blocking calls spins its own node, so it must never be added to an
  // executor that is already spinning. It is therefore created here, in main(),
  // and is driven by the state machine in the later stages -- never spun.
  //
  // First argument: debug logging; second: verbose. debug shows that the
  // library really ran, which is what this stage needs to prove.
  Navigator navigator(true, false);

  RCLCPP_INFO(
    node->get_logger(),
    "Navigator constructed from the MRTP navigation::navigator target");

  // Nothing to do yet: wait for Ctrl+C (rclcpp turns SIGINT into a shutdown)
  // and leave cleanly.
  rclcpp::spin(node);

  RCLCPP_INFO(node->get_logger(), "shutdown requested, exiting");

  rclcpp::shutdown();
  return 0;
}
