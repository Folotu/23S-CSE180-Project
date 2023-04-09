#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include <iostream>
#include <navigation/navigation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
int main(int argc, char **argv) {

  rclcpp::init(argc, argv); // initialize ROS
  Navigator navigator(true,
                      false); // create node with debug info but not verbose

  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  waypoints.reserve(7);

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "map";
  //       {2, 1, 0, 0, 0, 0, 1},    // 2. Upper left corner
  pose_stamped.pose.position.x = 2;
  pose_stamped.pose.position.y = 1;
  pose_stamped.pose.orientation.w = 1;
  waypoints.push_back(pose_stamped);
  //       {2, -0.05, 0, 0, 0, 0, 1},     // 3. Top middle
  pose_stamped.pose.position.x = 2;
  pose_stamped.pose.position.y = -0.05;
  pose_stamped.pose.orientation.w = 1;
  waypoints.push_back(pose_stamped);
  //       {-1.7, 0.005, 0, 0, 0, 0, 1},     // 7. Bottom middle
  pose_stamped.pose.position.x = -1.7;
  pose_stamped.pose.position.y = 0.005;
  pose_stamped.pose.orientation.w = 1;
  waypoints.push_back(pose_stamped);
  //       {1.7, -1, 0, 0, 0, 0, 1},     // 4. Upper right corner
  pose_stamped.pose.position.x = 1.7;
  pose_stamped.pose.position.y = -1;
  pose_stamped.pose.orientation.w = 1;
  waypoints.push_back(pose_stamped);
  //       {-2, -0.5, 0, 0, 0, 0, 1},     // 6. Lower right corner
  pose_stamped.pose.position.x = -2;
  pose_stamped.pose.position.y = -0.5;
  pose_stamped.pose.orientation.w = 1;
  waypoints.push_back(pose_stamped);
  //       {-1.7, 1, 0, 0, 0, 0, 1}     // 8. Lower left corner
  pose_stamped.pose.position.x = -1.7;
  pose_stamped.pose.position.y = 1;
  pose_stamped.pose.orientation.w = 1;
  waypoints.push_back(pose_stamped);

  // first: it is mandatory to initialize the pose of the robot
  geometry_msgs::msg::Pose::SharedPtr init =
      std::make_shared<geometry_msgs::msg::Pose>();
  init->position.x = -2;
  init->position.y = -0.5;
  init->orientation.w = 1;
  navigator.SetInitialPose(init);
  // wait for navigation stack to become operational
  navigator.WaitUntilNav2Active();
  // spin in place of 90 degrees (default parameter)
  navigator.Spin();
  while (!navigator.IsTaskComplete()) {
    // busy waiting for task to be completed
  }
  //   for (const auto &waypoint : waypoints) {
  //     geometry_msgs::msg::Pose::SharedPtr goal_pos =
  //         std::make_shared<geometry_msgs::msg::Pose>();
  //     *goal_pos = waypoint;
  //     navigator.GoToPose(goal_pos);
  if (navigator.FollowWaypoints(waypoints)) {
    RCLCPP_INFO(navigator.get_logger(),
                "Successfully sent FollowWaypoints request");
  } else {
    RCLCPP_ERROR(navigator.get_logger(),
                 "Failed to send FollowWaypoints request");
  }
  while (!navigator.IsTaskComplete()) {
    // busy waiting for the task to be completed
  }

  // backup of 0.15 m (default distance)
  navigator.Backup();
  while (!navigator.IsTaskComplete()) {
  }

  // complete here...

  rclcpp::shutdown(); // shutdown ROS
  return 0;
}