#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <finalproject/navigation.hpp>
#include <iostream>

class MyTestNavigator : public rclcpp::Node
{
public:
  MyTestNavigator()
    : Node("my_test_navigator")
  {
    // Subscribe to the map topic
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&MyTestNavigator::mapCallback, this, std::placeholders::_1));

    // Initialize the navigator with debug info but not verbose
    navigator_ = std::make_unique<Navigator>(true, false);
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    // Extract map information
    const auto width = msg->info.width;
    const auto height = msg->info.height;
    const auto resolution = msg->info.resolution;
    const auto occupancy_grid = msg->data;

    // Set the initial pose of the robot
    geometry_msgs::msg::Pose::SharedPtr init = std::make_shared<geometry_msgs::msg::Pose>();
    init->position.x = -2;
    init->position.y = -0.5;
    init->orientation.w = 1;
    navigator_->SetInitialPose(init);

    // Wait for navigation stack to become operational
    navigator_->WaitUntilNav2Active();

    // Spin in place of 90 degrees (default parameter)
    navigator_->Spin();

    // Wait for task to be completed
    while (!navigator_->IsTaskComplete()) {}

    // Move to the first goal position
    geometry_msgs::msg::Pose::SharedPtr goal_pos = std::make_shared<geometry_msgs::msg::Pose>();
    goal_pos->position.x = 2;
    goal_pos->position.y = 1;
    goal_pos->orientation.w = 1;
    navigator_->GoToPose(goal_pos);

    // Wait for task to be completed
    while (!navigator_->IsTaskComplete()) {}

    // Move to the second goal position
    goal_pos->position.x = 2;
    goal_pos->position.y = -1;
    goal_pos->orientation.w = 1;
    navigator_->GoToPose(goal_pos);

    // Wait for task to be completed
    while (!navigator_->IsTaskComplete()) {}

    // Backup of 0.15 m (default distance)
    navigator_->Backup();

    // Wait for task to be completed
    while (!navigator_->IsTaskComplete()) {}

    // Complete here....
    // You can use the map information here to plan and navigate to additional goal positions
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  std::unique_ptr<Navigator> navigator_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv); // Initialize ROS

  auto node = std::make_shared<MyTestNavigator>();

  rclcpp::spin(node); // Run node until it's exited

  rclcpp::shutdown(); // Shutdown ROS
  return 0;
}
