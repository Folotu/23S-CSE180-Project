#include <rclcpp/rclcpp.hpp> 
#include <finalproject/navigation.hpp>
#include <iostream>



int main(int argc,char **argv) {
 
  rclcpp::init(argc,argv); // initialize ROS 
  Navigator navigator(true,false); // create node with debug info but not verbose

  // first: it is mandatory to initialize the pose of the robot
  geometry_msgs::msg::Pose::SharedPtr init = std::make_shared<geometry_msgs::msg::Pose>();
  init->position.x = -2;
  init->position.y = -0.5;
  init->orientation.w = 1;
  navigator.SetInitialPose(init);
  // wait for navigation stack to become operationale
  navigator.WaitUntilNav2Active();
  // spin in place of 90 degrees (default parameter)
  navigator.Spin();
  while ( ! navigator.IsTaskComplete() ) {
    // busy waiting for task to be completed
  }
  geometry_msgs::msg::Pose::SharedPtr goal_pos = std::make_shared<geometry_msgs::msg::Pose>();
  goal_pos->position.x = 2;
  goal_pos->position.y = 1;
  goal_pos->orientation.w = 1;
  // move to new pose
  navigator.GoToPose(goal_pos);
  while ( ! navigator.IsTaskComplete() ) {
    
  }
  goal_pos->position.x = 2;
  goal_pos->position.y = -1;
  goal_pos->orientation.w = 1;
  navigator.GoToPose(goal_pos);
  // move to new pose
  while ( ! navigator.IsTaskComplete() ) {
    
  }
  // backup of 0.15 m (deafult distance)
  navigator.Backup();
  while ( ! navigator.IsTaskComplete() ) {
    
  }

  // complete here....
  
  rclcpp::shutdown(); // shutdown ROS
  return 0;
}
