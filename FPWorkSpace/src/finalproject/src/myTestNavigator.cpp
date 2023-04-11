#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include <iostream>
#include <finalproject/navigation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

std::pair<int, int> ConvertLocalToGlobalCostmapCoordinates(int i, int j,
                                                           const std::shared_ptr<nav2_msgs::msg::Costmap>& local_costmap,
                                                           const std::shared_ptr<nav2_msgs::msg::Costmap>& global_costmap) {
  double local_x = i * local_costmap->metadata.resolution + local_costmap->metadata.origin.position.x;
  double local_y = j * local_costmap->metadata.resolution + local_costmap->metadata.origin.position.y;

  int global_i = static_cast<int>((local_x - global_costmap->metadata.origin.position.x) / global_costmap->metadata.resolution);
  int global_j = static_cast<int>((local_y - global_costmap->metadata.origin.position.y) / global_costmap->metadata.resolution);

  return std::make_pair(global_i, global_j);
}

std::vector<std::pair<int, int>> CompareCostmaps(Navigator& navy) {
  auto local_costmap = navy.GetLocalCostmap();
  auto global_costmap = navy.GetGlobalCostmap();

  std::vector<std::pair<int, int>> inconsistent_cells;

  if (!local_costmap || !global_costmap) {
    RCLCPP_WARN(navy.get_logger(), "Failed to compare costmaps: Local or global costmap is missing");
    return inconsistent_cells;
  }

  for (unsigned int i = 0; i < local_costmap->metadata.size_x; ++i) {
    for (unsigned int j = 0; j < local_costmap->metadata.size_y; ++j) {
      std::pair<int, int> global_coord = ConvertLocalToGlobalCostmapCoordinates(i, j, local_costmap, global_costmap);
      if (global_coord.first < 0 || global_coord.second < 0 || 
          global_coord.first >= global_costmap->metadata.size_x || 
          global_coord.second >= global_costmap->metadata.size_y) {
        continue;
      }

      int local_cost = static_cast<int>(local_costmap->data[i + j * local_costmap->metadata.size_x]);
      int global_cost = static_cast<int>(global_costmap->data[global_coord.first + global_coord.second * global_costmap->metadata.size_x]);

      if (local_cost != global_cost) {
        inconsistent_cells.push_back(global_coord);
      }
    }
  }

  return inconsistent_cells;
}



geometry_msgs::msg::PoseStamped GlobalCostmapIndicesToPose(int i, int j, const nav2_msgs::msg::Costmap::SharedPtr& global_costmap) {
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = global_costmap->header.frame_id;
  pose_stamped.header.stamp = global_costmap->header.stamp;
  
  double resolution = global_costmap->metadata.resolution;
  pose_stamped.pose.position.x = i * resolution + global_costmap->metadata.origin.position.x + resolution / 2.0;
  pose_stamped.pose.position.y = j * resolution + global_costmap->metadata.origin.position.y + resolution / 2.0;
  pose_stamped.pose.orientation.w = 1.0; // No orientation information for individual cells

  return pose_stamped;
}




int main(int argc, char **argv) 
{

  rclcpp::init(argc, argv); // initialize ROS
  Navigator navigator(true, false); // create node with debug info but not verbose
  
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.reserve(8);

  geometry_msgs::msg::Pose pose;
  // pose_stamped.header.frame_id = "map";
  //       {0.07, 1.8, 0, 0, 0, 0, 1},    // 9. Left middle 
  pose.position.x = 0.07;
  pose.position.y = 1.8;
  pose.orientation.w = 1;
  waypoints.push_back(pose);
  //       {2, 1, 0, 0, 0, 0, 1},    // 2. Upper left corner
  pose.position.x = 1.7;
  pose.position.y = 1;
  pose.orientation.w = 1;
  waypoints.push_back(pose);
  //       {2, -0.05, 0, 0, 0, 0, 1},     // 3. Top middle
  pose.position.x = 1.7;
  pose.position.y = -0.05;
  pose.orientation.w = 1;
  waypoints.push_back(pose);
  //       {-1.7, 0.005, 0, 0, 0, 0, 1},     // 7. Bottom middle
  pose.position.x = -1.7;
  pose.position.y = 0.005;
  pose.orientation.w = 1;
  waypoints.push_back(pose);
  //       {1.7, -1, 0, 0, 0, 0, 1},     // 4. Upper right corner
  pose.position.x = 1.7;
  pose.position.y = -1;
  pose.orientation.w = 1;
  waypoints.push_back(pose);
//       {0, -1.7, 0, 0, 0, 0, 1},     // 4. Right middle 
  pose.position.x = 0;
  pose.position.y = -1.7;
  pose.orientation.w = 1;
  waypoints.push_back(pose);
  //       {-2, -0.5, 0, 0, 0, 0, 1},     // 6. Lower right corner
  pose.position.x = -1.7;
  pose.position.y = -0.5;
  pose.orientation.w = 1;
  waypoints.push_back(pose);
  //       {-1.7, 1, 0, 0, 0, 0, 1}     // 8. Lower left corner
  pose.position.x = -1.7;
  pose.position.y = 1;
  pose.orientation.w = 1;
  waypoints.push_back(pose);

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
  
    // for (const auto &waypoint : waypoints) {
    //   geometry_msgs::msg::Pose::SharedPtr goal_pos =
    //       std::make_shared<geometry_msgs::msg::Pose>();
    //   *goal_pos = waypoint;
    //   navigator.GoToPose(goal_pos);
  }
  
  for (std::size_t i = 0; i < waypoints.size(); ++i) 
  {
    geometry_msgs::msg::Pose::SharedPtr goal_pos = 
        std::make_shared<geometry_msgs::msg::Pose>();
    *goal_pos = waypoints[i];
    navigator.GoToPose(goal_pos);
    
       while (!navigator.IsTaskComplete()) 
       {
    // busy waiting for the task to be completed
    // std::pair<int, int> inconsistent_cell = CompareCostmaps(navigator); 
      //   std::vector<std::pair<int, int>> inconsistent_cells = CompareCostmaps(navigator);

      //   if (inconsistent_cells.empty()) {
      //   // Vector is empty; there are no inconsistencies
      //   continue;
      //   } else {
      //       // Vector is not empty; there are inconsistencies
      //       auto global_costmap = navigator.GetGlobalCostmap();
      //       geometry_msgs::msg::PoseStamped inconsistent_pose;
      //       for (const auto& inconsistent_cell : inconsistent_cells) 
      //       {
      //         inconsistent_pose = GlobalCostmapIndicesToPose(inconsistent_cell.first, inconsistent_cell.second, global_costmap);

      //       }
      //       RCLCPP_ERROR(navigator.get_logger(), "Unknown Obstacle at position: (x: %.2f, y: %.2f, z: %.2f)",
      //                                           inconsistent_pose.pose.position.x,
      //                                           inconsistent_pose.pose.position.y,
      //                                           inconsistent_pose.pose.position.z);
      //       break;

      //   }           
        
      }
  }
  
      

  // if (navigator.FollowWaypoints(waypoints)) {
  //   RCLCPP_INFO(navigator.get_logger(),
  //               "Successfully sent FollowWaypoints request");
  // } else {
  //   RCLCPP_ERROR(navigator.get_logger(),
  //                "Failed to send FollowWaypoints request");
  // }

  // complete here...

  rclcpp::shutdown(); // shutdown ROS
  return 0;

}