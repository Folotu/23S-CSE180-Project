#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include <finalproject/navigation.hpp>
#include <iostream>
#include <iterator>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include <fstream>
#include <linux/limits.h>
#include <stdio.h>
#include <stdlib.h>

std::string Getrealpath() {
  char resolved_path[PATH_MAX];
  realpath("../", resolved_path);
  return resolved_path;
}

double total_x = 0.0;
double total_y = 0.0;
int counter = 0;

// Create a relative path to output to
const std::string relative_path =
    "/FPWorkSpace/src/finalproject/src/txtFolders/GlobalCostMapOutput.txt";

std::string str(Getrealpath());

// const std::string relative_path =
//     "/home/user/FinalProjFold/FPWorkSpace/src/finalproject/src/txtFolders";
// std::ofstream outfile(relative_path);
std::ofstream outfile(str + relative_path);

nav_msgs::msg::OccupancyGrid::SharedPtr firstGlobalCostMapMsg;
std::chrono::system_clock::time_point last_update_time;
std::vector<std::pair<int, int>>
CompareCostmaps(const nav_msgs::msg::OccupancyGrid::SharedPtr &latest_costmap,
                const nav_msgs::msg::OccupancyGrid::SharedPtr &first_costmap,
                Navigator &navy);
geometry_msgs::msg::PoseStamped GlobalCostmapIndicesToPose(
    int row, int col, const nav_msgs::msg::OccupancyGrid::SharedPtr &costmap);

void mapSubscriber(const nav_msgs::msg::OccupancyGrid::SharedPtr msg,
                   Navigator &navy, std::ofstream &outfile) {

  if (!firstGlobalCostMapMsg) {
    firstGlobalCostMapMsg = msg;

  } else {
    // Check if the map is updated
    auto current_time = std::chrono::system_clock::now();
    auto time_since_last_update = current_time - last_update_time;
    const auto update_threshold = std::chrono::seconds(5); // adjust as needed

    if (time_since_last_update > update_threshold) {
      last_update_time = current_time;
      auto inconsistentCells =
          CompareCostmaps(msg, firstGlobalCostMapMsg, navy);

      for (const auto &cell : inconsistentCells) {
        auto pose_stamped =
            GlobalCostmapIndicesToPose(cell.first, cell.second, msg);
        RCLCPP_WARN(navy.get_logger(),
                    "Found inconsistency at position (x: %f, y: %f)",
                    pose_stamped.pose.position.x, pose_stamped.pose.position.y);
        total_x += pose_stamped.pose.position.x;
        total_y += pose_stamped.pose.position.y;
        counter = counter + 1;
        if (counter >= 5) {
          double average_x = total_x / counter;
          double average_y = total_y / counter;

          RCLCPP_WARN(navy.get_logger(),
                      "Average inconsistency position (x: %f, y: %f)",
                      average_x, average_y);
          navy.CancelTask();
          break;
        }
      }
      // Print the map data row by row
      outfile << msg->info.height << " height fr\n";
      outfile << msg->info.width << " width fr\n";
      outfile << msg->info.resolution << " resolution fr\n";
      for (unsigned int row = 0; row < msg->info.height; row++) {
        for (unsigned int col = 0; col < msg->info.width; col++) {
          outfile << static_cast<int>(msg->data[row * msg->info.width + col])
                  << " ";
        }
        outfile << std::endl;
      }
    }
  }
}

std::vector<std::pair<int, int>>
CompareCostmaps(const nav_msgs::msg::OccupancyGrid::SharedPtr &latest_costmap,
                const nav_msgs::msg::OccupancyGrid::SharedPtr &first_costmap,
                Navigator &navy) {
  std::vector<std::pair<int, int>> inconsistent_cells;

  if (!first_costmap || !latest_costmap) {
    RCLCPP_WARN(
        navy.get_logger(),
        "Failed to compare costmaps: Local or global costmap is missing");
    return inconsistent_cells;
  }

  // Check if both costmaps have the same dimensions
  if (first_costmap->info.width != latest_costmap->info.width ||
      first_costmap->info.height != latest_costmap->info.height) {
    RCLCPP_WARN(
        navy.get_logger(),
        "Failed to compare costmaps: The costmaps have different dimensions");
    return inconsistent_cells;
  }

  // Define a threshold for the absolute difference between cell values
  int threshold = 10; // Adjust this value based on your requirements

  for (unsigned int row = 0; row < latest_costmap->info.height; ++row) {
    for (unsigned int col = 0; col < latest_costmap->info.width; ++col) {
      int index = row * latest_costmap->info.width + col;

      // Ignore changes to old values that were -1
      if (static_cast<int>(first_costmap->data[index]) == -1 ||
          static_cast<int>(latest_costmap->data[index]) == -1) {
        continue;
      }

      int old_value = static_cast<int>(first_costmap->data[index]);
      int new_value = static_cast<int>(latest_costmap->data[index]);
      int value_diff = std::abs(new_value - old_value);

      if (value_diff > threshold && new_value > old_value &&
          (new_value == 100 || new_value == 99)) {
        inconsistent_cells.emplace_back(row, col);
        outfile << "New: " << static_cast<int>(latest_costmap->data[index])
                << ", Old: " << static_cast<int>(first_costmap->data[index])
                << "\n";

        // Stop processing if we have found 5 or more inconsistent cells
        if (inconsistent_cells.size() >= 5) {
          return inconsistent_cells;
        }
      }
    }
  }

  return inconsistent_cells;
}

geometry_msgs::msg::PoseStamped GlobalCostmapIndicesToPose(
    int row, int col, const nav_msgs::msg::OccupancyGrid::SharedPtr &costmap) {
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = costmap->header.frame_id;
  pose_stamped.header.stamp = costmap->header.stamp;

  double resolution = costmap->info.resolution;
  pose_stamped.pose.position.x =
      col * resolution + costmap->info.origin.position.x + resolution / 2.0;
  pose_stamped.pose.position.y =
      row * resolution + costmap->info.origin.position.y + resolution / 2.0;
  pose_stamped.pose.orientation.w =
      1.0; // No orientation information for individual cells

  return pose_stamped;
}

int main(int argc, char **argv) {

  rclcpp::init(argc, argv); // initialize ROS
  Navigator navigator(true,
                      false); // create node with debug info but not verbose

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

  auto node = rclcpp::Node::make_shared("map_subscriber");

  auto sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap/costmap", rclcpp::QoS(rclcpp::KeepLast(5)),
      [&](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        mapSubscriber(msg, navigator, outfile);
      });

  std::cout << str << std::endl;
  // spin in place of 90 degrees (default parameter)
  navigator.Spin();
  while (!navigator.IsTaskComplete()) {
    rclcpp::spin_some(node);
  }

  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    geometry_msgs::msg::Pose::SharedPtr goal_pos =
        std::make_shared<geometry_msgs::msg::Pose>();
    *goal_pos = waypoints[i];
    navigator.GoToPose(goal_pos);

    while (!navigator.IsTaskComplete()) {
      rclcpp::spin_some(node);
    }
  }

  rclcpp::shutdown(); // shutdown ROS
  outfile.close();
  return 0;
}