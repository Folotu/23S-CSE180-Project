#include <chrono>
#include <fstream>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdlib.h> 
#include <stdio.h> 
#include <linux/limits.h>
std::chrono::system_clock::time_point last_update_time;

void mapSubscriber(const nav_msgs::msg::OccupancyGrid::SharedPtr msg,
                   std::ofstream &outfile) {

  // Check if the map is updated
  auto current_time = std::chrono::system_clock::now();
  auto time_since_last_update = current_time - last_update_time;
  const auto update_threshold = std::chrono::seconds(5); // adjust as needed

  if (time_since_last_update > update_threshold) {
    last_update_time = current_time;

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

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("map_subscriber");
  
  char resolved_path[PATH_MAX]; 
  realpath("../", resolved_path); 
// Create a relative path to output to
  const std::string relative_path = "/src/finalproject/src/txtFolders/GlobalCostMapOutput.txt";

  std::string str(resolved_path);

  std::ofstream outfile(str+relative_path);

   // Check if the file was opened successfully
    if (!outfile.is_open()) {
        std::cerr << "Failed to open output file." << std::endl;
        return 1;
    }


  outfile << " weight fr height fr\n";

  auto sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap/costmap", rclcpp::QoS(rclcpp::KeepLast(10)),
      [&](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        mapSubscriber(msg, outfile);
      });

  rclcpp::spin(node);

  rclcpp::shutdown();

  // Close the output file
  outfile.close();
  return 0;
}

// #include <fstream>
// #include <map_msgs/msg/occupancy_grid_update.hpp>
// #include <rclcpp/rclcpp.hpp>

// void costmapUpdateCallback(
//     const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg,
//     std::ofstream &outfile) {
//   // Print the header information
//   outfile << "Header: " << std::endl;
//   outfile << "  Frame ID: " << msg->header.frame_id << std::endl;
//   // outfile << "  Seq: " << msg->header.seq << std::endl;
//   outfile << "  Stamp: " << msg->header.stamp.sec << "."
//           << msg->header.stamp.nanosec << std::endl;

//   // Print the origin information
//   outfile << "Origin: (" << msg->x << ", " << msg->y << ")" << std::endl;

//   // Print the data information
//   outfile << "Data: " << std::endl;
//   for (unsigned int i = 0; i < msg->data.size(); ++i) {
//     outfile << msg->data[i] << ", ";
//   }
//   outfile << std::endl;
// }

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);

//   auto node = rclcpp::Node::make_shared("costmap_subscriber");

//   std::ofstream
//   outfile("/home/user/FinalProjFold/FPWorkSpace/src/finalproject/"
//                         "src/txtFolders/GlobalCostMapUpdatesOutput.txt");

//   auto subscription =
//       node->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
//           "global_costmap/costmap_updates", rclcpp::SystemDefaultsQoS(),
//           [&](const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg) {
//             costmapUpdateCallback(msg, outfile);
//           });

//   //   auto subscription =
//   node->create_subscription<nav2_msgs::msg::Costmap>(
//   //   "/global_costmap/costmap_updates", rclcpp::QoS(rclcpp::KeepLast(2)),
//   //   [&](const nav2_msgs::msg::Costmap::SharedPtr msg) {
//   //     costmapUpdateCallback(msg, outfile);
//   //   });

//   rclcpp::spin(node);

//   rclcpp::shutdown();
//   return 0;
// }
