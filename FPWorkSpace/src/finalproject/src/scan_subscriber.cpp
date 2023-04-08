#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

void scanSubscriber(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                    std::ofstream &outfile) {
  // Print the range data
  for (long unsigned int i = 0; i < msg->ranges.size(); i++) {
    std::cout << "Range " << i << ": " << msg->ranges[i] << std::endl;
    if (i == 1) {
      outfile << "Front of Robot, Range " << i << ": " << msg->ranges[i]
              << std::endl;
    } else if (i == 180) {
      outfile << "Left of Robot, Range " << i << ": " << msg->ranges[i]
              << std::endl;
    } else if (i == 540) {
      outfile << "Right of Robot, Range " << i << ": " << msg->ranges[i]
              << std::endl;
    } else if (i == 359) {
      outfile << "Back of Robot, Range " << i << ": " << msg->ranges[i]
              << std::endl;
    }
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Open the output file
  std::ofstream outfile("/home/user/FPWorkSpace/src/finalproject/src/"
                        "txtFolders/scanSubOutput.txt");

  auto node = rclcpp::Node::make_shared("scan_subscriber");

  // auto sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
  //     "/scan", 10, scanSubscriber);

  auto sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, [&](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        scanSubscriber(msg, outfile);
      });

  rclcpp::spin(node);

  rclcpp::shutdown();
  outfile.close();
  return 0;
}