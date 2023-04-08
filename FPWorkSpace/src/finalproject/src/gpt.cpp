#include <iostream>
#include <laser_geometry/laser_geometry.hpp>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_listener.h>

class MapComparisonNode : public rclcpp::Node {
public:
  MapComparisonNode()
      : Node("map_comparison_node"), tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&MapComparisonNode::scan_callback, this,
                  std::placeholders::_1));
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        std::bind(&MapComparisonNode::map_callback, this,
                  std::placeholders::_1));
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    if (!map_received_)
      return;

    // Convert LaserScan to PointCloud
    sensor_msgs::msg::PointCloud2 cloud_msg;
    projector_.projectLaser(*scan_msg, cloud_msg);

    // Transform PointCloud to map frame
    try {
      cloud_in_map_frame_ = tf_buffer_.transform(
          cloud_msg, map_.header.frame_id, tf2::durationFromSec(0.1));
      compare_map_and_pointcloud();
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(
          this->get_logger(),
          "Could not transform PointCloud from frame '%s' to frame '%s'.",
          cloud_msg.header.frame_id.c_str(), map_.header.frame_id.c_str());
    }
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg) {
    map_ = *map_msg;
    map_received_ = true;
  }

  //   void compare_map_and_pointcloud()
  //   {
  //     // Convert OccupancyGrid map data to a grid representation and compare
  //     it with the PointCloud data
  //     // TODO: Implement the comparison method

  //   }

  void compare_map_and_pointcloud() {
    // Create a 2D grid representation of the OccupancyGrid map
    int width = map_.info.width;
    int height = map_.info.height;
    double resolution = map_.info.resolution;
    std::vector<std::vector<bool>> grid(width,
                                        std::vector<bool>(height, false));

    for (int i = 0; i < width; ++i) {
      for (int j = 0; j < height; ++j) {
        int index = i + j * width;
        grid[i][j] = map_.data[index] > 0;
      }
    }

    // Count the number of points in the PointCloud that correspond to occupied
    // cells in the OccupancyGrid map
    int num_matching_points = 0;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_in_map_frame_,
                                                        "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_in_map_frame_,
                                                        "y");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
      double x = *iter_x - map_.info.origin.position.x;
      double y = *iter_y - map_.info.origin.position.y;
      int i = std::floor(x / resolution);
      int j = std::floor(y / resolution);

      if (i >= 0 && i < width && j >= 0 && j < height && grid[i][j]) {
        num_matching_points++;
      }
    }

    // Calculate the percentage of matching points
    double matching_points_percentage =
        static_cast<double>(num_matching_points) / cloud_in_map_frame_.width *
        100.0;

    RCLCPP_INFO(this->get_logger(), "Matching points: %d/%d (%.2f%%)",
                num_matching_points, cloud_in_map_frame_.width,
                matching_points_percentage);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;

  bool map_received_ = false;
  nav_msgs::msg::OccupancyGrid map_;
  sensor_msgs::msg::PointCloud2 cloud_in_map_frame_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  laser_geometry::LaserProjection projector_;
};

#include "rclcpp/rclcpp.hpp"
// #include "map_comparison_node.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapComparisonNode>());
  rclcpp::shutdown();
  return 0;
}
