#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

void mapSubscriber(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    // Print the map data
    for (unsigned int i = 0; i < msg->info.width * msg->info.height; i++)
    {
        std::cout << msg->data[i] << " ";
    }
    std::cout << std::endl;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("map_subscriber");

    auto sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, mapSubscriber);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}