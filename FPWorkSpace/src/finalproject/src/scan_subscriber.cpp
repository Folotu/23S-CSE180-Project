#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

void scanSubscriber(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Print the range data
    for (long unsigned int i = 0; i < msg->ranges.size(); i++)
    {
        std::cout << "Range " << i << ": " << msg->ranges[i] << std::endl;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("scan_subscriber");

    auto sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, scanSubscriber);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}