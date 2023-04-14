#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

void poseSubscriber(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // Print the pose data
    std::cout << "Position: (" << msg->pose.pose.position.x << ", "
              << msg->pose.pose.position.y << ", " << msg->pose.pose.position.z
              << ")" << std::endl;
    std::cout << "Orientation: (" << msg->pose.pose.orientation.x << ", "
              << msg->pose.pose.orientation.y << ", " << msg->pose.pose.orientation.z
              << ", " << msg->pose.pose.orientation.w << ")" << std::endl;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pose_subscriber");

    auto sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10, poseSubscriber);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}