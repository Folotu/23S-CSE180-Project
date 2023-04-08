#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

void tfSubscriber(const tf2_msgs::msg::TFMessage::SharedPtr msg,
                  std::ofstream &outfile) {
  // Print the transform data
  //   for (const auto &transform : msg->transforms) {
  //     std::cout << "Transform from " << transform.child_frame_id << " to "
  //               << transform.header.frame_id << std::endl;
  //     std::cout << "Translation: (" << transform.transform.translation.x <<
  //     ", "
  //               << transform.transform.translation.y << ", "
  //               << transform.transform.translation.z << ")" << std::endl;
  //     std::cout << "Rotation: (" << transform.transform.rotation.x << ", "
  //               << transform.transform.rotation.y << ", "
  //               << transform.transform.rotation.z << ", "
  //               << transform.transform.rotation.w << ")" << std::endl;
  //   }

  for (const auto &transform : msg->transforms) {
    std::cout << "Transform from " << transform.child_frame_id << " to "
              << transform.header.frame_id << std::endl;
    outfile << "Transform from " << transform.child_frame_id << " to "
            << transform.header.frame_id << std::endl;
    outfile << "Translation: (" << transform.transform.translation.x << ", "
            << transform.transform.translation.y << ", "
            << transform.transform.translation.z << ")" << std::endl;
    outfile << "Rotation: (" << transform.transform.rotation.x << ", "
            << transform.transform.rotation.y << ", "
            << transform.transform.rotation.z << ", "
            << transform.transform.rotation.w << ")" << std::endl;
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tf_subscriber");
  std::ofstream outfile("/home/speet/Documents/23S-CSE180-Project/FPWorkSpace/"
                        "src/finalproject/src/txtFolders/tfstatFR.txt");

  // auto sub = node->create_subscription<tf2_msgs::msg::TFMessage>(
  //     "/tf_static", 10, tfSubscriber);

  auto sub = node->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", 10, [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        tfSubscriber(msg, outfile);
      });

  rclcpp::spin(node);

  rclcpp::shutdown();
  outfile.close();
  return 0;
}