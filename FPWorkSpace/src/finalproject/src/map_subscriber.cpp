#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <fstream>

void mapSubscriber(const nav_msgs::msg::OccupancyGrid::SharedPtr msg, std::ofstream& outfile)
{
    // // Print the map data
    // for (unsigned int i = 0; i < msg->info.width * msg->info.height; i++)
    // {
    //     outfile << static_cast<int>(msg->data[i]) << " ";
    // }
    // outfile << std::endl;

    // // Print the map data row by row
    // for (unsigned int row = 0; row < msg->info.height; row++)
    // {
    //     for (unsigned int col = 0; col < msg->info.width; col++)
    //     {
    //         outfile << static_cast<int>(msg->data[row * msg->info.width + col]) << " ";
    //     }
    //     outfile << std::endl;
    // }

    // Print the map data row by row
    for (unsigned int row = 0; row < msg->info.height; row++)
    {
        for (unsigned int col = 0; col < msg->info.width; col++)
        {
            int value = msg->data[row * msg->info.width + col];
            if (value == 0 || value == 100)
            {
                outfile << value << " ";
            }
            else
            {
                outfile << "  " << " ";
            }
        }
        outfile << std::endl;
    }
}




int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("map_subscriber");

    // Open the output file
    std::ofstream outfile("/mnt/hgfs/23S-CSE180-Project/FPWorkSpace/src/finalproject/src/mapOutputjust0_100.txt");

     auto sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(rclcpp::KeepLast(10)),
        [&](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            mapSubscriber(msg, outfile);
        });

    rclcpp::spin(node);

    rclcpp::shutdown();

    // Close the output file
    outfile.close();
    return 0;
}