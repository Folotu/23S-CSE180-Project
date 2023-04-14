#include <fstream>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sstream>
#include <iostream>
#include <cmath>
#include <rclcpp/clock.hpp>


void mapSubscriber(const std::string& filename, nav_msgs::msg::OccupancyGrid& occupancy_grid)
{
    // Open the file for reading
    std::ifstream infile(filename);
    if (!infile.is_open())
    {
        std::cerr << "Error: could not open file " << filename << std::endl;
        return;
    }

    // Read the map dimensions from the file
    unsigned int width, height;
    infile >> width >> height;

    // Resize the occupancy grid to the map dimensions
    occupancy_grid.info.width = width;
    occupancy_grid.info.height = height;
    occupancy_grid.data.resize(width * height);

    // Read the map data from the file and create the modified data vector
    std::vector<int8_t> modified_data(width * height);
    for (unsigned int row = 0; row < height; row++)
    {
        for (unsigned int col = 0; col < width; col++)
        {
            int value;
            infile >> value;
            if (value == 0 || value == 100)
            {
                modified_data[row * width + col] = static_cast<int8_t>(value);
            }
            else
            {
                modified_data[row * width + col] = -1;
            }
        }
    }

    // Set the occupancy grid data to the modified data vector
    occupancy_grid.data = modified_data;

    // Close the file
    infile.close();
}



int main() {
    // Load the occupancy grid from file
    std::string filename = "/home/speet/Documents/23S-CSE180-Project/FPWorkSpace/src/finalproject/src/extraPostMapOutputAll.txt";
    int width = 0;
    int height = 0;
    std::vector<int8_t> data;

    std::ifstream file(filename);
    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            int value;
            while (iss >> value) {
                data.push_back(value);
            }
            if (width == 0) {
                width = data.size();
            }
            height++;
        }
        file.close();
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
        return 1;
    }

    // Create the occupancy grid message
    nav_msgs::msg::OccupancyGrid occupancy_grid;
    occupancy_grid.header.frame_id = "map";
    occupancy_grid.header.stamp = rclcpp::Clock().now();
    occupancy_grid.info.resolution = 0.1; // Set the resolution of the occupancy grid
    occupancy_grid.info.width = width;
    occupancy_grid.info.height = height;
    occupancy_grid.info.origin.position.x = 0.0; // Set the origin of the occupancy grid
    occupancy_grid.info.origin.position.y = 0.0;
    occupancy_grid.info.origin.position.z = 0.0;
    occupancy_grid.info.origin.orientation.x = 0.0;
    occupancy_grid.info.origin.orientation.y = 0.0;
    occupancy_grid.info.origin.orientation.z = 0.0;
    occupancy_grid.info.origin.orientation.w = 1.0;
    occupancy_grid.data = data;

    // Find the cell that the point belongs to
    double x = 0;
    double y = -2;
    double z = 0.25;
    double resolution = occupancy_grid.info.resolution;
    int x_index = round((x - occupancy_grid.info.origin.position.x) / resolution);
    int y_index = round((y - occupancy_grid.info.origin.position.y) / resolution);
    int z_index = round((z - occupancy_grid.info.origin.position.z) / resolution);
    int cell_index = y_index * occupancy_grid.info.width + x_index;
    int cell_value = occupancy_grid.data[cell_index];

    // If the cell is occupied or unknown, return null pose
    geometry_msgs::msg::Pose pose;
    if (cell_value == 100 || cell_value == -1) {
        pose = geometry_msgs::msg::Pose();
    } else {
        // Calculate the position of the cell
        double x_pos = x_index * resolution + occupancy_grid.info.origin.position.x;
        double y_pos = y_index * resolution + occupancy_grid.info.origin.position.y;
        double z_pos = z_index * resolution + occupancy_grid.info.origin.position.z;
        pose.position.x = x_pos;
        pose.position.y = y_pos;
        pose.position.z = z_pos;

        // Set the orientation to zero
        pose.orientation.w = 1.0;
    }

    // Print the result
    if (pose.position.x == 0 && pose.position.y == 0 && pose.position.z == 0) {
        std::cout << "The point is in an occupied or unknown cell." << std::endl;
    } else {
        std::cout << "The pose of the point is:" << std::endl;
        std::cout << "position: (" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << ")" << std::endl;
        std::cout << "orientation: (" << pose.orientation.x << ", " << pose.orientation.y << ", " << pose.orientation.z << ", " << pose.orientation.w << ")" << std::endl;
    }

    return 0;
}