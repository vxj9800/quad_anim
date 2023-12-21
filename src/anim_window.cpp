// Add C++ core libraries
#include <iostream>

// Add ROS headers
#include <rclcpp/rclcpp.hpp>

// Add other external libraries

// Add package headers
#include <quad_anim/animWindowNode.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<animWindowNode>());
    rclcpp::shutdown();
    return 0;
}
