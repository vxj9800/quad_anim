// Add package headers
extern "C"
{
#include <xCoords.h>
#include <yCoords.h>
#include <zCoords.h>
}

// Add ROS libraries
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <builtin_interfaces/msg/time.hpp>

// Add other external libraries
#include <matplot/matplot.h>

#ifndef __ANIM_WINDOW_HEADER__
#define __ANIM_WINDOW_HEADER__

class animWindowNode : public rclcpp::Node
{
public:
    animWindowNode();
    // ~animWindowNode();

private:
    // Variables for subscribers
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr bodyPose_Sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr motB_Sub, motC_Sub, motD_Sub, motE_Sub;
    rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr tick_Sub;

    // Subscriber callback functions
    void bodyPose_Cb(const geometry_msgs::msg::Pose::SharedPtr msg);
    void motB_Cb(const std_msgs::msg::Float64::SharedPtr msg);
    void motC_Cb(const std_msgs::msg::Float64::SharedPtr msg);
    void motD_Cb(const std_msgs::msg::Float64::SharedPtr msg);
    void motE_Cb(const std_msgs::msg::Float64::SharedPtr msg);
    void tick_Cb(const builtin_interfaces::msg::Time msg);

    // Variables for plot
    matplot::figure_handle f;
    matplot::axes_handle ax;
    std::vector<matplot::line_handle> basePlot, bodyPlot, propPlot;

    // Constants for motor positions //
    std::vector<double> pB = {0.08, 0.08, 0.015};   // pB = [lB; wB; hB];
    std::vector<double> pC = {-0.08, 0.08, 0.015};  // pC = [lC; wC; hC];
    std::vector<double> pD = {-0.08, -0.08, 0.015}; // pD = [lD; wD; hD];
    std::vector<double> pE = {0.08, -0.08, 0.015};  // pE = [lE; wE; hE];

    // Define Propeller data //
    double propDia = 0.127; // Propeller Diameter

    // Define generalized coordinates to check //
    std::vector<double> q = {0, 0, 0, 1, 0, 0, 0, M_PI_4, M_PI_2 + M_PI_4, M_PI_4, M_PI_2 + M_PI_4};

    // Get the x, y and z coordinate values //
    std::vector<double> x = std::vector<double>(12 * 2);
    std::vector<double> y = std::vector<double>(12 * 2);
    std::vector<double> z = std::vector<double>(12 * 2);
};

#endif // __ANIM_WINDOW_HEADER__