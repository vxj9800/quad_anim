#include <quad_anim/animWindowNode.h>

animWindowNode::animWindowNode() : Node("animWindow")
{
    // Get coordinates of all the positions
    xCoords(q.data(), pB.data(), pC.data(), pD.data(), pE.data(), propDia, x.data());
    yCoords(q.data(), pB.data(), pC.data(), pD.data(), pE.data(), propDia, y.data());
    zCoords(q.data(), pB.data(), pC.data(), pD.data(), pE.data(), propDia, z.data());

    // Initialize variable that hold line information
    baseStart.resize(4);
    baseEnd.resize(4);
    bodyStart.resize(4);
    bodyEnd.resize(4);
    propStart.resize(4);
    propEnd.resize(4);

    // Convert initial coordinate data to line data
    convCoordToLine();

    // Initialize the subscribers
    bodyPose_Sub = this->create_subscription<geometry_msgs::msg::Pose>("quadPose", rclcpp::SensorDataQoS(), std::bind(&animWindowNode::bodyPose_Cb, this, std::placeholders::_1));
    motB_Sub = this->create_subscription<std_msgs::msg::Float64>("motB", rclcpp::SensorDataQoS(), std::bind(&animWindowNode::motB_Cb, this, std::placeholders::_1));
    motC_Sub = this->create_subscription<std_msgs::msg::Float64>("motC", rclcpp::SensorDataQoS(), std::bind(&animWindowNode::motC_Cb, this, std::placeholders::_1));
    motD_Sub = this->create_subscription<std_msgs::msg::Float64>("motD", rclcpp::SensorDataQoS(), std::bind(&animWindowNode::motD_Cb, this, std::placeholders::_1));
    motE_Sub = this->create_subscription<std_msgs::msg::Float64>("motE", rclcpp::SensorDataQoS(), std::bind(&animWindowNode::motE_Cb, this, std::placeholders::_1));
    tick_Sub = this->create_subscription<builtin_interfaces::msg::Time>("tick", rclcpp::SensorDataQoS(), std::bind(&animWindowNode::tick_Cb, this, std::placeholders::_1));
}

void animWindowNode::bodyPose_Cb(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    // Update the body position
    q[0] = msg->position.x;
    q[1] = msg->position.x;
    q[2] = msg->position.x;

    // Update the body orientation
    q[3] = msg->orientation.w;
    q[4] = msg->orientation.x;
    q[5] = msg->orientation.y;
    q[6] = msg->orientation.z;
}

void animWindowNode::motB_Cb(const std_msgs::msg::Float64::SharedPtr msg)
{
    q[7] = msg->data;
}

void animWindowNode::motC_Cb(const std_msgs::msg::Float64::SharedPtr msg)
{
    q[8] = msg->data;
}

void animWindowNode::motD_Cb(const std_msgs::msg::Float64::SharedPtr msg)
{
    q[9] = msg->data;
}

void animWindowNode::motE_Cb(const std_msgs::msg::Float64::SharedPtr msg)
{
    q[10] = msg->data;
}

void animWindowNode::tick_Cb(const builtin_interfaces::msg::Time msg)
{
    // Update the simulation time
    time = msg.sec + msg.nanosec * 1e-9;
    
    // Get coordinates of all the positions
    xCoords(q.data(), pB.data(), pC.data(), pD.data(), pE.data(), propDia, x.data());
    yCoords(q.data(), pB.data(), pC.data(), pD.data(), pE.data(), propDia, y.data());
    zCoords(q.data(), pB.data(), pC.data(), pD.data(), pE.data(), propDia, z.data());

    // Convert new coordinate data to line data
    convCoordToLine();
}

void animWindowNode::convCoordToLine()
{
    // Transfer the coordinates to different lines
    for (int i = 0; i < 4; ++i)
    {
        baseStart[i] = Vector3{(float)x[i * 2], (float)y[i * 2], (float)z[i * 2]};
        baseEnd[i] = Vector3{(float)x[i * 2 + 1], (float)y[i * 2 + 1], (float)z[i * 2 + 1]};
    }

    for (int i = 4; i < 8; ++i)
    {
        bodyStart[i - 4] = Vector3{(float)x[i * 2], (float)y[i * 2], (float)z[i * 2]};
        bodyEnd[i - 4] = Vector3{(float)x[i * 2 + 1], (float)y[i * 2 + 1], (float)z[i * 2 + 1]};
    }

    for (int i = 8; i < 12; ++i)
    {
        propStart[i - 8] = Vector3{(float)x[i * 2], (float)y[i * 2], (float)z[i * 2]};
        propEnd[i - 8] = Vector3{(float)x[i * 2 + 1], (float)y[i * 2 + 1], (float)z[i * 2 + 1]};
    }
}