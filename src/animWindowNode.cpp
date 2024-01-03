#include <quad_anim/animWindowNode.h>

animWindowNode::animWindowNode() : Node("animWindow")
{
    // Initialize variable that hold line information
    baseStart.resize(4);
    baseEnd.resize(4);
    bodyStart.resize(4);
    bodyEnd.resize(4);
    propStart.resize(4);
    propEnd.resize(4);

    // Initialize the subscribers
    bodyPose_Sub = this->create_subscription<geometry_msgs::msg::Pose>("quadPose", 1, std::bind(&animWindowNode::bodyPose_Cb, this, std::placeholders::_1));
    motB_Sub = this->create_subscription<std_msgs::msg::Float64>("motB", 1, std::bind(&animWindowNode::motB_Cb, this, std::placeholders::_1));
    motC_Sub = this->create_subscription<std_msgs::msg::Float64>("motC", 1, std::bind(&animWindowNode::motC_Cb, this, std::placeholders::_1));
    motD_Sub = this->create_subscription<std_msgs::msg::Float64>("motD", 1, std::bind(&animWindowNode::motD_Cb, this, std::placeholders::_1));
    motE_Sub = this->create_subscription<std_msgs::msg::Float64>("motE", 1, std::bind(&animWindowNode::motE_Cb, this, std::placeholders::_1));
}

void animWindowNode::bodyPose_Cb(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    // Update the body position
    q[0] = msg->position.x;
    q[1] = msg->position.y;
    q[2] = msg->position.z;

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

double animWindowNode::updateLineData()
{
    // Get coordinates of all the positions
    xCoords(q.data(), pB.data(), pC.data(), pD.data(), pE.data(), propDia, x.data());
    yCoords(q.data(), pB.data(), pC.data(), pD.data(), pE.data(), propDia, y.data());
    zCoords(q.data(), pB.data(), pC.data(), pD.data(), pE.data(), propDia, z.data());

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

    // Get current node time and return it
    rclcpp::Time timeNow = now();
    return timeNow.seconds() + timeNow.nanoseconds() * 1e-9;
}