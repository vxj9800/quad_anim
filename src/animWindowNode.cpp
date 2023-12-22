#include <quad_anim/animWindowNode.h>

animWindowNode::animWindowNode() : Node("animWindow")
{
    // Get coordinates of all the positions
    xCoords(q.data(), pB.data(), pC.data(), pD.data(), pE.data(), propDia, x.data());
    yCoords(q.data(), pB.data(), pC.data(), pD.data(), pE.data(), propDia, y.data());
    zCoords(q.data(), pB.data(), pC.data(), pD.data(), pE.data(), propDia, z.data());

    // Plot the coordinates for different lines
    matplot::vector_2d X_base(12, std::vector<double>(2)), Y_base(12, std::vector<double>(2)), Z_base(12, std::vector<double>(2));
    matplot::vector_2d X_body(12, std::vector<double>(2)), Y_body(12, std::vector<double>(2)), Z_body(12, std::vector<double>(2));
    matplot::vector_2d X_prop(12, std::vector<double>(2)), Y_prop(12, std::vector<double>(2)), Z_prop(12, std::vector<double>(2));

    for (int i = 0; i < 4; ++i)
    {
        X_base[i][0] = x[i * 2];
        X_base[i][1] = x[i * 2 + 1];
        Y_base[i][0] = y[i * 2];
        Y_base[i][1] = y[i * 2 + 1];
        Z_base[i][0] = z[i * 2];
        Z_base[i][1] = z[i * 2 + 1];
    }

    for (int i = 4; i < 8; ++i)
    {
        X_body[i][0] = x[i * 2];
        X_body[i][1] = x[i * 2 + 1];
        Y_body[i][0] = y[i * 2];
        Y_body[i][1] = y[i * 2 + 1];
        Z_body[i][0] = z[i * 2];
        Z_body[i][1] = z[i * 2 + 1];
    }

    for (int i = 8; i < 12; ++i)
    {
        X_prop[i][0] = x[i * 2];
        X_prop[i][1] = x[i * 2 + 1];
        Y_prop[i][0] = y[i * 2];
        Y_prop[i][1] = y[i * 2 + 1];
        Z_prop[i][0] = z[i * 2];
        Z_prop[i][1] = z[i * 2 + 1];
    }

    // Initialize figure with axis
    f = matplot::figure(true);
    ax = f->add_axes();
    ax->axes_aspect_ratio(1.0);

    // Plot the first frame
    basePlot = ax->plot3(X_base, Y_base, Z_base);
    for (auto line : basePlot)
        line->line_width(3).color("blue");
    bodyPlot = ax->plot3(X_body, Y_body, Z_body);
    for (auto line : bodyPlot)
        line->line_width(3).color("blue");
    propPlot = ax->plot3(X_prop, Y_prop, Z_prop);
    for (auto line : propPlot)
        line->line_width(3).color("#FFFFFF");

    matplot::xlim({-0.5, 0.5});
    matplot::ylim({-0.5, 0.5});
    matplot::zlim({-0.1, 0.5});

    // Draw the graph
    f->draw();

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
    // Get coordinates of all the positions
    xCoords(q.data(), pB.data(), pC.data(), pD.data(), pE.data(), propDia, x.data());
    yCoords(q.data(), pB.data(), pC.data(), pD.data(), pE.data(), propDia, y.data());
    zCoords(q.data(), pB.data(), pC.data(), pD.data(), pE.data(), propDia, z.data());

    // Plot the coordinates for different lines
    matplot::vector_2d X_base(12, std::vector<double>(2)), Y_base(12, std::vector<double>(2)), Z_base(12, std::vector<double>(2));
    matplot::vector_2d X_body(12, std::vector<double>(2)), Y_body(12, std::vector<double>(2)), Z_body(12, std::vector<double>(2));
    matplot::vector_2d X_prop(12, std::vector<double>(2)), Y_prop(12, std::vector<double>(2)), Z_prop(12, std::vector<double>(2));

    for (int i = 0; i < 4; ++i)
    {
        X_base[i][0] = x[i * 2];
        X_base[i][1] = x[i * 2 + 1];
        Y_base[i][0] = y[i * 2];
        Y_base[i][1] = y[i * 2 + 1];
        Z_base[i][0] = z[i * 2];
        Z_base[i][1] = z[i * 2 + 1];
    }

    for (int i = 4; i < 8; ++i)
    {
        X_body[i][0] = x[i * 2];
        X_body[i][1] = x[i * 2 + 1];
        Y_body[i][0] = y[i * 2];
        Y_body[i][1] = y[i * 2 + 1];
        Z_body[i][0] = z[i * 2];
        Z_body[i][1] = z[i * 2 + 1];
    }

    for (int i = 8; i < 12; ++i)
    {
        X_prop[i][0] = x[i * 2];
        X_prop[i][1] = x[i * 2 + 1];
        Y_prop[i][0] = y[i * 2];
        Y_prop[i][1] = y[i * 2 + 1];
        Z_prop[i][0] = z[i * 2];
        Z_prop[i][1] = z[i * 2 + 1];
    }

    // Update position vectors from quad body to motor base
    for (size_t i = 0; i < X_base.size(); ++i)
    {
        basePlot[i]->x_data(X_base[i]);
        basePlot[i]->y_data(Y_base[i]);
        basePlot[i]->z_data(Z_base[i]);
    }

    // Update position vectors from motor base to motor body
    for (size_t i = 0; i < X_body.size(); ++i)
    {
        bodyPlot[i]->x_data(X_body[i]);
        bodyPlot[i]->y_data(Y_body[i]);
        bodyPlot[i]->z_data(Z_body[i]);
    }

    // Update position vectors representing propellers
    for (size_t i = 0; i < X_prop.size(); ++i)
    {
        propPlot[i]->x_data(X_prop[i]);
        propPlot[i]->y_data(Y_prop[i]);
        propPlot[i]->z_data(Z_prop[i]);
    }

    // Add a time-stamp in title
    ax->title("t = " + matplot::num2str(msg.sec + msg.nanosec * 1e-9) + " sec");

    // Update the graph
    ax->draw();
}