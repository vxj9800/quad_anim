// Add C++ core libraries
#include <iostream>

// Add ROS headers
#include <rclcpp/rclcpp.hpp>

// Add other external libraries
#include <raylib.h>

// Add package headers
#include <quad_anim/animWindowNode.h>

int main(int argc, char **argv)
{
    // Some initialization.
    rclcpp::init(argc, argv);

    // Initialize the animWindowNode node
    rclcpp::executors::MultiThreadedExecutor rosExecutor;
    std::shared_ptr<animWindowNode> animWindowNodePtr(new animWindowNode());
    rosExecutor.add_node(animWindowNodePtr);

    // Initialize window
    InitWindow(animWindowNodePtr->screenHeight, animWindowNodePtr->screenHeight, "raylib [core] example - basic window");

    // Define the camera to look into our 3d world
    Camera camera = {0};
    camera.position = (Vector3){0.0f, 10.0f, 10.0f};
    camera.target = (Vector3){0.0f, 0.0f, 0.0f};
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    while (!WindowShouldClose() && rclcpp::ok())
    {
        // Allow ROS to finish publishing
        rosExecutor.spin_some();
        BeginDrawing();
        EndDrawing();
    }

    rclcpp::shutdown();
    CloseWindow();
    return 0;
}
