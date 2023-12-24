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

    // Initialize the ROS executor
    rclcpp::executors::MultiThreadedExecutor rosExecutor;

    // Get a shared pointer for a node object
    std::shared_ptr<animWindowNode> animWindowPtr = std::make_shared<animWindowNode>();
    rosExecutor.add_node(animWindowPtr);

    // Initialize window
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    unsigned int screenWidth = 1280, screenHeight = 720;
    InitWindow(screenWidth, screenHeight, "raylib [core] example - basic window");

    // Define the camera to look into our 3d world
    Camera camera;
    camera.position = Vector3({0.0f, 10.0f, 10.0f});
    camera.target = Vector3({0.0f, 0.0f, 0.0f});
    camera.up = Vector3({0.0f, 1.0f, 0.0f});
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    while (!WindowShouldClose() && rclcpp::ok())
    {
        BeginDrawing();
        ClearBackground(BLACK);

        BeginMode3D(camera);

        DrawGrid(10, 1.0f); // Draw a grid

        // Draw lines till the base of the motors
        for (size_t i = 0; i < animWindowPtr->baseStart.size(); ++i)
            DrawLine3D(animWindowPtr->baseStart[i], animWindowPtr->baseEnd[i], BLUE);

        for (size_t i = 0; i < animWindowPtr->bodyStart.size(); ++i)
            DrawLine3D(animWindowPtr->bodyStart[i], animWindowPtr->bodyEnd[i], BLUE);

        for (size_t i = 0; i < animWindowPtr->propStart.size(); ++i)
            DrawLine3D(animWindowPtr->propStart[i], animWindowPtr->propEnd[i], RED);

        EndMode3D();

        DrawFPS(10, 10);
        EndDrawing();
        
        // Let ROS process subscriber callbacks
        rosExecutor.spin_some();
    }

    rclcpp::shutdown();
    CloseWindow();
    return 0;
}
