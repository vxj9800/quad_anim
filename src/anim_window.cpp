// Add C++ core libraries
#include <iostream>

// Add ROS headers
#include <rclcpp/rclcpp.hpp>

// Add other external libraries
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

// Add package headers
#include <quad_anim/animWindowNode.h>

// If the package name is not defined at compile time then set it to empty
#ifndef ROS_PACKAGE_NAME
#define ROS_PACKAGE_NAME ""
#endif

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
    InitWindow(screenWidth, screenHeight, animWindowPtr->get_name());

    // Create a Camera object
    Camera cam = {
        {1, 0, 0.2},       // Position
        {0, 0, 0},         // Looking at
        {0, 0, 1},         // Direction that is up in camera view
        {45},              // Field of view
        CAMERA_PERSPECTIVE // Projection type
    };

    // Create a Plane and sphere models
    Model plane = LoadModelFromMesh(GenMeshPlane(2, 2, 1, 1)); // Plane is created on XZ plane with normal pointing in +Y direction

    // Load the infinite plane shaders
    const std::string vs_source =
        #include "quad_anim/shaders/infPlane.vs"
    ;
    const std::string fs_source =
        #include "quad_anim/shaders/infPlane.fs"
    ;
    Shader infPlaneShader = LoadShaderFromMemory(vs_source.c_str(), fs_source.c_str());

    // Provide shader location for the camera position
    infPlaneShader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(infPlaneShader, "viewPos");

    // Assign infPlane shader to model
    plane.materials[0].shader = infPlaneShader;

    // Apply transformation on the plane to make the normal point to +Z
    plane.transform = MatrixRotate({1, 0, 0}, PI / 2);

    while (!WindowShouldClose() && rclcpp::ok())
    {
        // Update the window size if it has been changed
        screenWidth = GetScreenWidth(), screenHeight = GetScreenHeight();

        // Orbit the camera

        cam.target.x = animWindowPtr->baseStart[0].x;
        cam.target.y = animWindowPtr->baseStart[0].y;
        cam.target.z = animWindowPtr->baseStart[0].z;

        cam.position.x = cam.target.x + 1;
        cam.position.y = cam.target.y + 0;
        cam.position.z = cam.target.z + 0.2;

        // Send camera location to the shader
        float viewPos[3] = {cam.position.x, cam.position.y, cam.position.z};
        SetShaderValue(infPlaneShader, infPlaneShader.locs[SHADER_LOC_VECTOR_VIEW], viewPos, SHADER_UNIFORM_VEC3);

        BeginDrawing();
        ClearBackground(BLACK);

        BeginMode3D(cam);

        DrawModel(plane, Vector3Zero(), 1, WHITE);
        DrawRay(Ray{{0, 0, 0}, {0, 0, 1}}, Color({0, 0, 255, 255}));

        // Draw lines till the base of the motors
        for (size_t i = 0; i < animWindowPtr->baseStart.size(); ++i)
            DrawLine3D(animWindowPtr->baseStart[i], animWindowPtr->baseEnd[i], BLUE);

        for (size_t i = 0; i < animWindowPtr->bodyStart.size(); ++i)
            DrawLine3D(animWindowPtr->bodyStart[i], animWindowPtr->bodyEnd[i], BLUE);

        for (size_t i = 0; i < animWindowPtr->propStart.size(); ++i)
            DrawLine3D(animWindowPtr->propStart[i], animWindowPtr->propEnd[i], RED);

        EndMode3D();

        DrawFPS(10, 10);
        std::string printTime = "Sim Time: " + std::to_string(animWindowPtr->time);
        int txtWidth = MeasureText(printTime.c_str(), 24);
        DrawText(printTime.c_str(), (screenWidth - txtWidth)/2, 10, 24, LIGHTGRAY);

        EndDrawing();

        // Let ROS process subscriber callbacks
        rosExecutor.spin_some();
    }

    rclcpp::shutdown();
    UnloadShader(infPlaneShader);
    CloseWindow();
    return 0;
}
