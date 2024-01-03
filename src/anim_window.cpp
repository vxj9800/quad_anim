// Add C++ core libraries
#include <iostream>

// Add ROS headers
#include <ament_index_cpp/get_package_share_directory.hpp>
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

void getCamPosAndTrg(const std::vector<double> &bodyPose, std::vector<double> &camPos, std::vector<double> &camTrg)
{
    // Calculate rotation matrix
    std::vector<double> R_NA = {pow(bodyPose[3], 2) + pow(bodyPose[4], 2) - pow(bodyPose[5], 2) - pow(bodyPose[6], 2), 2 * (bodyPose[4] * bodyPose[5] - bodyPose[3] * bodyPose[6]), 2 * (bodyPose[4] * bodyPose[6] + bodyPose[3] * bodyPose[5]), 2 * (bodyPose[4] * bodyPose[5] + bodyPose[3] * bodyPose[6]), pow(bodyPose[3], 2) - pow(bodyPose[4], 2) + pow(bodyPose[5], 2) - pow(bodyPose[6], 2), 2 * (bodyPose[5] * bodyPose[6] - bodyPose[3] * bodyPose[4]), 2 * (bodyPose[4] * bodyPose[6] - bodyPose[3] * bodyPose[5]), 2 * (bodyPose[5] * bodyPose[6] + bodyPose[3] * bodyPose[4]), pow(bodyPose[3], 2) - pow(bodyPose[4], 2) - pow(bodyPose[5], 2) + pow(bodyPose[6], 2)};

    // Calculate camera position
    // Camera is supposed to be at [-1, 0, 0.2] position in the body attached w.r.t body attached point
    for (size_t i = 0; i < 3; ++i)
        camPos[i] = bodyPose[i] + R_NA[i * 3 + 0] * (-1) + R_NA[i * 3 + 1] * (0) + R_NA[i * 3 + 2] * (0.2);

    // Calculate camera target
    // Camera should be looking at the quadcopter
    for (size_t i = 0; i < 3; ++i)
        camTrg[i] = bodyPose[i];
}

int main(int argc, char **argv)
{
    // Some initialization.
    rclcpp::init(argc, argv);

    // Get location of the package share directory
    std::string pkgShareDir = ament_index_cpp::get_package_share_directory(ROS_PACKAGE_NAME);

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
    std::vector<double> camPos(3), camTrg(3); // Variables to store camera position and target
    getCamPosAndTrg(animWindowPtr->q, camPos, camTrg);
    Camera cam = {
        {camPos[0], camPos[1], camPos[2]}, // Position
        {camTrg[0], camTrg[1], camTrg[2]}, // Looking at
        {0, 0, 1},                         // Direction that is up in camera view
        {45},                              // Field of view
        CAMERA_PERSPECTIVE                 // Projection type
    };

    // Create a Plane and sphere models
    Model plane = LoadModelFromMesh(GenMeshPlane(2, 2, 1, 1)); // Plane is created on XZ plane with normal pointing in +Y direction

    // Load the infinite plane shaders
    Shader infPlaneShader = LoadShader((pkgShareDir + "/shaders/infPlane.vs").c_str(),
                                       (pkgShareDir + "/shaders/infPlane.fs").c_str());

    // Provide shader location for the camera position
    infPlaneShader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(infPlaneShader, "viewPos");

    // Assign infPlane shader to model
    plane.materials[0].shader = infPlaneShader;

    // Apply transformation on the plane to make the normal point to +Z
    plane.transform = MatrixRotate({1, 0, 0}, PI / 2);

    // Define framerate for the window
    SetTargetFPS(120);

    while (!WindowShouldClose() && rclcpp::ok())
    {
        // Update the window size if it has been changed
        screenWidth = GetScreenWidth(), screenHeight = GetScreenHeight();

        // Update camera position and target
        getCamPosAndTrg(animWindowPtr->q, camPos, camTrg);
        cam.position.x = camPos[0];
        cam.position.y = camPos[1];
        cam.position.z = camPos[2];
        cam.target.x = camTrg[0];
        cam.target.y = camTrg[1];
        cam.target.z = camTrg[2];

        // Send camera location to the shader
        float viewPos[3] = {cam.position.x, cam.position.y, cam.position.z};
        SetShaderValue(infPlaneShader, infPlaneShader.locs[SHADER_LOC_VECTOR_VIEW], viewPos, SHADER_UNIFORM_VEC3);

        BeginDrawing();
        ClearBackground(BLACK);

        BeginMode3D(cam);

        DrawModel(plane, Vector3Zero(), 1, WHITE);
        DrawRay(Ray{{0, 0, 0}, {0, 0, 1}}, Color({0, 0, 255, 255}));

        // Draw lines representing the quadcopter
        double timeStamp = animWindowPtr->updateLineData();
        for (size_t i = 0; i < animWindowPtr->baseStart.size(); ++i)
            DrawLine3D(animWindowPtr->baseStart[i], animWindowPtr->baseEnd[i], BLUE);

        for (size_t i = 0; i < animWindowPtr->bodyStart.size(); ++i)
            DrawLine3D(animWindowPtr->bodyStart[i], animWindowPtr->bodyEnd[i], BLUE);

        for (size_t i = 0; i < animWindowPtr->propStart.size(); ++i)
            DrawLine3D(animWindowPtr->propStart[i], animWindowPtr->propEnd[i], RED);

        EndMode3D();

        DrawFPS(10, 10);
        std::string printTime = "Sim Time: " + std::to_string(timeStamp);
        int txtWidth = MeasureText(printTime.c_str(), 24);
        DrawText(printTime.c_str(), (screenWidth - txtWidth) / 2, 10, 24, LIGHTGRAY);

        EndDrawing();

        // Let ROS process subscriber callbacks
        rosExecutor.spin_some();
    }

    rclcpp::shutdown();
    UnloadShader(infPlaneShader);
    CloseWindow();
    return 0;
}
