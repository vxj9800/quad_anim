cmake_minimum_required(VERSION 3.8)

# Create project and make it's name available to the executables.
# This will allow the nodes to figure out the name of the package they are in.
project(quad_anim C CXX)
add_compile_definitions(ROS_PACKAGE_NAME="${CMAKE_PROJECT_NAME}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Python REQUIRED)
find_package(raylib REQUIRED)

# Somehow the next line makes the raylib work.
# However, it is not required by other libraries.
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Generate some C/C++ code
execute_process(
  COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/genGeomVects.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# Add target
add_executable(anim_window
  src/xCoords.c
  src/yCoords.c
  src/zCoords.c
  src/animWindowNode.cpp
  src/anim_window.cpp
)
ament_target_dependencies(anim_window rclcpp std_msgs geometry_msgs builtin_interfaces)
target_link_libraries(anim_window raylib)
target_include_directories(anim_window PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(anim_window PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Move binaries to lib folder
install(TARGETS anim_window
  DESTINATION lib/${PROJECT_NAME}
)

# Move launch files to share folder
install(DIRECTORY
  launch
  shaders
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
