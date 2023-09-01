cmake_minimum_required(VERSION 3.8)
project(conveyor_belt)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)

# Install All Directories
install(DIRECTORY
    urdf
    meshes
    meshes/box
    launch
    worlds
    src
    scripts
    DESTINATION share/${PROJECT_NAME}/
    )



# Launch File Executable
install(
    PROGRAMS
    launch/robot_description_publisher.py
    scripts/spawn_object.py
    DESTINATION lib/${PROJECT_NAME}
    )


# Plugin

add_library(conveyor_belt_plugin SHARED
  src/conveyor_belt_plugin.cpp
)
target_include_directories(conveyor_belt_plugin PUBLIC include)
ament_target_dependencies(conveyor_belt_plugin
  "gazebo_ros"
)
ament_export_libraries(conveyor_belt_plugin)


install(TARGETS
    conveyor_belt_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()

install(DIRECTORY include/
        DESTINATION include)

