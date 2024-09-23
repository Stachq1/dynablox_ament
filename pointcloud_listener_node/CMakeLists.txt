cmake_minimum_required(VERSION 3.5)
project(pointcloud_listener_node)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynablox REQUIRED)
find_package(PCL REQUIRED)

# Include directories
include_directories(
    include
    ${dynablox_INCLUDE_DIRS}
)

# Add executable
add_executable(pointcloud_listener_node src/pointcloud_listener_node.cpp)

# Link libraries
ament_target_dependencies(pointcloud_listener_node
    PCL
    rclcpp
    sensor_msgs
    dynablox
)

# Install targets
install(TARGETS pointcloud_listener_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(PCL rclcpp sensor_msgs dynablox)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# Ament package
ament_package()
