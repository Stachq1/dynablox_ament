cmake_minimum_required(VERSION 3.5)
project(pointcloud_listener_node)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(dynablox REQUIRED)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/build include)

# Add the voxblox generated .pb.h files to include directories
include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/..)

# Add executable
add_executable(pointcloud_listener_node src/pointcloud_listener_node.cpp)

# Link libraries
ament_target_dependencies(pointcloud_listener_node
    PCL
    pcl_conversions
    rclcpp
    sensor_msgs
    dynablox
)

# Explicitly link yaml-cpp and glog libraries (God himself knows why using ament_target_dependencies does not work here)
target_link_libraries(pointcloud_listener_node
    glog
    ${YAML_CPP_LIBRARIES}
)

# Install targets
install(TARGETS pointcloud_listener_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(PCL pcl_conversions rclcpp sensor_msgs dynablox)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# Ament package
ament_package()
