cmake_minimum_required(VERSION 3.5)
project(dynablox)

# see here: https://github.com/PointCloudLibrary/pcl/issues/3680#issuecomment-700642602
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()
# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_definitions(GLOG_USE_GLOG_EXPORT)

#############
# PACKAGES  #
#############
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(Glog REQUIRED)
find_package(gflags REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(voxblox REQUIRED)

#############
# LIBRARIES #
#############
include_directories(${CMAKE_SOURCE_DIR}/build include)

add_library(${PROJECT_NAME} SHARED
            src/dynablox.cpp
            src/clustering.cpp
            src/TsdfMapper.cpp)

# Add the voxblox generated .pb.h files to include directories
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/..)

ament_target_dependencies(${PROJECT_NAME} PCL Glog gflags Eigen3 yaml-cpp voxblox)

#############
# INSTALL   #
#############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/include/
  DESTINATION include
)

ament_export_dependencies(PCL Glog gflags Eigen3 yaml-cpp voxblox)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
