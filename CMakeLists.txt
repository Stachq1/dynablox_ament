cmake_minimum_required(VERSION 3.14)
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
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_subdirectory(voxblox)
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/build/voxblox")

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(Glog REQUIRED)
find_package(gflags REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
set(LOG_LIB glog::glog gflags)
set(FUN_LIB ${PCL_LIBRARIES} voxblox::voxblox Eigen3::Eigen yaml-cpp)

#############
# LIBRARIES #
#############
include_directories(${CMAKE_SOURCE_DIR}/build)
include_directories(include)

add_library(dm_lib SHARED 
            src/dynablox.cpp 
            src/clustering.cpp 
            src/TsdfMapper.cpp)
add_dependencies(dm_lib voxblox)
target_link_libraries(dm_lib ${LOG_LIB} ${FUN_LIB})

#############
# INSTALL   #
#############
install(TARGETS dm_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(PCL Glog gflags Eigen3 yaml-cpp)
ament_export_include_directories(include)
ament_export_libraries(dm_lib)

ament_package()
