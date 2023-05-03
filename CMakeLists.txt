cmake_minimum_required(VERSION 3.14)
project(dynablox_benchmark)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(default_build_type "Release")

#############
# PACKAGES  #
#############
add_subdirectory(voxblox)

find_package(PCL REQUIRED)
find_package(Glog REQUIRED)
find_package(gflags REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
set(LOG_LIB glog::glog gflags)
set(FUN_LIB ${PCL_LIBRARIES} Eigen3::Eigen yaml-cpp)

#############
# LIBRARIES #
#############
include_directories(${CMAKE_SOURCE_DIR}/build)
include_directories(${CMAKE_SOURCE_DIR}/voxblox/include)
include_directories(include)

add_library(dufo_lib SHARED 
            src/dynablox.cpp 
            src/clustering.cpp 
            src/TsdfMapper.cpp)

target_link_libraries(dufo_lib ${LOG_LIB} yaml-cpp ${FUN_LIB} voxblox)

add_executable(dynablox_run dynablox_pcd.cpp)
target_link_libraries(dynablox_run dufo_lib)