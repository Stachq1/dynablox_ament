cmake_minimum_required(VERSION 3.0.2)
project(dynablox)

find_package(PCL REQUIRED)
find_package(Glog REQUIRED)
find_package(gflags REQUIRED)
set(LOG_LIB glog::glog gflags)
find_package(PCL REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/build)
include_directories(${CMAKE_SOURCE_DIR}/voxblox/include)

add_compile_options(-std=c++17 -Wall -Wextra)

add_library(${PROJECT_NAME}
        src/processing/preprocessing.cpp
        src/processing/clustering.cpp
        src/processing/tracking.cpp
        src/processing/ever_free_integrator.cpp
        src/evaluation/evaluator.cpp
        src/evaluation/ground_truth_handler.cpp
        src/evaluation/io_tools.cpp
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
        ${PCL_LIBRARIES} ${LOG_LIB}
)

# install(TARGETS ${PROJECT_NAME}
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# install(FILES package.xml
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )