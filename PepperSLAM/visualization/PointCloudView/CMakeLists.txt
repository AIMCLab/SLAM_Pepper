cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PointCloudView)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories( ${EIGEN_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

aux_source_directory(. SRCS)

add_executable(show_pointcloud show_pointcloud.cpp)
add_executable(merge_pointclouds merge_pointclouds.cpp)
add_executable(beautify_pointcloud beautify_pointcloud.cpp)

target_link_libraries(show_pointcloud ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(merge_pointclouds ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(beautify_pointcloud ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})