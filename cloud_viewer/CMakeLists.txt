cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(cloud_viewer)

rospack_add_boost_directories()

rospack_add_library(cloud_viewer src/libcloud_viewer/cloud_viewer.cpp)
rospack_add_executable(src/standalone/cloud src/standalone/cloud.cpp)
target_link_libraries(src/standalone/cloud cloud_viewer)
rospack_add_executable(bin/cloud_node src/cloud_node/cloud_node.cpp)
target_link_libraries(bin/cloud_node cloud_viewer)
rospack_link_boost(bin/cloud_node thread)
