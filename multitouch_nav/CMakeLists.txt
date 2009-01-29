cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(multitouch_nav)
rospack_add_boost_directories()

rospack_add_executable(skeleton skeleton.cpp)
rospack_link_boost(skeleton thread)
