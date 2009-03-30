cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(multitouch_nav)
rospack_add_executable(touchServer touchServer.cpp)
