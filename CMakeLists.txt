cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(${PROJECT_NAME} 
src/interactive_marker_server.cpp
src/tools.cpp
src/menu_handler.cpp
src/interactive_marker_client.cpp
src/single_client.cpp
src/message_context.cpp
)

# C++ Unit Tests
rosbuild_add_executable(server_test EXCLUDE_FROM_ALL src/test/server_test.cpp)
target_link_libraries(server_test ${PROJECT_NAME})
rosbuild_add_gtest_build_flags(server_test)
rosbuild_add_rostest(test/cpp_server.test)

# Test program to simulate Interactive Marker with missing tf information
#rosbuild_add_executable(bursty_tf src/test/bursty_tf.cpp)
#target_link_libraries(bursty_tf ${PROJECT_NAME})

# Test program to simulate Interactive Marker with wrong tf information
#rosbuild_add_executable(missing_tf src/test/missing_tf.cpp)
#target_link_libraries(missing_tf ${PROJECT_NAME})
