cmake_minimum_required(VERSION 2.8.3)
project(interactive_markers)
find_package(catkin REQUIRED 
  message_filters
  rosbag
  rosconsole
  roscpp
  rospy
  std_msgs
  std_srvs
  tf
  visualization_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES interactive_markers
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
src/interactive_marker_server.cpp
src/tools.cpp
src/menu_handler.cpp
src/interactive_marker_client.cpp
src/single_client.cpp
src/message_context.cpp
)

install(TARGETS interactive_markers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/interactive_markers/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

# C++ Unit Tests
#
message("TODO: convert interactive_markers tests to catkin")
##
##  add_executable(server_test EXCLUDE_FROM_ALL src/test/server_test.cpp)
##  target_link_libraries(server_test ${PROJECT_NAME})
##  rosbuild_add_gtest_build_flags(server_test)
##  rosbuild_add_rostest(test/cpp_server.test)
##  
##  rosbuild_add_executable(client_test EXCLUDE_FROM_ALL src/test/client_test.cpp)
##  target_link_libraries(client_test ${PROJECT_NAME})
##  rosbuild_add_gtest_build_flags(client_test)
##  rosbuild_add_rostest(test/cpp_client.test)
##  
##  rosbuild_add_executable(server_client_test EXCLUDE_FROM_ALL src/test/server_client_test.cpp)
##  target_link_libraries(server_client_test ${PROJECT_NAME})
##  rosbuild_add_gtest_build_flags(server_client_test)
##  rosbuild_add_rostest(test/cpp_server_client.test)

# Test program to simulate Interactive Marker with missing tf information
#rosbuild_add_executable(bursty_tf src/test/bursty_tf.cpp)
#target_link_libraries(bursty_tf ${PROJECT_NAME})

# Test program to simulate Interactive Marker with wrong tf information
#rosbuild_add_executable(missing_tf src/test/missing_tf.cpp)
#target_link_libraries(missing_tf ${PROJECT_NAME})
