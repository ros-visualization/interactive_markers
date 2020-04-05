cmake_minimum_required(VERSION 3.0.2)
project(interactive_markers)
find_package(catkin REQUIRED 
  rosconsole
  roscpp
  rospy
  rostest
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES interactive_markers
  CATKIN_DEPENDS roscpp rosconsole rospy tf2_ros std_msgs visualization_msgs
)
catkin_python_setup()

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
src/interactive_marker_server.cpp
src/tools.cpp
src/menu_handler.cpp
src/interactive_marker_client.cpp
src/single_client.cpp
src/message_context.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "INTERACTIVE_MARKERS_BUILDING_LIBRARY")

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/interactive_markers/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp")

# C++ Unit Tests
#
if(CATKIN_ENABLE_TESTING)
  include_directories(${GTEST_INCLUDE_DIRS})

  add_executable(server_test EXCLUDE_FROM_ALL src/test/server_test.cpp)
  target_link_libraries(server_test ${PROJECT_NAME} ${GTEST_LIBRARIES})
  add_dependencies(tests server_test)
  add_rostest(test/cpp_server.test)

  add_executable(client_test EXCLUDE_FROM_ALL src/test/client_test.cpp)
  target_link_libraries(client_test ${PROJECT_NAME} ${GTEST_LIBRARIES})
  add_dependencies(tests client_test)
  add_rostest(test/cpp_client.test)

  add_executable(server_client_test EXCLUDE_FROM_ALL src/test/server_client_test.cpp)
  target_link_libraries(server_client_test ${PROJECT_NAME} ${GTEST_LIBRARIES})
  add_dependencies(tests server_client_test)
  add_rostest(test/cpp_server_client.test)

  # Test program to simulate Interactive Marker with missing tf information
  add_executable(bursty_tf EXCLUDE_FROM_ALL src/test/bursty_tf.cpp)
  target_link_libraries(bursty_tf ${PROJECT_NAME})
  add_dependencies(tests bursty_tf)

  # Test program to simulate Interactive Marker with wrong tf information
  add_executable(missing_tf EXCLUDE_FROM_ALL src/test/missing_tf.cpp)
  target_link_libraries(missing_tf ${PROJECT_NAME})
  add_dependencies(tests missing_tf)
endif()
