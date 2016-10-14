cmake_minimum_required(VERSION 2.8.3)
project(interactive_markers)
find_package(catkin REQUIRED 
  dynamic_reconfigure
  rosconsole
  roscpp
  rospy
  rostest
  message_generation
  std_msgs
  tf
  visualization_msgs
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/TransformParameter.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES interactive_markers
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs message_runtime roscpp rosconsole rospy tf visualization_msgs
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

add_executable(dynamic_transform_publisher src/dynamic_transform_publisher.cpp)
add_dependencies(dynamic_transform_publisher ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)
target_link_libraries(dynamic_transform_publisher ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/interactive_markers/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

# C++ Unit Tests
#
if(GTEST_FOUND)
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
