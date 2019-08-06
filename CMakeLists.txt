cmake_minimum_required(VERSION 3.5)
project(interactive_markers)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(${PROJECT_NAME}
  src/interactive_marker_server.cpp
  src/tools.cpp
  src/menu_handler.cpp
  src/interactive_marker_client.cpp
  src/message_context.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rmw"
  "tf2"
  "tf2_geometry_msgs"
  "visualization_msgs")

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  DIRECTORY include/
  DESTINATION include)

ament_export_dependencies("rclcpp")
ament_export_dependencies("rmw")
ament_export_dependencies("tf2")
ament_export_dependencies("tf2_geometry_msgs")
ament_export_dependencies("visualization_msgs")
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)

  set(ament_cmake_flake8_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_interactive_marker_server
    test/${PROJECT_NAME}/test_interactive_marker_server.cpp
  )
  ament_target_dependencies(test_interactive_marker_server
    "geometry_msgs"
    "rclcpp"
    "std_msgs"
  )
  target_link_libraries(test_interactive_marker_server
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_interactive_marker_client
    test/${PROJECT_NAME}/test_interactive_marker_client.cpp
  )
  ament_target_dependencies(test_interactive_marker_client
    "geometry_msgs"
    "rclcpp"
    "std_msgs"
    "tf2"
  )
  target_link_libraries(test_interactive_marker_client
    ${PROJECT_NAME}
  )
endif()

ament_package()
