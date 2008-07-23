cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(wx_camera_panel)

set(ROS_BUILD_TYPE Debug)

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories( ${wxWidgets_INCLUDE_DIRS} )

set(CommonFiles src/wx_camera_panel/CameraPanelsGenerated.cpp 
		src/wx_camera_panel/CameraPanel.cpp
		src/wx_camera_panel/CameraSetupDialog.cpp)


# shared library
rospack_add_library(wxcamerapanel ${CommonFiles})
target_link_libraries(wxcamerapanel ${wxWidgets_LIBRARIES})

# test application
rospack_add_executable(bin/test src/test/test.cpp ${CommonFiles})
target_link_libraries(bin/test ${wxWidgets_LIBRARIES})
