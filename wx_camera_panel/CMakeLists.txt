cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(wx_camera_panel)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories( ${wxWidgets_INCLUDE_DIRS} )


# shared library
rospack_add_library(wxcamerapanel src/wx_camera_panel/CameraPanelsGenerated.cpp 
				  src/wx_camera_panel/CameraPanel.cpp
				  src/wx_camera_panel/CameraSetupDialog.cpp)
target_link_libraries(wxcamerapanel ${wxWidgets_LIBRARIES})

# test application
rospack_add_executable(camera_test src/test/test.cpp)
target_link_libraries(camera_test wxcamerapanel ${wxWidgets_LIBRARIES})
