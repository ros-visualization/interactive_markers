cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(log_gui)

set(wxWidgets_USE_LIBS base core)

find_package(wxWidgets REQUIRED)

include(${wxWidgets_USE_FILE})
include_directories(${wxWidgets_INCLUDE_DIRS})

find_program(WXFORMBUILDER NAMES wxformbuilder)

if(WXFORMBUILDER)
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/src/log_gui/GenLogGui.cpp
                            ${PROJECT_SOURCE_DIR}/src/log_gui/GenLogGui.h
                     COMMAND ${WXFORMBUILDER} -g ${PROJECT_SOURCE_DIR}/src/log_gui/log_gui.fbp
                     DEPENDS src/log_gui/log_gui.fbp)
endif(WXFORMBUILDER)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(log_gui src/log_gui/LogGui.cpp
                               src/log_gui/GenLogGui.cpp)

target_link_libraries(log_gui wxtopicdisplay ${wxWidgets_LIBRARIES})

