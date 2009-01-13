cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(wx_topic_display)

set(wxWidgets_USE_LIBS base core)

find_package(wxWidgets REQUIRED)

include(${wxWidgets_USE_FILE})
include_directories(${wxWidgets_INCLUDE_DIRS})

find_program(WXFORMBUILDER NAMES wxformbuilder)

#if(WXFORMBUILDER)
#  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/src/wx_topic_display/GenTopicDisplay.cpp
#                            ${PROJECT_SOURCE_DIR}/src/wx_topic_display/GenTopicDisplay.h
#                     COMMAND ${WXFORMBUILDER} -g ${PROJECT_SOURCE_DIR}/src/wx_topic_display/topic_display.fbp
#                     DEPENDS src/wx_topic_display/topic_display.fbp)
#
#  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/src/test/GenTestTopicDisplay.cpp
#                            ${PROJECT_SOURCE_DIR}/src/test/GenTestTopicDisplay.h
#                     COMMAND ${WXFORMBUILDER} -g ${PROJECT_SOURCE_DIR}/src/test/test_topic_display.fbp
#                     DEPENDS src/test/test_topic_display.fbp)
#endif(WXFORMBUILDER)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(${PROJECT_SOURCE_DIR}/src)

rospack_add_library(wxtopicdisplay src/wx_topic_display/TopicDisplay.cpp
                                   src/wx_topic_display/GenTopicDisplay.cpp
				   src/wx_topic_display/TopicDisplayDialog.cpp)
target_link_libraries(wxtopicdisplay ${wxWidgets_LIBRARIES})

rospack_add_executable(simpletest src/test/test.cpp
                            src/test/GenTestTopicDisplay.cpp)

target_link_libraries(simpletest wxtopicdisplay ${wxWidgets_LIBRARIES})

