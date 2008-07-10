cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(wx_topic_display)

set(wxWidgets_USE_LIBS base core)

find_package(wxWidgets REQUIRED)

include(${wxWidgets_USE_FILE})
include_directories(${wxWidgets_INCLUDE_DIRS})

set(EXECUTABLE_OUTPUT_PATH ${wx_topic_display_SOURCE_DIR}/bin)

rospack_add_library(wxtopicdisplay src/wx_topic_display/TopicDisplay.cpp
                                   src/wx_topic_display/GenTopicDisplay.cpp)

rospack_add_executable(test src/test/test.cpp
                            src/test/GenTestTopicDisplay.cpp)
target_link_libraries(test wxtopicdisplay ${wxWidgets_LIBRARIES})

