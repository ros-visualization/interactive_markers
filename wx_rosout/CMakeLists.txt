cmake_minimum_required(VERSION 2.6)
include(rosbuild)

set(ROS_BUILD_TYPE Debug)

rospack(wx_rosout)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories(${wxWidgets_INCLUDE_DIRS})

# Find the combined swig flags for this project
_rospack_invoke(${PROJECT_NAME} ${PROJECT_NAME} SWIG_FLAGS "--lang=swig" "--attrib=flags" "export")
set(SWIG_FLAGS ${${PROJECT_NAME}_SWIG_FLAGS})

# Find the wxswig executable
find_ros_package(wxswig)
set(WXSWIG_EXECUTABLE ${wxswig_PACKAGE_PATH}/bin/swig)

# Add a custom command for generating the swig output files
set(SWIG_INTERFACE_FILE ${PROJECT_SOURCE_DIR}/src/wx_rosout/rosout_panel.i)
set(SWIG_OUTPUT_CPP_FILE ${PROJECT_SOURCE_DIR}/src/wx_rosout/rosout_panel_swig_generated.cpp)
set(SWIG_OUTPUT_PYTHON_FILE ${PROJECT_SOURCE_DIR}/lib/wx_rosout.py)
add_custom_command(OUTPUT   ${SWIG_OUTPUT_CPP_FILE} 
                            ${SWIG_OUTPUT_PYTHON_FILE}
                   COMMAND  ${WXSWIG_EXECUTABLE} ${SWIG_FLAGS} -o ${SWIG_OUTPUT_CPP_FILE} -outdir ../lib -module ${PROJECT_NAME} ${SWIG_INTERFACE_FILE}
                   DEPENDS  ${SWIG_INTERFACE_FILE}
                            src/wx_rosout/rosout_generated.h
                            src/wx_rosout/rosout_panel.h)

# shared library
rospack_add_library(${PROJECT_NAME} src/wx_rosout/rosout_generated.cpp 
				  src/wx_rosout/rosout_panel.cpp
				  src/wx_rosout/rosout_list_control.cpp
				  src/wx_rosout/rosout_setup_dialog.cpp
				  ${SWIG_OUTPUT_CPP_FILE})
target_link_libraries(${PROJECT_NAME} ${wxWidgets_LIBRARIES} -lboost_regex)
# swig python needs a shared library named _<modulename>.[so|dll|...]
# this renames the output file to conform to that by prepending an underscore and removing the "lib" prefix
set_target_properties(${PROJECT_NAME}
                      PROPERTIES OUTPUT_NAME _${PROJECT_NAME}
                      PREFIX "")

# test application
rospack_add_executable(rosout_test src/test/test.cpp)
rospack_add_executable(rosout_send_test src/test/send_test.cpp)
target_link_libraries(rosout_test ${PROJECT_NAME} ${wxWidgets_LIBRARIES})
target_link_libraries(rosout_send_test ${wxWidgets_LIBRARIES})
