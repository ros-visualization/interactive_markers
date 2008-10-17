cmake_minimum_required(VERSION 2.6)
include(rosbuild)
include(FindPkgConfig)

#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Debug)

rospack(ogre_visualizer)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories( ${wxWidgets_INCLUDE_DIRS} )

pkg_check_modules(OGRE OGRE)
include_directories( ${OGRE_INCLUDE_DIRS} )
link_directories( ${OGRE_LIBRARY_DIRS} )

include_directories( src/ogre_visualizer )

# Find the combined swig flags for this project
_rospack_invoke(${PROJECT_NAME} ${PROJECT_NAME} SWIG_FLAGS "--lang=swig" "--attrib=flags" "export")
set(SWIG_FLAGS ${${PROJECT_NAME}_SWIG_FLAGS})

# Find the wxswig executable
find_ros_package(wxswig)
set(WXSWIG_EXECUTABLE ${wxswig_PACKAGE_PATH}/bin/swig)

# Add a custom command for generating the swig output files
set(SWIG_INTERFACE_FILE ${PROJECT_SOURCE_DIR}/src/ogre_visualizer/ogre_visualizer.i)
set(SWIG_OUTPUT_CPP_FILE ${PROJECT_SOURCE_DIR}/src/ogre_visualizer/ogre_visualizer_swig_generated.cpp)
set(SWIG_OUTPUT_PYTHON_FILE ${PROJECT_SOURCE_DIR}/lib/ogre_visualizer.py)
add_custom_command(OUTPUT   ${SWIG_OUTPUT_CPP_FILE} 
                            ${SWIG_OUTPUT_PYTHON_FILE}
                   COMMAND  ${WXSWIG_EXECUTABLE} ${SWIG_FLAGS} -o ${SWIG_OUTPUT_CPP_FILE} -outdir ../lib -module ${PROJECT_NAME} ${SWIG_INTERFACE_FILE}
                   DEPENDS  ${SWIG_INTERFACE_FILE}
                            src/ogre_visualizer/visualizer_base.i
                            src/ogre_visualizer/visualization_panel.i
                            src/ogre_visualizer/visualization_manager.i
                            src/ogre_visualizer/visualizers/axes_visualizer.i
                            src/ogre_visualizer/visualizers/grid_visualizer.i
                            src/ogre_visualizer/visualizers/laser_scan_visualizer.i
                            src/ogre_visualizer/visualizers/marker_visualizer.i
                            src/ogre_visualizer/visualizers/octree_visualizer.i
                            src/ogre_visualizer/visualizers/planning_visualizer.i
                            src/ogre_visualizer/visualizers/point_cloud_visualizer.i
                            src/ogre_visualizer/visualizers/robot_model_visualizer.i
                            src/ogre_visualizer/helpers/color.i
                            src/ogre_visualizer/visualization_panel.h
                            src/ogre_visualizer/visualization_manager.h  
                            src/ogre_visualizer/generated/visualization_panel_generated.h
                            src/ogre_visualizer/visualizer_base.h
                            src/ogre_visualizer/visualizers/axes_visualizer.h
                            src/ogre_visualizer/visualizers/grid_visualizer.h
                            src/ogre_visualizer/visualizers/laser_scan_visualizer.h
                            src/ogre_visualizer/visualizers/marker_visualizer.h
                            src/ogre_visualizer/visualizers/octree_visualizer.h
                            src/ogre_visualizer/visualizers/planning_visualizer.h
                            src/ogre_visualizer/visualizers/point_cloud_visualizer.h
                            src/ogre_visualizer/visualizers/robot_model_visualizer.h
                            src/ogre_visualizer/helpers/color.h)

rospack_add_library(${PROJECT_NAME}  src/ogre_visualizer/properties/property.cpp
                                     src/ogre_visualizer/properties/property_manager.cpp
                                     src/ogre_visualizer/visualization_panel.cpp
                                     src/ogre_visualizer/visualization_manager.cpp
  				   												 src/ogre_visualizer/visualizer_base.cpp
                                     src/ogre_visualizer/common.cpp
                                     src/ogre_visualizer/factory.cpp
                                     src/ogre_visualizer/ros_topic_property.cpp
                                     src/ogre_visualizer/new_display_dialog.cpp
                                     src/ogre_visualizer/visualizers/marker_visualizer.cpp
                                     src/ogre_visualizer/visualizers/axes_visualizer.cpp
  				   												 src/ogre_visualizer/visualizers/grid_visualizer.cpp
  				   												 src/ogre_visualizer/visualizers/point_cloud_visualizer.cpp
  				   												 src/ogre_visualizer/visualizers/laser_scan_visualizer.cpp
  				   												 src/ogre_visualizer/visualizers/robot_model_visualizer.cpp
  				   												 src/ogre_visualizer/visualizers/planning_visualizer.cpp
  				   												 src/ogre_visualizer/visualizers/octree_visualizer.cpp
  				   												 src/ogre_visualizer/visualizers/robot_base2d_pose_visualizer.cpp
   				   												 src/ogre_visualizer/generated/visualization_panel_generated.cpp
   				   												 src/ogre_visualizer/helpers/robot.cpp
 				   												   ${SWIG_OUTPUT_CPP_FILE})
target_link_libraries(${PROJECT_NAME} ${wxWidgets_LIBRARIES} boost_signals ${OGRE_LIBRARIES})
# swig python needs a shared library named _<modulename>.[so|dll|...]
# this renames the output file to conform to that by prepending an underscore and removing the "lib" prefix
set_target_properties(${PROJECT_NAME}
                      PROPERTIES OUTPUT_NAME _${PROJECT_NAME}
                      PREFIX "")

rospack_add_executable(visualizer_test src/test/visualizer_test.cpp)
target_link_libraries(visualizer_test ${PROJECT_NAME} ${wxWidgets_LIBRARIES})

rospack_add_executable(marker_test src/test/marker_test.cpp)
 
