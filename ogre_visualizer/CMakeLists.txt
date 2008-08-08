cmake_minimum_required(VERSION 2.6)
include(rosbuild)

#set(ROS_BUILD_TYPE Debug)

rospack(ogre_visualizer)

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories( ${wxWidgets_INCLUDE_DIRS} )

rospack_add_library(ogrevisualizer src/ogre_visualizer/visualization_panel.cpp
				   												 src/ogre_visualizer/visualizer_base.cpp
                                   src/ogre_visualizer/common.cpp
                                   src/ogre_visualizer/visualizers/marker_visualizer.cpp
                                   src/ogre_visualizer/visualizers/axes_visualizer.cpp
				   												 src/ogre_visualizer/visualizers/grid_visualizer.cpp
				   												 src/ogre_visualizer/visualizers/point_cloud_visualizer.cpp
				   												 src/ogre_visualizer/visualizers/laser_scan_visualizer.cpp
				   												 src/ogre_visualizer/visualizers/robot_model_visualizer.cpp
				   												 src/ogre_visualizer/visualizers/grid_options_panel.cpp
				   												 src/ogre_visualizer/visualizers/generated/grid_options_panel.cpp
 				   												 src/ogre_visualizer/generated/visualization_panel_generated.cpp)
target_link_libraries(ogrevisualizer ${wxWidgets_LIBRARIES})

rospack_add_executable(bin/visualizer_test src/test/visualizer_test.cpp)
target_link_libraries(bin/visualizer_test ogrevisualizer ${wxWidgets_LIBRARIES})

rospack_add_executable(bin/marker_test src/test/marker_test.cpp)
 
