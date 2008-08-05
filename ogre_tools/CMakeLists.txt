cmake_minimum_required(VERSION 2.6)
include(rosbuild)
include(FindPkgConfig)

rospack(ogre_tools)

#set(ROS_BUILD_TYPE Debug)

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories( ${wxWidgets_INCLUDE_DIRS} )

# We need GTK2 and OpenGL for the wx render window
pkg_check_modules(GTK2 gtk+-2.0)

include_directories( ${GTK2_INCLUDE_DIRS} )
add_definitions( ${GTK2_DEFINITIONS} )
link_directories( ${GTK2_LIBRARY_DIRS} )

find_package(OpenGL REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIR} )

rospack_add_library(ogretools src/ogre_tools/point_cloud.cpp
            src/ogre_tools/cone.cpp
			      src/ogre_tools/grid.cpp
			      src/ogre_tools/super_ellipsoid.cpp
			      src/ogre_tools/axes.cpp
			      src/ogre_tools/camera_base.cpp
			      src/ogre_tools/fps_camera.cpp
			      src/ogre_tools/orbit_camera.cpp
			      src/ogre_tools/wx_ogre_render_window.cpp)
target_link_libraries(ogretools ${wxWidgets_LIBRARIES} ${GTK2_LIBRARIES} ${OPENGL_gl_LIBRARY})


rospack_add_executable(bin/point_cloud_test src/test/point_cloud_test.cpp)
target_link_libraries(bin/point_cloud_test ogretools ${wxWidgets_LIBRARIES})

rospack_add_executable(bin/grid_test src/test/grid_test.cpp)
target_link_libraries(bin/grid_test ogretools ${wxWidgets_LIBRARIES})
