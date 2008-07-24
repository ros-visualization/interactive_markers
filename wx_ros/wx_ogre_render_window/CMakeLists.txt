cmake_minimum_required(VERSION 2.6)
include(rosbuild)
include(FindPkgConfig)

rospack(wx_ogre_render_window)

#set(ROS_BUILD_TYPE Debug)

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories( ${wxWidgets_INCLUDE_DIRS} )

pkg_check_modules(GTK2 gtk+-2.0)

include_directories( ${GTK2_INCLUDE_DIRS} )
add_definitions( ${GTK2_DEFINITIONS} )
link_directories( ${GTK2_LIBRARY_DIRS} )

find_package(OpenGL REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIR} )

# shared library
rospack_add_library(wxogrerenderwindow src/wx_ogre_render_window/wx_ogre_render_window.cpp)
target_link_libraries(wxogrerenderwindow ${wxWidgets_LIBRARIES} ${GTK2_LIBRARIES} ${OPENGL_gl_LIBRARY})

# test application
rospack_add_executable(bin/test src/test/test.cpp)
target_link_libraries(bin/test wxogrerenderwindow ${wxWidgets_LIBRARIES})
