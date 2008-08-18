cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(wx_roserr)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories( ${wxWidgets_INCLUDE_DIRS} )


# shared library
rospack_add_library(wxroserr src/wx_roserr/roserrGenerated.cpp 
				  src/wx_roserr/RoserrPanel.cpp
				  src/wx_roserr/RoserrSetupDialog.cpp)
target_link_libraries(wxroserr ${wxWidgets_LIBRARIES})

# test application
rospack_add_executable(roserr_test src/test/test.cpp)
rospack_add_executable(roserr_send_test src/test/send_test.cpp)
target_link_libraries(roserr_test wxroserr ${wxWidgets_LIBRARIES})
target_link_libraries(roserr_send_test ${wxWidgets_LIBRARIES})