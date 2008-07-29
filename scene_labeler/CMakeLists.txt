cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(scene_labeler)
add_definitions(-Wall)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
rospack_add_executable(scene_labeler src/scene_labeler/scene_labeler.cpp)
set_target_properties(scene_labeler PROPERTIES LINK_FLAGS "-g -pg" COMPILE_FLAGS "-g -pg")

