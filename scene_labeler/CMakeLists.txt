cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(scene_labeler)
add_definitions(-Wall)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#rospack_add_executable(scene_labeler src/scene_labeler/scene_labeler.cpp)
rospack_add_executable(scene_labeler_stereo src/scene_labeler/scene_labeler_stereo.cpp)
rospack_add_executable(smallv_transformer src/smallv_transformer/smallv_transformer.cpp)

# The multi version should be general enough to handle this case.
#rospack_add_executable(image_bg_subtr 
#  src/image_bg_subtr/CaptureObjects.cpp 
#  src/image_bg_subtr/cvx_multiwin.cpp 
#  src/image_bg_subtr/cv_yuv_codebook.cpp)

rospack_add_executable(image_bg_subtr_multi 
  src/image_bg_subtr/CaptureObjectsMulti.cpp 
  src/image_bg_subtr/cvx_multiwin.cpp 
  src/image_bg_subtr/cv_yuv_codebook.cpp)

rospack_add_library(scene_labeler src/scene_labeler/scene_labeler_stereo.cpp)
