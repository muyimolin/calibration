cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_library(image_cb_detector_old src/image_cb_detector_old.cpp)

rosbuild_add_executable(image_cb_detector_node_old src/image_cb_detector_node_old.cpp)
target_link_libraries(image_cb_detector_node_old image_cb_detector_old)

rosbuild_add_executable(image_cb_detector_action_old src/image_cb_detector_action_old.cpp)
target_link_libraries(image_cb_detector_action_old image_cb_detector_old)

rosbuild_add_executable(image_annotator src/image_annotator.cpp)
target_link_libraries(image_annotator image_cb_detector_old)