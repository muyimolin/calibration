cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#generate actions
rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

#generate dynamic reconfigure
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#generate messages
rosbuild_genmsg()

find_package(OpenCV REQUIRED)

rosbuild_add_boost_directories()

rosbuild_add_executable(image_annotator src/image_annotator.cpp)
target_link_libraries(image_annotator ${OpenCV_LIBRARIES})
rosbuild_link_boost(image_annotator signals)

rosbuild_add_library(calibration_targets include/image_cb_detector/image_converter.h
                                         #src/image_converter.cpp
                                         include/image_cb_detector/chessboard_target.h
                                         src/chessboard_target.cpp
                                         include/image_cb_detector/circles_target.h
                                         src/circles_target.cpp)
target_link_libraries(calibration_targets ${OpenCV_LIBRARIES})

rosbuild_add_executable(calibration_detector include/image_cb_detector/calibration_detector_server.h
                                             src/calibration_detector_server.cpp)
rosbuild_link_boost(calibration_detector thread)
