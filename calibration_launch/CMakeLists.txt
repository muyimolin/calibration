cmake_minimum_required(VERSION 2.8.3)
project(calibration_launch)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package(
)

# Unit tests need to be refactored to use nose.
catkin_add_nosetests(test
  DEPENDENCIES
    sensor_msgs
    calibration_msgs
    geometry_msgs
    trajectory_msgs
    std_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/urdf_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch share
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})