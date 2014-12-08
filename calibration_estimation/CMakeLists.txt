cmake_minimum_required(VERSION 2.8.3)
project(calibration_estimation)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package(
)

catkin_python_setup()

# Unit tests need to be refactored to use nose.
catkin_add_nosetests(test
  DEPENDENCIES
  	sensor_msgs
  	calibration_msgs
  	visualization_msgs
)

install(DIRECTORY docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY test/data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test)