cmake_minimum_required(VERSION 2.8.3)
project(camera_intrinsic_calibrator)

find_package(catkin REQUIRED COMPONENTS rospy message_generation actionlib actionlib_msgs sensor_msgs)

catkin_python_setup()

add_action_files(
  DIRECTORY 
    action
  FILES 
    MonoCalibrator.action
    StereoCalibrator.action
)

add_message_files(
  DIRECTORY 
    msg
  FILES 
    CalibrationCoverage.msg
    Progress.msg
)

generate_messages(DEPENDENCIES actionlib_msgs sensor_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime  actionlib actionlib_msgs sensor_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/mono_calibrator.py
    scripts/stereo_calibrator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})