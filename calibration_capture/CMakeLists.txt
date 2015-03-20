cmake_minimum_required(VERSION 2.8.3)
project(calibration_capture)

find_package(catkin REQUIRED COMPONENTS message_generation rospy actionlib actionlib_msgs trajectory_msgs calibration_msgs)

catkin_python_setup()

add_action_files(
  DIRECTORY 
    action
  FILES 
    CaptureManager.action
)

add_message_files(
  DIRECTORY 
    msg
  FILES 
    Command.msg
    Measurement.msg
)

generate_messages(DEPENDENCIES actionlib_msgs trajectory_msgs calibration_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib actionlib_msgs trajectory_msgs calibration_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/capture_exec.py
    scripts/capture_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})