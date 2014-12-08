cmake_minimum_required(VERSION 2.8.3)
project(image_cb_detector)

find_package(catkin REQUIRED COMPONENTS message_generation cmake_modules rospy roscpp actionlib actionlib_msgs settlerlib calibration_msgs sensor_msgs message_filters dynamic_reconfigure cv_bridge image_geometry pluginlib)

find_package(Boost REQUIRED COMPONENTS signals thread)
find_package(TinyXML REQUIRED)
find_package(OpenCV REQUIRED)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/Chessboard.cfg
  cfg/Circles.cfg
)

add_action_files(
  DIRECTORY 
    action
  FILES 
    Config.action
    Select.action
)

generate_messages(DEPENDENCIES actionlib_msgs calibration_msgs sensor_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES calibration_targets
  CATKIN_DEPENDS message_runtime roscpp actionlib actionlib_msgs settlerlib calibration_msgs sensor_msgs message_filters dynamic_reconfigure cv_bridge image_geometry pluginlib
  DEPENDS Boost TinyXML OpenCV
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(calibration_targets 
  src/chessboard_target.cpp
  src/circles_target.cpp
)

target_link_libraries(calibration_targets
  ${Boost_LIBRARIES} 
  ${TinyXML_LIBRARIES}
  ${OpenCV_LIBRARIES} 
  ${catkin_LIBRARIES}
)
add_dependencies(calibration_targets ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(calibration_targets ${catkin_EXPORTED_TARGETS})

add_executable(image_annotator src/image_annotator.cpp)
target_link_libraries(image_annotator ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(image_annotator ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(image_annotator ${catkin_EXPORTED_TARGETS})

add_executable(calibration_detector src/calibration_detector_server.cpp)
target_link_libraries(calibration_detector ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(calibration_detector ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(calibration_detector ${catkin_EXPORTED_TARGETS})

install(TARGETS image_annotator calibration_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES calibration_targets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY cfg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
)

catkin_install_python(
  PROGRAMS
    scripts/calibration_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)