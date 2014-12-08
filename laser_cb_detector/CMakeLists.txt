cmake_minimum_required(VERSION 2.8.3)
project(laser_cb_detector)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp actionlib actionlib_msgs std_msgs calibration_msgs message_filters cv_bridge)

find_package(Boost REQUIRED COMPONENTS signals thread)
find_package(OpenCV REQUIRED)

add_action_files(
  DIRECTORY 
    action
  FILES 
    Config.action
)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs calibration_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime roscpp actionlib actionlib_msgs std_msgs calibration_msgs message_filters cv_bridge
  DEPENDS Boost OpenCV
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/cv_laser_bridge.cpp
    src/laser_cb_detector.cpp
    src/laser_interval_calc.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES} 
  ${OpenCV_LIBRARIES} 
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(laser_cb_detector_node src/laser_cb_detector_node.cpp)
target_link_libraries(laser_cb_detector_node ${PROJECT_NAME})

add_executable(laser_interval_calc_node src/laser_interval_calc_node.cpp)
target_link_libraries(laser_interval_calc_node ${PROJECT_NAME})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
catkin_add_gtest(cv_laser_bridge_unittest     test/cv_laser_bridge_unittest.cpp)
catkin_add_gtest(laser_cb_detector_unittest   test/laser_cb_detector_unittest.cpp)
catkin_add_gtest(laser_interval_calc_unittest test/laser_interval_calc_unittest.cpp)
target_link_libraries(cv_laser_bridge_unittest     ${PROJECT_NAME})
target_link_libraries(laser_cb_detector_unittest   ${PROJECT_NAME})
target_link_libraries(laser_interval_calc_unittest ${PROJECT_NAME})

install(TARGETS laser_cb_detector_node laser_interval_calc_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY cfg test/data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)