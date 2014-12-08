cmake_minimum_required(VERSION 2.8.3)
project(interval_intersection)

find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs calibration_msgs)

find_package(Boost REQUIRED COMPONENTS thread)

add_action_files(
  DIRECTORY 
    action
  FILES 
    Config.action
)

generate_messages(DEPENDENCIES actionlib_msgs calibration_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime roscpp actionlib actionlib_msgs calibration_msgs
  DEPENDS Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
  src/interval_intersection.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(interval_intersection_node src/interval_intersection_node.cpp)
target_link_libraries(interval_intersection_node ${PROJECT_NAME})

add_executable(interval_intersection_action src/interval_intersection_action.cpp)
target_link_libraries(interval_intersection_action ${PROJECT_NAME})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
catkin_add_gtest(interval_intersection_unittest test/interval_intersection_unittest.cpp)
target_link_libraries(interval_intersection_unittest ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} interval_intersection_node interval_intersection_action
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)