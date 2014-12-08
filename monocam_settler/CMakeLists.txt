cmake_minimum_required(VERSION 2.8.3)
project(monocam_settler)

find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs settlerlib calibration_msgs)

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
  CATKIN_DEPENDS message_runtime roscpp actionlib actionlib_msgs settlerlib calibration_msgs
  DEPENDS Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
  src/monocam_settler.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(monocam_settler_action src/monocam_settler_action.cpp)
target_link_libraries(monocam_settler_action ${PROJECT_NAME})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
catkin_add_gtest(monocam_settler_unittest test/monocam_settler_unittest.cpp)
target_link_libraries(monocam_settler_unittest ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} monocam_settler_action
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)