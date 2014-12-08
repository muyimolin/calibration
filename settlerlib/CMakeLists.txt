cmake_minimum_required(VERSION 2.8.3)
project(settlerlib)

find_package(catkin REQUIRED COMPONENTS roscpp calibration_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp calibration_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
  src/deflated.cpp
  src/interval_calc.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
catkin_add_gtest(sorted_deque_unittest  test/sorted_deque_unittest.cpp)
catkin_add_gtest(interval_calc_unittest test/interval_calc_unittest.cpp)
catkin_add_gtest(deflated_unittest      test/deflated_unittest.cpp)
target_link_libraries(sorted_deque_unittest  ${PROJECT_NAME})
target_link_libraries(interval_calc_unittest ${PROJECT_NAME})
target_link_libraries(deflated_unittest      ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)