cmake_minimum_required(VERSION 2.8.3)
project(extrinsic_calibrator)

find_package(catkin REQUIRED COMPONENTS roscpp interactive_markers visualization_msgs dynamic_transform_publisher)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  CATKIN_DEPENDS roscpp interactive_markers visualization_msgs dynamic_transform_publisher
  DEPENDS Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(interactive_calibrator src/InteractiveCalibratorControl.cpp)

target_link_libraries(interactive_calibrator
  ${Boost_LIBRARIES} 
  ${catkin_LIBRARIES}
)
add_dependencies(interactive_calibrator ${catkin_EXPORTED_TARGETS})

install(TARGETS interactive_calibrator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)