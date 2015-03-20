cmake_minimum_required(VERSION 2.8.3)
project(dynamic_transform_publisher)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp geometry_msgs tf)

include(FindPkgConfig)
pkg_check_modules(Yaml REQUIRED yaml-cpp)

add_service_files(
  DIRECTORY 
    srv
  FILES 
    Save.srv
    Update.srv
)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp geometry_msgs tf
  DEPENDS Yaml
)

include_directories(
  ${Yaml_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(dynamic_transform_publisher src/publisher_node.cpp)

target_link_libraries(dynamic_transform_publisher
  ${Yaml_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(dynamic_transform_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(dynamic_transform_publisher ${catkin_EXPORTED_TARGETS})

install(TARGETS dynamic_transform_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)