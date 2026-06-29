cmake_minimum_required(VERSION 3.16)
project(ld06_lidar)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Find libserialport via pkg-config (Nix-friendly)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)

# Library with core components
add_library(ld06_lib STATIC
  src/ld06_utils.cpp
  src/ld06_serial_libserialport.cpp
)

target_include_directories(ld06_lib
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${LIBSERIALPORT_INCLUDE_DIRS}
)

target_link_libraries(ld06_lib
  PUBLIC
    ${LIBSERIALPORT_LIBRARIES}
)

# Main executable
add_executable(ld06_node
  src/ld06_ros_node.cpp
)

add_executable(ld06_safety_node
  src/ld06_safety_node.cpp
)

target_include_directories(ld06_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${LIBSERIALPORT_INCLUDE_DIRS}
)

target_link_libraries(ld06_node
  PRIVATE
    ld06_lib
)

ament_target_dependencies(ld06_node
  PUBLIC
    rclcpp
    sensor_msgs
    std_msgs
)

ament_target_dependencies(ld06_safety_node
  PUBLIC
    rclcpp
    sensor_msgs
    std_msgs
    geometry_msgs
)

# Install
install(
  TARGETS
    ld06_node
    ld06_safety_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
