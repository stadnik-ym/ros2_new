# cmake_minimum_required(VERSION 3.8)
# project(diff_drive)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# # find_package(PkgConfig REQUIRED)
# # pkg_check_modules(LGPIO REQUIRED lgpio)

# add_executable(motor_node src/motor_node.cpp)
# target_include_directories(motor_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
# target_compile_features(motor_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(
#   motor_node
#   "rclcpp"
#   "geometry_msgs"
#   "std_msgs"
#   "nav_msgs"
# )

# target_link_libraries(motor_node
# liblgpio)

# install(TARGETS motor_node
#   DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()



cmake_minimum_required(VERSION 3.8)
project(diff_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Find lgpio library manually (correct library name: lgpio, not lg)
find_library(LGPIO_LIB NAMES lgpio REQUIRED
  HINTS $ENV{LIBRARY_PATH}
  PATHS /nix/store/*/lib)

find_path(LGPIO_INCLUDE_DIR NAMES lgpio.h REQUIRED
  HINTS $ENV{CPATH}
  PATHS /nix/store/*/include)

message(STATUS "Found lgpio library: ${LGPIO_LIB}")
message(STATUS "Found lgpio headers: ${LGPIO_INCLUDE_DIR}")

add_executable(motor_node src/motor_node.cpp)
target_include_directories(motor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${LGPIO_INCLUDE_DIR})

target_compile_features(motor_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  motor_node
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
  "nav_msgs"
)

target_link_libraries(motor_node
  ${LGPIO_LIB})

install(TARGETS motor_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
