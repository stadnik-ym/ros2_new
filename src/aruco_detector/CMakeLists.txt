cmake_minimum_required(VERSION 3.8)
project(aruco_detector)

# Поиск необходимых пакетов и зависимостей
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# ==========================================
# 1. Сборка узла aruco_detector_node
# ==========================================
add_executable(aruco_detector_node
  src/aruco_detector_node.cpp
)

target_include_directories(aruco_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(aruco_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(aruco_detector_node
  ${OpenCV_LIBS}
)

# ==========================================
# 2. Сборка узла camera_publisher_node
# ==========================================
add_executable(camera_publisher_node
  src/camera_publisher_node.cpp
)

target_include_directories(camera_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(camera_publisher_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(camera_publisher_node
  ${OpenCV_LIBS}
)

# ==========================================
# Настройки установки (Узлы и Заголовки)
# ==========================================
# Установка исполняемых файлов в lib/aruco_detector для ros2 run
install(TARGETS
  aruco_detector_node
  camera_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Установка заголовочных файлов (для cv_bridge.hpp)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
