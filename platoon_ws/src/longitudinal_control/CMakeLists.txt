cmake_minimum_required(VERSION 3.8)
project(longitudinal_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)

# ───── Library (Gap Control) ─────
add_library(gap_control_node
  src/gap_control.cpp
)

target_include_directories(gap_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gap_control_node
  rclcpp
  std_msgs
)

# ───── Library (Velocity Control) ─────
add_library(velocity_control_node
  src/gap_control.cpp
)

target_include_directories(velocity_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(velocity_control_node
  rclcpp
  std_msgs
)

# ───── Executable Node ─────
add_executable(longitudinal_control_node
  src/main.cpp
  src/longitudinal_control_node.cpp
)

target_link_libraries(longitudinal_control_node
  gap_control_node
  velocity_control_node
)

ament_target_dependencies(longitudinal_control_node
  rclcpp
  std_msgs
)

# ───── Install ─────
install(TARGETS
  gap_control_node
  velocity_control_node
  longitudinal_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# ───── Tests (Optional) ─────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
