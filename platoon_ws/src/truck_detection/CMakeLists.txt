cmake_minimum_required(VERSION 3.8)
project(truck_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(obstacle_detector REQUIRED)
# find_package(builtin_interfaces REQUIRED)
# find_package(rosidl_default_generators REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/CircleObstacle.msg"
#   "msg/Obstacles.msg"
#   "msg/SegmentObstacle.msg"
#   DEPENDENCIES geometry_msgs std_msgs builtin_interfaces 
# )

include_directories(
  include
)

# ───── Executable: circle_tracking_node ─────
add_library(circle_tracking
  src/circle_tracking.cpp
)

ament_target_dependencies(circle_tracking
  rclcpp
  geometry_msgs
  obstacle_detector
)

target_include_directories(circle_tracking PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(circle_tracking_node
  src/circle_tracking_node.cpp
)

target_link_libraries(circle_tracking_node
  circle_tracking
)

ament_target_dependencies(circle_tracking_node
  rclcpp
)

target_include_directories(circle_tracking_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ───── Executable: pointcloud2_roi_node ─────
add_library(pointcloud2_roi SHARED
  src/pointcloud2_roi.cpp
)

ament_target_dependencies(pointcloud2_roi
  rclcpp
  rclcpp_components
  sensor_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
)

target_include_directories(pointcloud2_roi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

rclcpp_components_register_nodes(pointcloud2_roi
  "truck_detection::PointCloud2ROI")

add_executable(pointcloud2_roi_node
  src/pointcloud2_roi_node.cpp
)

target_link_libraries(pointcloud2_roi_node
  pointcloud2_roi
)

ament_target_dependencies(pointcloud2_roi_node
  rclcpp
)

target_include_directories(pointcloud2_roi_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ───── Install ─────
install(TARGETS
  circle_tracking_node
  pointcloud2_roi_node
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
