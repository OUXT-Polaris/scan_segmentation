cmake_minimum_required(VERSION 3.5)
project(scan_segmentation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_apps_msgs REQUIRED)

include_directories(
  include
)

add_library(scan_segmentation_component SHARED
  src/scan_segmentation_component.cpp)
target_compile_definitions(scan_segmentation_component
  PRIVATE "SCAN_SEGMENTATION_SCAN_SEGMENTATION_COMPONENT_BUILDING_DLL")
ament_target_dependencies(scan_segmentation_component
  rclcpp
  rclcpp_components
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  pcl_apps_msgs
  sensor_msgs)
rclcpp_components_register_nodes(scan_segmentation_component
  "scan_segmentation::ScanSegmentationComponent")
ament_export_libraries(scan_segmentation_component)

add_executable(scan_segmentation_node
  src/scan_segmentation_node.cpp
)
target_link_libraries(scan_segmentation_node scan_segmentation_component)
ament_target_dependencies(scan_segmentation_node
  rclcpp
  rclcpp_components)

install(TARGETS
  scan_segmentation_node
  DESTINATION lib/scan_segmentation
)
install(TARGETS scan_segmentation_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  DIRECTORY "include/"
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
