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
find_package(pcl_conversions REQUIRED)

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
  pcl_conversions
  sensor_msgs)
rclcpp_components_register_nodes(scan_segmentation_component
  "scan_segmentation::ScanSegmentationComponent")
ament_export_libraries(scan_segmentation_component)

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
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
