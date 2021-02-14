// Copyright (c) 2020 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SCAN_SEGMENTATION__SCAN_SEGMENTATION_COMPONENT_HPP_
#define SCAN_SEGMENTATION__SCAN_SEGMENTATION_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define SCAN_SEGMENTATION_SCAN_SEGMENTATION_COMPONENT_EXPORT __attribute__((dllexport))
#define SCAN_SEGMENTATION_SCAN_SEGMENTATION_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define SCAN_SEGMENTATION_SCAN_SEGMENTATION_COMPONENT_EXPORT __declspec(dllexport)
#define SCAN_SEGMENTATION_SCAN_SEGMENTATION_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef SCAN_SEGMENTATION_SCAN_SEGMENTATION_COMPONENT_BUILDING_DLL
#define SCAN_SEGMENTATION_SCAN_SEGMENTATION_COMPONENT_PUBLIC \
  SCAN_SEGMENTATION_SCAN_SEGMENTATION_COMPONENT_EXPORT
#else
#define SCAN_SEGMENTATION_SCAN_SEGMENTATION_COMPONENT_PUBLIC \
  SCAN_SEGMENTATION_SCAN_SEGMENTATION_COMPONENT_IMPORT
#endif
#define SCAN_SEGMENTATION_SCAN_SEGMENTATION_COMPONENT_PUBLIC_TYPE \
  SCAN_SEGMENTATION_SCAN_SEGMENTATION_COMPONENT_PUBLIC
#define SCAN_SEGMENTATION_SCAN_SEGMENTATION_COMPONENT_LOCAL
#else
#define SCAN_SEGMENTATION_SCAN_SEGMENTATION_COMPONENT_EXPORT __attribute__((visibility("default")))
#define SCAN_SEGMENTATION_SCAN_SEGMENTATION_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define SCAN_SEGMENTATION_SCAN_SEGMENTATION_COMPONENT_PUBLIC __attribute__((visibility("default")))
#define SCAN_SEGMENTATION_SCAN_SEGMENTATION_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define SCAN_SEGMENTATION_SCAN_SEGMENTATION_COMPONENT_PUBLIC
#define SCAN_SEGMENTATION_SCAN_SEGMENTATION_COMPONENT_LOCAL
#endif
#define SCAN_SEGMENTATION_SCAN_SEGMENTATION_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_apps_msgs/msg/polygon_array.hpp>

#include <boost/optional.hpp>

#include <vector>
#include <string>

namespace scan_segmentation
{
class ScanSegmentationComponent : public rclcpp::Node
{
public:
  SCAN_SEGMENTATION_SCAN_SEGMENTATION_COMPONENT_PUBLIC
  explicit ScanSegmentationComponent(const rclcpp::NodeOptions & options);

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr data);
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<pcl_apps_msgs::msg::PolygonArray>::SharedPtr polygon_pub_;
  std::vector<geometry_msgs::msg::Point32> getPoints(
    const sensor_msgs::msg::LaserScan::SharedPtr scan);
  std::vector<geometry_msgs::msg::Polygon> getPolygons(
    std::vector<geometry_msgs::msg::Point32> points);
  std::vector<geometry_msgs::msg::Polygon> transformPolygons(
    std::vector<geometry_msgs::msg::Polygon> polygons, std_msgs::msg::Header header);
  boost::optional<std::vector<geometry_msgs::msg::Polygon>> inflatePolygons(
    std::vector<geometry_msgs::msg::Polygon> polygons);
  visualization_msgs::msg::MarkerArray generateMarker(
    std::vector<geometry_msgs::msg::Polygon> polygons, std_msgs::msg::Header header);
  visualization_msgs::msg::MarkerArray generateDeleteMarker();
  boost::optional<geometry_msgs::msg::PointStamped> transform(
    geometry_msgs::msg::PointStamped point, std::string target_frame_id, bool exact = false);
  double max_segment_distance_;
  double min_segment_distance_;
  double distance_ratio_;
  double theta_threashold_;
  double range_max_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::string output_frame_id_;
  std::string visualize_frame_id_;
  size_t previous_marker_size_;
};
}  // namespace scan_segmentation

#endif  // SCAN_SEGMENTATION__SCAN_SEGMENTATION_COMPONENT_HPP_
