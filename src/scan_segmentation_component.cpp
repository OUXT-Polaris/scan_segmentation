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

#include <scan_segmentation/scan_segmentation_component.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <cmath>
#include <boost/algorithm/clamp.hpp>

namespace scan_segmentation
{
ScanSegmentationComponent::ScanSegmentationComponent(const rclcpp::NodeOptions & options)
: Node("scan_segmentation", options)
{
  std::string scan_topic;
  declare_parameter("scan_topic", "/obstacle_scan");
  get_parameter("scan_topic", scan_topic);
  declare_parameter("max_segment_distance", 0.8);
  get_parameter("max_segment_distance", max_segment_distance_);
  declare_parameter("min_segment_distance", 0.1);
  get_parameter("min_segment_distance", min_segment_distance_);
  declare_parameter("distance_ratio", 2.0);
  get_parameter("distance_ratio", distance_ratio_);
  declare_parameter("theta_threashold", M_PI * 0.1);
  get_parameter("theta_threashold", theta_threashold_);
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, 1,
    std::bind(&ScanSegmentationComponent::scanCallback, this, std::placeholders::_1));
}

std::vector<geometry_msgs::msg::Polygon> ScanSegmentationComponent::getPolygons(
  std::vector<geometry_msgs::msg::Point32> points)
{
  std::vector<geometry_msgs::msg::Polygon> polygons;
  std::vector<bool> is_connected;
  unsigned int num_points = points.size();
  if (num_points == 0) {
    return polygons;
  }
  for (unsigned int i = 0; i < num_points; i++) {
    geometry_msgs::msg::Point32 p0, p1, p;
    if (i == (num_points - 1)) {
      p0 = points[num_points - 1];
      p1 = points[0];
    } else {
      p0 = points[i];
      p1 = points[i + 1];
    }
    double l = std::hypot(p0.x - p1.x, p0.y - p1.y);
    double d0 = std::hypot(p0.x, p0.y);
    double d1 = std::hypot(p1.x, p1.y);
    double theta = std::acos((p0.x * p1.x + p0.y * p1.y) / (d0 * d1));
    if (theta > theta_threashold_) {
      is_connected.push_back(false);
      continue;
    }
    double dist_threashold = boost::algorithm::clamp(std::min(d0,
        d1) * distance_ratio_, min_segment_distance_,
        max_segment_distance_);
    if (l > dist_threashold) {
      is_connected.push_back(false);
      continue;
    }
    is_connected.push_back(true);
  }
  if (std::count(is_connected.begin(), is_connected.end(), false) <= 1) {
    geometry_msgs::msg::Polygon poly;
    poly.points = points;
    polygons.push_back(poly);
    return polygons;
  }
  std::vector<geometry_msgs::msg::Point32> buf;
  for (unsigned int i = 0; i < num_points; i++) {
    buf.push_back(points[i]);
    if (!is_connected[i]) {
      geometry_msgs::msg::Polygon poly;
      poly.points = points;
      polygons.push_back(poly);
      buf.clear();
    }
  }
  if (is_connected[is_connected.size()-1]) {
    std::vector<geometry_msgs::msg::Point32> start_poly = polygons[0].points;
    std::vector<geometry_msgs::msg::Point32> end_poly = polygons[polygons.size()-1].points;
    end_poly.insert(end_poly.end(), start_poly.begin(), start_poly.end());
    polygons.erase(polygons.begin());
    polygons.erase(polygons.end());
    geometry_msgs::msg::Polygon poly;
    poly.points = end_poly;
    polygons.push_back(poly);
  }
  return polygons;
}

void ScanSegmentationComponent::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr data)
{
  auto points = getPoints(data);
  auto polygons = getPolygons(points);
  std::cout << polygons.size() << std::endl;
}

std::vector<geometry_msgs::msg::Point32> ScanSegmentationComponent::getPoints(
  const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  std::vector<geometry_msgs::msg::Point32> ret;
  for (int i = 0; i < static_cast<int>(scan->ranges.size()); i++) {
    if (scan->range_max >= scan->ranges[i] && scan->ranges[i] >= scan->range_min) {
      double theta = scan->angle_min + scan->angle_increment * static_cast<double>(i);
      geometry_msgs::msg::Point32 p;
      p.x = scan->ranges[i] * std::cos(theta);
      p.y = scan->ranges[i] * std::sin(theta);
      p.z = 0.0;
      ret.push_back(p);
    }
  }
  return ret;
}
}  // namespace scan_segmentation

RCLCPP_COMPONENTS_REGISTER_NODE(scan_segmentation::ScanSegmentationComponent)
