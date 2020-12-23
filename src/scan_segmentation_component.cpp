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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/algorithm/clamp.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/assign/list_of.hpp>

#include <cmath>
#include <algorithm>
#include <vector>
#include <string>

#define BOOST_GEOMETRY_DEBUG_HAS_SELF_INTERSECTIONS

namespace scan_segmentation
{
ScanSegmentationComponent::ScanSegmentationComponent(const rclcpp::NodeOptions & options)
: Node("scan_segmentation", options), buffer_(get_clock()), listener_(buffer_)
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
  declare_parameter("output_frame_id", "base_link");
  get_parameter("output_frame_id", output_frame_id_);
  declare_parameter("visualize_frame_id", "map");
  get_parameter("visualize_frame_id", visualize_frame_id_);
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, 1,
    std::bind(&ScanSegmentationComponent::scanCallback, this, std::placeholders::_1));
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/marker", 1);
}

boost::optional<geometry_msgs::msg::PointStamped> ScanSegmentationComponent::transform(
  geometry_msgs::msg::PointStamped point, std::string target_frame_id, bool exact)
{
  if (point.header.frame_id == target_frame_id) {
    return point;
  }
  if (exact) {
    tf2::TimePoint time_point = tf2::TimePoint(
      std::chrono::seconds(point.header.stamp.sec) +
      std::chrono::nanoseconds(point.header.stamp.nanosec));
    try {
      geometry_msgs::msg::TransformStamped transform_stamped =
        buffer_.lookupTransform(
        target_frame_id, point.header.frame_id,
        time_point, tf2::durationFromSec(1.0));
      tf2::doTransform(point, point, transform_stamped);
      return point;
    } catch (...) {
      return boost::none;
    }
  } else {
    tf2::TimePoint time_point = tf2::TimePoint(
      std::chrono::seconds(0) +
      std::chrono::nanoseconds(0));
    try {
      geometry_msgs::msg::TransformStamped transform_stamped =
        buffer_.lookupTransform(
        target_frame_id, point.header.frame_id,
        time_point, tf2::durationFromSec(1.0));
      tf2::doTransform(point, point, transform_stamped);
      return point;
    } catch (...) {
      return boost::none;
    }
  }
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
    double dist_threashold = boost::algorithm::clamp(
      std::min(
        d0,
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
      poly.points = buf;
      polygons.push_back(poly);
      buf.clear();
    }
  }
  if (is_connected[is_connected.size() - 1]) {
    std::vector<geometry_msgs::msg::Point32> start_poly = polygons[0].points;
    std::vector<geometry_msgs::msg::Point32> end_poly = polygons[polygons.size() - 1].points;
    end_poly.insert(end_poly.end(), start_poly.begin(), start_poly.end());
    polygons.erase(polygons.begin());
    polygons.erase(polygons.end());
    geometry_msgs::msg::Polygon poly;
    poly.points = end_poly;
    polygons.push_back(poly);
  }
  return polygons;
}

visualization_msgs::msg::MarkerArray ScanSegmentationComponent::generateDeleteMarker()
{
  visualization_msgs::msg::MarkerArray ret;
  visualization_msgs::msg::Marker marker;
  marker.action = marker.DELETEALL;
  ret.markers.push_back(marker);
  return ret;
}

visualization_msgs::msg::MarkerArray ScanSegmentationComponent::generateMarker(
  std::vector<geometry_msgs::msg::Polygon> polygons, std_msgs::msg::Header header)
{
  visualization_msgs::msg::MarkerArray marker_array;
  auto stamp = get_clock()->now();
  header.stamp = stamp;
  int count = 0;
  for (auto poly_itr = polygons.begin(); poly_itr != polygons.end(); poly_itr++) {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.header.frame_id = visualize_frame_id_;
    marker.ns = "polygon";
    marker.id = count;
    marker.type = marker.LINE_STRIP;
    marker.action = marker.ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    for (auto point_itr = poly_itr->points.begin(); point_itr != poly_itr->points.end();
      point_itr++)
    {
      geometry_msgs::msg::PointStamped p;
      p.point.x = point_itr->x;
      p.point.y = point_itr->y;
      p.point.z = point_itr->z;
      p.header = header;
      p.header.frame_id = output_frame_id_;
      auto p_transformed = transform(p, visualize_frame_id_, false);
      if (p_transformed) {
        marker.points.push_back(p_transformed->point);
        marker.colors.push_back(marker.color);
      } else {
        marker_array = visualization_msgs::msg::MarkerArray();
        return marker_array;
      }
    }
    marker_array.markers.push_back(marker);
    count++;
  }
  return marker_array;
}

std::vector<geometry_msgs::msg::Polygon> ScanSegmentationComponent::transformPolygons(
  std::vector<geometry_msgs::msg::Polygon> polygons, std_msgs::msg::Header header)
{
  std::vector<geometry_msgs::msg::Polygon> ret;
  for (auto poly_itr = polygons.begin(); poly_itr != polygons.end(); poly_itr++) {
    geometry_msgs::msg::Polygon poly;
    bool transform_succeeded = true;
    for (auto point_itr = poly_itr->points.begin(); point_itr != poly_itr->points.end();
      point_itr++)
    {
      geometry_msgs::msg::PointStamped p;
      p.point.x = point_itr->x;
      p.point.y = point_itr->y;
      p.point.z = point_itr->z;
      p.header = header;
      auto p_transformed = transform(p, output_frame_id_, false);
      if (p_transformed) {
        geometry_msgs::msg::Point32 p32;
        p32.x = p_transformed->point.x;
        p32.y = p_transformed->point.y;
        p32.z = p_transformed->point.z;
        poly.points.push_back(p32);
      } else {
        transform_succeeded = false;
      }
    }
    if (transform_succeeded) {
      ret.push_back(poly);
    }
  }
  return ret;
}

void ScanSegmentationComponent::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr data)
{
  auto points = getPoints(data);
  auto polygons = getPolygons(points);
  auto inflated_polygons = inflatePolygons(polygons);
  marker_pub_->publish(generateDeleteMarker());
  if (inflated_polygons) {
    auto marker = generateMarker(inflated_polygons.get(), data->header);
    marker_pub_->publish(marker);
  }
}

boost::optional<std::vector<geometry_msgs::msg::Polygon>>
ScanSegmentationComponent::inflatePolygons(
  std::vector<geometry_msgs::msg::Polygon> polygons)
{
  namespace bg = boost::geometry;
  namespace trans = bg::strategy::transform;
  typedef bg::model::d2::point_xy<double> boost_point;
  typedef bg::model::polygon<boost_point> boost_polygon;
  std::vector<geometry_msgs::msg::Polygon> ret;
  for (auto poly_itr = polygons.begin(); poly_itr != polygons.end(); poly_itr++) {
    boost_polygon union_poly;
    for (auto point_itr = poly_itr->points.begin(); point_itr != poly_itr->points.end();
      point_itr++)
    {
      boost_polygon poly;
      double d = std::hypot(point_itr->x, point_itr->y);
      double radius = boost::algorithm::clamp(
        d * distance_ratio_ * 0.5, min_segment_distance_,
        max_segment_distance_);
      std::array<double, 8> x, y;
      for (int i = 0; i < 8; i++) {
        x[i] = point_itr->x + radius * std::sin(M_PI_4 * i);
        y[i] = point_itr->y + radius * std::cos(M_PI_4 * i);
      }
      bg::exterior_ring(poly) =
        boost::assign::list_of<boost_point>(x[0], y[0])(x[1], y[1])(x[2], y[2])(x[3], y[3])(
        x[4],
        y[4])(x[5], y[5])(x[6], y[6])(x[7], y[7])(x[0], y[0]);
      if (point_itr == poly_itr->points.begin()) {
        union_poly = poly;
      } else {
        std::vector<boost_polygon> out;
        bg::union_(union_poly, poly, out);
        if (out.size() == 0) {
          return boost::none;
        } else {
          union_poly = out[0];
        }
      }
    }
    boost_polygon hull;
    bg::convex_hull(union_poly, hull);
    geometry_msgs::msg::Polygon polygon;
    for (auto it = boost::begin(boost::geometry::exterior_ring(hull));
      it != boost::end(boost::geometry::exterior_ring(hull)); ++it)
    {
      double x = bg::get<0>(*it);
      double y = bg::get<1>(*it);
      geometry_msgs::msg::Point32 p;
      p.x = x;
      p.y = y;
      p.z = 0.0;
      polygon.points.push_back(p);
    }
    ret.push_back(polygon);
  }
  return ret;
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
