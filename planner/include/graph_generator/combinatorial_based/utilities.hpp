#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <chrono>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include "graph_generator/map_generator.hpp"
#include "graph_generator/utils/orchestrator.hpp"

#include "clipper.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "rclcpp/qos.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "graph_generator/utils/dubins.hpp"
#include <nav_msgs/msg/path.hpp>

#define SHELFINO_INFLATION 0.5
#define CIRCLE_APPROXIMATION 36

namespace bg = boost::geometry;
using namespace ClipperLib;


typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_t;
typedef bg::model::segment<point_t> line_t;
typedef boost::geometry::model::polygon<point_t> polygon_t;
typedef boost::geometry::model::multi_polygon<polygon_t> multi_polygon_t;

typedef std::vector<double> pose_t;
const double CLIPPER_SCALE_FACTOR = 1000.0;

static const rmw_qos_profile_t qos_profile_custom = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    10,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};

/* //bool is_tangent(const line_xy_t &line, const polygon_xy_t &border, const multipolygon_xy_t &inner_polygons);
polygon_xy_t convertPolygon(const polygon_t &polygon);
multipolygon_xy_t convertMultiPolygon(const multi_polygon_t &multipolygon);
 */

std::vector<std::vector<double>> convert_points(std::vector<point_t> points);

