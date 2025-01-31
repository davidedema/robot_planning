#include "graph_generator/marker_publishers/PointMarker.hpp"

PointMarkerNode::PointMarkerNode()
    : Node("point_marker_node"), timer_interval_ms_(500)
{
    // Publisher for sampled points
    sampled_points_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "sampled_points_marker", rclcpp::QoS(10));

    // Publisher for the path
    path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "path_marker", rclcpp::QoS(10));

    // Publisher for the Dubins curve
    dubins_curve_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "dubins_curve_marker", rclcpp::QoS(10));

    shelfino1_nav2_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "shelfino1/plan1", rclcpp::QoS(10));
    shelfino2_nav2_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "shelfino2/plan1", rclcpp::QoS(10));
    // Timer to periodically check for subscribers and publish
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_interval_ms_),
        std::bind(&PointMarkerNode::checkAndPublishMarkers, this));

    // parameters
    this->declare_parameter<int>("points_to_sample", 3000);

    int points_to_sample = this->get_parameter("points_to_sample").as_int();

    RCLCPP_INFO(this->get_logger(), "Point Marker Node started");
}

int PointMarkerNode::get_points_to_sample()
{
    return points_to_sample;
}

void PointMarkerNode::setSampledPoints(const std::vector<std::vector<double>> &sampled_points)
{
    sampled_points_ = sampled_points;
}

void PointMarkerNode::setPathPoints(const std::vector<std::vector<double>> &path_points)
{
    path_points_ = path_points;
}

void PointMarkerNode::setDubinsCurves(const std::vector<dubins_curve> &curves)
{
    dubins_curves_ = curves;
}
void PointMarkerNode::send_nav2(const nav_msgs::msg::Path &path1, const nav_msgs::msg::Path &path2)
{
    shelfino1_nav2_path_pub_->publish(path1);
    shelfino2_nav2_path_pub_->publish(path2);
}

void PointMarkerNode::checkAndPublishMarkers()
{
    if (sampled_points_pub_->get_subscription_count() > 0 ||
        path_pub_->get_subscription_count() > 0 ||
        dubins_curve_pub_->get_subscription_count() > 0)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "Subscriber detected. Publishing markers.");
        publishMarkers();
    }
}

void PointMarkerNode::publishMarkers()
{
    publishSampledPointsMarker();
    publishPathMarker();
    publishDubinsCurveMarkers();
}

void PointMarkerNode::publishSampledPointsMarker()
{
    if (sampled_points_.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No sampled points to publish.");
        return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map"; // Adjust frame_id as needed
    marker.header.stamp = this->now();
    marker.ns = "sampled_points";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST; // Individual points
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Define marker properties
    marker.scale.x = 0.05; // Point size
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0; // Blue color
    marker.color.a = 1.0;

    // Fill marker points
    for (const auto &point : sampled_points_)
    {
        if (point.size() == 2)
        {
            geometry_msgs::msg::Point p;
            p.x = point[0];
            p.y = point[1];
            p.z = 0.0;
            marker.points.push_back(p);
        }
    }

    // Publish the marker
    sampled_points_pub_->publish(marker);
    RCLCPP_INFO(this->get_logger(), "Published %zu sampled points.", marker.points.size());
}

void PointMarkerNode::publishPathMarker()
{
    if (path_points_.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No path points to publish.");
        return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map"; // Adjust frame_id as needed
    marker.header.stamp = this->now();
    marker.ns = "path";
    marker.id = 2;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP; // Connected path
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Define marker properties
    marker.scale.x = 0.05; // Path width
    marker.color.r = 1.0;  // Red color
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Fill marker points
    for (const auto &point : path_points_)
    {
        if (point.size() == 2)
        {
            geometry_msgs::msg::Point p;
            p.x = point[0];
            p.y = point[1];
            p.z = 0.0;
            marker.points.push_back(p);
        }
    }

    // Publish the marker
    path_pub_->publish(marker);
    RCLCPP_INFO(this->get_logger(), "Published path with %zu points.", marker.points.size());
}
void PointMarkerNode::publishDubinsCurveMarkers()
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "dubins_curve";
    marker.id = 3;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP; // Connected curves
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Define marker properties
    marker.scale.x = 0.05; // Curve line width
    marker.color.r = 0.0;
    marker.color.g = 1.0; // Green color
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Fill marker points for all Dubins curves
    for (const auto &curve : dubins_curves_)
    {
        auto add_arc_points = [&](const dubins_arc &arc)
        {
            double step = 0.1; // Step size for sampling points along the arc
            double theta = arc.th0;

            if (std::abs(arc.k) < 1e-6)
            {
                // Straight-line case
                for (double s = 0; s <= arc.L; s += step)
                {
                    double x = arc.x0 + s * cos(theta);
                    double y = arc.y0 + s * sin(theta);
                    geometry_msgs::msg::Point p;
                    p.x = x;
                    p.y = y;
                    p.z = 0.0;
                    marker.points.push_back(p);
                }
            }
            else
            {
                // Curved case
                double radius = 1.0 / arc.k;
                for (double s = 0; s <= arc.L; s += step)
                {
                    double x = arc.x0 + radius * (sin(theta + arc.k * s) - sin(theta));
                    double y = arc.y0 + radius * (cos(theta) - cos(theta + arc.k * s));
                    geometry_msgs::msg::Point p;
                    p.x = x;
                    p.y = y;
                    p.z = 0.0;
                    marker.points.push_back(p);
                }
            }
        };

        // Add points from all arcs in the Dubins curve
        add_arc_points(curve.a1);
        add_arc_points(curve.a2);
        add_arc_points(curve.a3);
    }

    // Publish the marker
    dubins_curve_pub_->publish(marker);
    RCLCPP_INFO(this->get_logger(), "Published Dubins curves with %zu points.", marker.points.size());
}
