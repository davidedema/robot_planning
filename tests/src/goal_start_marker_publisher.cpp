#include "tests/goal_start_marker_publisher.hpp"

GoalStartMarkerPublisher::GoalStartMarkerPublisher(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal)
    : Node("marker_publisher"), start_(start), goal_(goal)
{
    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("start_goal_markers", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&GoalStartMarkerPublisher::publishMarkers, this));
}

void GoalStartMarkerPublisher::publishMarkers()
{
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map"; // Adjust as needed
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "example_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set the scale of the points
    marker.scale.x = .3; // Width of the points
    marker.scale.y = .3; // Height of the points

    // Set the color of the points
    marker.color.a = 1.0; // Alpha transparency
    marker.color.r = 1.0; // Red
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    // Add the points to the marker
    marker.points.push_back(start_);

    auto marker2 = visualization_msgs::msg::Marker();
    marker2.header.frame_id = "map"; // Adjust as needed
    marker2.header.stamp = this->get_clock()->now();
    marker2.ns = "example_namespace";
    marker2.id = 1;
    marker2.type = visualization_msgs::msg::Marker::POINTS;
    marker2.action = visualization_msgs::msg::Marker::ADD;

    // Set the scale of the points
    marker2.scale.x = .3; // Width of the points
    marker2.scale.y = .3; // Height of the points

    // Set the color of the points
    marker2.color.a = 1.0; // Alpha transparency
    marker2.color.r = 0.0; // Red
    marker2.color.g = 1.0;
    marker2.color.b = 0.0;

    marker2.points.push_back(goal_);

    // Publish the marker
    publisher_->publish(marker);
    publisher_->publish(marker2);

    RCLCPP_INFO(this->get_logger(), "Published marker points");
}
