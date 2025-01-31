#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

class GoalStartMarkerPublisher : public rclcpp::Node
{
public:
    geometry_msgs::msg::Point start_, goal_;

    GoalStartMarkerPublisher(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal);

private:
    void publishMarkers();

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

