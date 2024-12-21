#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tests/goal_start_marker_publisher.hpp>


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    geometry_msgs::msg::Point start, goal;
    start.x = 1.0;
    start.y = 1.0;
    start.z = 0.0;

    goal.x = 2.0;
    goal.y = 2.0;
    goal.z = 0.0;

    auto node = std::make_shared<GoalStartMarkerPublisher>(start,goal);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
