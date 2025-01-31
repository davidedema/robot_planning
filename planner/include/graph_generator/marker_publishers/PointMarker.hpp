#include "graph_generator/combinatorial_based/utilities.hpp"

class PointMarkerNode : public rclcpp::Node
{
    public:
        PointMarkerNode();
        int get_points_to_sample();
        void setSampledPoints(const std::vector<std::vector<double>> &sampled_points);
        void setPathPoints(const std::vector<std::vector<double>> &path_points);
        void setDubinsCurves(const std::vector<dubins_curve> &curves);
        void send_nav2(const nav_msgs::msg::Path &path1, const nav_msgs::msg::Path &path2);
        
    private:
        std::vector<std::vector<double>> sampled_points_;
        std::vector<std::vector<double>> path_points_;
        std::vector<struct dubins_curve> dubins_curves_;

        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sampled_points_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr dubins_curve_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr shelfino1_nav2_path_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr shelfino2_nav2_path_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        int timer_interval_ms_;
        int points_to_sample;

        void checkAndPublishMarkers();
        void publishMarkers();
        void publishSampledPointsMarker();
        void publishPathMarker();
        void publishDubinsCurveMarkers();
};
