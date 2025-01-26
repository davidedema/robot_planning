#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "graph_generator/combinatorial_based/utilities.hpp"


class MapEdgePublisherNode : public rclcpp::Node
{
public:
    bg::model::multi_polygon<polygon_t> multi_polygon_;
    std::string topic_;

    MapEdgePublisherNode(
        boost::geometry::model::multi_polygon<polygon_t> multi_polygon, std::string topic);

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr edge_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void publishEdges();
    void publishPoints();

    template <typename Ring>
    void addEdgesToMarker(const Ring &ring,
                          visualization_msgs::msg::Marker &marker, visualization_msgs::msg::Marker &marker1);
};

class SimpleEdgePublisherNode : public rclcpp::Node
{
public:
    std::vector<line_t> edges_;
    std::string topic_;

    SimpleEdgePublisherNode(
        std::vector<line_t> edges_, std::string topic);

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr edge_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void publishEdges();
};

class SimplePointPublisherNode : public rclcpp::Node
{
public:
    std::vector<point_t> points_;
    std::string topic_;

    SimplePointPublisherNode(
        std::vector<point_t> points_, std::string topic);

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr point_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void publishPoints();
};
