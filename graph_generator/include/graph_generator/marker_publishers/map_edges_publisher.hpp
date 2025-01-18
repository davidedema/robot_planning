#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "graph_generator/combinatorial_based/map_construction.hpp"

namespace bg = boost::geometry;

typedef  bg::model::multi_polygon<polygon_t> multi_polygon_t;
typedef bg::model::segment<point_t> Line;

using point_xy_t = boost::geometry::model::d2::point_xy<double>;
using polygon_xy_t = boost::geometry::model::polygon<point_xy_t>;
using multipolygon_xy_t = bg::model::multi_polygon<polygon_xy_t>;
using line_xy_t = bg::model::linestring<point_xy_t>;

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
                          visualization_msgs::msg::Marker &marker);
};

class SimpleEdgePublisherNode : public rclcpp::Node
{
public:
    std::vector<line_xy_t> edges_;
    std::string topic_;

    SimpleEdgePublisherNode(
        std::vector<line_xy_t> edges_, std::string topic);

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr edge_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void publishEdges();
};
