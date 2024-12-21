#include <graph_generator/marker_publishers/map_edges_publisher.hpp>

MapEdgePublisherNode::MapEdgePublisherNode(
    boost::geometry::model::multi_polygon<polygon_t> multi_polygon, std::string topic)
    : Node("map_publisher")
{
    bg::correct(multi_polygon);
    multi_polygon_ = multi_polygon;
    topic_ = topic;
    // Create publisher
    edge_publisher_ =
        this->create_publisher<visualization_msgs::msg::Marker>(topic_, 10);
    // Publish edges in a timer callback
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MapEdgePublisherNode::publishEdges, this));
}

void MapEdgePublisherNode::publishEdges()
{
    if (edge_publisher_->get_subscription_count() > 0 ||
        edge_publisher_->get_intra_process_subscription_count() > 0)
    {

        // Prepare Marker message
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "out";
        marker.id = 0;
        marker.type =
            visualization_msgs::msg::Marker::LINE_LIST; // Use LINE_LIST for edges
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.1; // Line width
        marker.color.a = 1.0; // Alpha
        marker.color.r = 1.0; // Red
        marker.color.g = 1.0; // Red

        auto marker2 = visualization_msgs::msg::Marker();
        marker2.header.frame_id = "map";
        marker2.header.stamp = this->get_clock()->now();
        marker2.ns = "in";
        marker2.id = 0;
        marker2.type =
            visualization_msgs::msg::Marker::LINE_LIST; // Use LINE_LIST for edges
        marker2.action = visualization_msgs::msg::Marker::ADD;
        marker2.scale.x = 0.1; // Line width
        marker2.color.a = 1.0; // Alpha
        marker2.color.b = 1.0;
        marker2.color.r = 1.0;

        // Extract edges and populate marker points
        for (const auto &polygon : this->multi_polygon_)
        {
            // Outer ring
            addEdgesToMarker(polygon.outer(), marker);

            // Inner rings
            for (const auto &inner_ring : polygon.inners())
            {
                addEdgesToMarker(inner_ring, marker2);
            }
        }
        // Publish marker
        edge_publisher_->publish(marker);
        edge_publisher_->publish(marker2);
    }

}
template <typename Ring>

void MapEdgePublisherNode::addEdgesToMarker(const Ring &ring,
                                            visualization_msgs::msg::Marker &marker)
{
    for (std::size_t i = 0; i < ring.size(); ++i)
    {
        const auto &p1 = ring[i];
        const auto &p2 = ring[(i + 1) % ring.size()]; // Next point (wrap around)

        // Convert to geometry_msgs::msg::Point
        geometry_msgs::msg::Point ros_point1, ros_point2;
        ros_point1.x = bg::get<0>(p1);
        ros_point1.y = bg::get<1>(p1);
        ros_point1.z = 0.0; // Assume z = 0 for 2D points
        ros_point2.x = bg::get<0>(p2);
        ros_point2.y = bg::get<1>(p2);
        ros_point2.z = 0.0;

        // Add to marker
        marker.points.push_back(ros_point1);
        marker.points.push_back(ros_point2);
    }
}

/////Simple edges Publisher//////////////////////////////////////////////

SimpleEdgePublisherNode::SimpleEdgePublisherNode(
    std::vector<line_xy_t> edges, std::string topic)
    : Node("edge_pubslisher")
{
    edges_ = edges;
    topic_ = topic;
    // Create publisher
    edge_publisher_ =
        this->create_publisher<visualization_msgs::msg::Marker>(topic_, 10);
    // Publish edges in a timer callback
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&SimpleEdgePublisherNode::publishEdges, this));
}

void SimpleEdgePublisherNode::publishEdges()
{
    if (edge_publisher_->get_subscription_count() > 0 ||
        edge_publisher_->get_intra_process_subscription_count() > 0)
    {

        // Prepare Marker message
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "out";
        marker.id = 0;
        marker.type =
            visualization_msgs::msg::Marker::LINE_LIST; // Use LINE_LIST for edges
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.1; // Line width
        marker.color.a = 1.0; // Alpha
        marker.color.r = 1.0; // Red
        marker.color.g = 1.0; // Red

        for (std::size_t i = 0; i < edges_.size(); ++i)
        {
            auto edge = edges_[i];

            // Convert to geometry_msgs::msg::Point
            geometry_msgs::msg::Point ros_point1, ros_point2;
            ros_point1.x = edge[0].x();
            ros_point1.y = edge[0].y();
            ros_point1.z = 0.0; // Assum 2D oints
            ros_point2.x = edge[1].x();
            ros_point2.y = edge[1].y();
            ros_point2.z = 0.0;

            // Add to marker
            marker.points.push_back(ros_point1);
            marker.points.push_back(ros_point2);
        }

        // Publish marker
        edge_publisher_->publish(marker);
    }
}