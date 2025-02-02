#include <graph_generator/marker_publishers/map_edges_publisher.hpp>

MapEdgePublisherNode::MapEdgePublisherNode(
    boost::geometry::model::multi_polygon<polygon_t> multi_polygon, std::string topic)
    : Node(topic)
{
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
        // Prepare Marker message
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "out";
        marker.id = 0;
        marker.type =
            visualization_msgs::msg::Marker::LINE_LIST; // Use LINE_LIST for edges
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.01; // Line width
        marker.color.a = 1.0;  // Alpha
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        // Prepare Marker message
        auto marker1 = visualization_msgs::msg::Marker();
        marker1.header.frame_id = "map";
        marker1.header.stamp = this->get_clock()->now();
        marker1.ns = "out1";
        marker1.id = 2;
        marker1.type =
            visualization_msgs::msg::Marker::POINTS; // Use LINE_LIST for edges
        marker1.action = visualization_msgs::msg::Marker::ADD;
        marker1.scale.x = 0.1; // Line width
        marker1.scale.y = 0.1; // Line width

        marker1.color.a = 1.0; // Alpha
        marker1.color.r = 1.0;
        marker1.color.g = 1.0;

        // Extract edges and populate marker points
        for (const auto &polygon : this->multi_polygon_)
        {
            // Outer ring
            addEdgesToMarker(polygon.outer(), marker,marker1);

        }
        // Publish marker
        edge_publisher_->publish(marker);
        edge_publisher_->publish(marker1);
    }

}
template <typename Ring>

void MapEdgePublisherNode::addEdgesToMarker(const Ring &ring,
                                            visualization_msgs::msg::Marker &marker, visualization_msgs::msg::Marker &marker1)
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
        marker1.points.push_back(ros_point1);
        marker1.points.push_back(ros_point2);
    }
}

void MapEdgePublisherNode::publishPoints()
{
    if (edge_publisher_->get_subscription_count() > 0 ||
        edge_publisher_->get_intra_process_subscription_count() > 0)
    {
        // Create a Marker message
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map"; // Set to the appropriate frame
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "polygons";
        marker.id = 0; // You can increment this if you want to add multiple markers
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the point size (in RViz)
        marker.scale.x = 0.1; // Size of the points
        marker.scale.y = 0.1;
        marker.color.a = 1.0; // Fully opaque
        marker.color.r = 1.0; // Red color

        // Iterate over the multi-polygon and extract points
        size_t id = 0; // Unique marker id (for each point)
        for (const auto &polygon : this->multi_polygon_)
        {
            // Process outer points (boundary)
            for (const auto &pt : polygon.outer())
            {
                geometry_msgs::msg::Point p;
                p.x = boost::geometry::get<0>(pt); // x-coordinate
                p.y = boost::geometry::get<1>(pt); // y-coordinate
                p.z = 0.0;                         // Set z-coordinate to 0 (if it's 2D)
                marker.points.push_back(p);

                // Optionally, set unique IDs for each point in the marker
                marker.id = id++;
            }

            // Process inner points (holes)
            for (const auto &hole : polygon.inners())
            {
                for (const auto &pt : hole)
                {
                    geometry_msgs::msg::Point p;
                    p.x = boost::geometry::get<0>(pt); // x-coordinate
                    p.y = boost::geometry::get<1>(pt); // y-coordinate
                    p.z = 0.0;                         // Set z-coordinate to 0 (if it's 2D)
                    marker.points.push_back(p);

                    // Optionally, set unique IDs for each point in the marker
                    marker.id = id++;
                }
            }
        }

        // Publish marker
        edge_publisher_->publish(marker);
    }
}

/////Simple edges Publisher//////////////////////////////////////////////

SimpleEdgePublisherNode::SimpleEdgePublisherNode(
    std::vector<line_t> edges, std::string topic)
    : Node(topic)
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
        marker.scale.x = 0.01; // Line width
        marker.color.a = 1.0; // Alpha
        marker.color.r = 1.0;
        marker.color.g = .5;
        // Prepare Marker message
        auto marker1 = visualization_msgs::msg::Marker();
        marker1.header.frame_id = "map";
        marker1.header.stamp = this->get_clock()->now();
        marker1.ns = "out1";
        marker1.id = 2;
        marker1.type =
            visualization_msgs::msg::Marker::POINTS; // Use LINE_LIST for edges
        marker1.action = visualization_msgs::msg::Marker::ADD;
        marker1.scale.x = 0.1; // Line width
        marker1.scale.y = 0.1; // Line width

        marker1.color.a = 1.0;  // Alpha
        marker1.color.r = 1.0;
        marker1.color.g = .5;

        for (std::size_t i = 0; i < edges_.size(); ++i)
        {
            auto edge = edges_[i];

            // Convert to geometry_msgs::msg::Point
            geometry_msgs::msg::Point ros_point1, ros_point2;
            ros_point1.x = bg::get<0,0>(edge);
            ros_point1.y = bg::get<0,1>(edge);
            ros_point1.z = 0.0; // Assum 2D oints
            ros_point2.x = bg::get<1,0>(edge);
            ros_point2.y = bg::get<1,1>(edge);
            ros_point2.z = 0.0;

            // Add to marker
            marker.points.push_back(ros_point1);
            marker.points.push_back(ros_point2);
            marker1.points.push_back(ros_point1);
            marker1.points.push_back(ros_point2);
        }

        // Publish marker
        edge_publisher_->publish(marker);
        edge_publisher_->publish(marker1);
    }
}

/////Simple points Publisher//////////////////////////////////////////////

SimplePointPublisherNode::SimplePointPublisherNode(
    std::vector<point_t> points, std::string topic)
    : Node(topic)
{
    points_ = points;
    topic_ = topic;
    // Create publisher
    point_publisher_ =
        this->create_publisher<visualization_msgs::msg::Marker>(topic_, 10);
    // Publish edges in a timer callback
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&SimplePointPublisherNode::publishPoints, this));
}

void SimplePointPublisherNode::publishPoints()
{
    if (point_publisher_->get_subscription_count() > 0 ||
        point_publisher_->get_intra_process_subscription_count() > 0)
    {

        // Prepare Marker message
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map"; // Set to the appropriate frame
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "points";
        marker.id = 0; // You can increment this if you want to add multiple markers
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the point size (in RViz)
        marker.scale.x = 0.1; // Size of the points
        marker.scale.y = 0.1;
        marker.color.a = 1.0; // Fully opaque
        marker.color.r = 1.0; // Red color

        for (std::size_t i = 0; i < points_.size(); ++i)
        {
            auto point = points_[i];

            // Convert to geometry_msgs::msg::Point
            geometry_msgs::msg::Point ros_point1;
            ros_point1.x = bg::get<0>(point);
            ros_point1.y = bg::get<1>(point);
            ros_point1.z = 0.0; // Assum 2D points
            // Add to marker
            marker.points.push_back(ros_point1);
        }

        // Publish marker
        point_publisher_->publish(marker);
    }
}


