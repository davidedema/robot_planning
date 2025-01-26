#include "graph_generator/combinatorial_based/utilities.hpp"

#include "graph_generator/combinatorial_based/map_construction.hpp"

MapConstruction::MapConstruction() : Node("MapConstruction")
{

    // create the QoS for the subscribers

    borders_r_ = false;
    obstacles_r_ = false;
    gates_r_ = false;
    pos1_r_ = false;
    pos2_r_ = false;
    pos3_r_ = false;

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), qos_profile_custom);

    subscription_borders_ = this->create_subscription<geometry_msgs::msg::Polygon>(
        "/map_borders", qos, std::bind(&MapConstruction::callback_borders, this, std::placeholders::_1));
    subscription_obstacles_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
        "/obstacles", qos, std::bind(&MapConstruction::callback_obstacles, this, std::placeholders::_1));
    subscription_gates_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/gates", qos, std::bind(&MapConstruction::callback_gates, this, std::placeholders::_1));
    subscription_position1_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/shelfino1/amcl_pose", qos, std::bind(&MapConstruction::callback_pos1, this, std::placeholders::_1));
    subscription_position2_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/shelfino2/amcl_pose", qos, std::bind(&MapConstruction::callback_pos2, this, std::placeholders::_1));
    subscription_position3_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/shelfino3/amcl_pose", qos, std::bind(&MapConstruction::callback_pos3, this, std::placeholders::_1));
}

MapConstruction::~MapConstruction() {}

void MapConstruction::callback_borders(const geometry_msgs::msg::Polygon::SharedPtr msg)
{
    subscription_borders_.reset();
    if (!msg->points.empty())
    {
        this->set_borders(*msg);
        borders_r_ = true;
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Received an empty polygon message!");
    }
}

void MapConstruction::callback_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
{
    subscription_obstacles_.reset();
    if (!msg->obstacles.empty())
    {
        this->set_obstacles(*msg);
        obstacles_r_ = true;
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Received an empty obstacle array message!");
    }
}

void MapConstruction::callback_gates(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    subscription_gates_.reset();
    if (!msg->poses.empty())
    {
        gates_r_ = true;
        this->set_gate(*msg);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Received an empty pose array message!");
    }
}

void MapConstruction::callback_pos1(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    subscription_position1_.reset();
    pos1_r_ = true;
    this->set_pos1(*msg);
}

void MapConstruction::callback_pos2(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    subscription_position2_.reset();
    pos2_r_ = true;
    this->set_pos2(*msg);
}

void MapConstruction::callback_pos3(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    subscription_position3_.reset();
    pos3_r_ = true;
    this->set_pos3(*msg);
}

pose_t MapConstruction::get_pose1()
{
    return pos1;
}

pose_t MapConstruction::get_pose2()
{
    return pos2;
}

pose_t MapConstruction::get_pose3()
{
    return pos3;
}

void MapConstruction::set_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg &msg)
{

    polygon_t aux_p; // aux polygon

    for (const auto &obstacle : msg.obstacles)
    {
        if (obstacle.radius)
        {

            // declare all buffer strategies
            int points_per_circle(10);
            boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
            boost::geometry::strategy::buffer::end_flat end_strategy;
            boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
            boost::geometry::strategy::buffer::side_straight side_strategy;

            // set the distance strategy
            double radius = obstacle.radius;
            boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(radius);

            point_t center = point_t(obstacle.polygon.points[0].x, obstacle.polygon.points[0].y);
            boost::geometry::model::multi_polygon<polygon_t> tmp; // buffer generates multipolygons
            polygon_t disk;

            // make disk centered on `center` and of correct `radius`
            boost::geometry::buffer(center, tmp, distance_strategy, side_strategy,
                                    join_strategy, end_strategy, circle_strategy);

            // convert the MultiPolygon output to a simple polygon
            disk = polygon_t(tmp[0]);

            obstacle_arr.push_back(disk);
        }
        else
        { // the obstacle is a polygon
            for (const auto &point : obstacle.polygon.points)
            {
                aux_p.outer().insert(aux_p.outer().begin(), point_t(point.x, point.y));
            }
            obstacle_arr.push_back(aux_p);
            aux_p.clear();
        }
    }

    // this->inflate_obstacles();
}

void MapConstruction::inflate_obstacles()
{
}

std::vector<polygon_t> MapConstruction::get_obstacles()
{
    return obstacle_arr;
}

void MapConstruction::set_borders(const geometry_msgs::msg::Polygon &msg)
{
    for (auto const &point : msg.points)
    {
        // this->map_borders.outer().insert(this->map_borders.outer().begin(), point_t(point.x, point.y));
        if (msg.points.size() > 4)
        {
            map_borders.outer().push_back(point_t(point.x, point.y));
        }
        else
        {
            map_borders.outer().insert(this->map_borders.outer().begin(), point_t(point.x, point.y));
        }
    }

    map_borders.outer().push_back(map_borders.outer().front());

}

polygon_t MapConstruction::get_borders()
{
    return map_borders;
}

void MapConstruction::set_gate(const geometry_msgs::msg::PoseArray &msg)
{
    for (const auto &pose : msg.poses)
    {
        std::vector<double> gate = {pose.position.x, pose.position.y};

        tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        double r, p, y;
        tf2::Matrix3x3 m(q);
        m.getRPY(r, p, y);
        gate.push_back(y);

        gates.push_back(gate);
    }
}

void MapConstruction::set_pos1(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
{
    pos1.push_back(msg.pose.pose.position.x);
    pos1.push_back(msg.pose.pose.position.y);
    tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    double r, p, y;
    tf2::Matrix3x3 m(q);
    m.getRPY(r, p, y);
    pos1.push_back(y);
}

void MapConstruction::set_pos2(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
{
    pos2.push_back(msg.pose.pose.position.x);
    pos2.push_back(msg.pose.pose.position.y);
    tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    double r, p, y;
    tf2::Matrix3x3 m(q);
    m.getRPY(r, p, y);
    pos2.push_back(y);
}

void MapConstruction::set_pos3(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
{
    pos3.push_back(msg.pose.pose.position.x);
    pos3.push_back(msg.pose.pose.position.y);
    tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    double r, p, y;
    tf2::Matrix3x3 m(q);
    m.getRPY(r, p, y);
    pos3.push_back(y);
}

pose_t MapConstruction::get_gate()
{
    return gates.at(0);
}

// Scale factor for Clipper (to convert floating-point to integers)

// Convert Boost.Geometry polygon to Clipper polygon
ClipperLib::Paths MapConstruction::boostToClipper(const polygon_t &boost_polygon)
{
    ClipperLib::Paths clipper_paths;
    ClipperLib::Path clipper_path;

    // Convert exterior ring
    for (const auto &point : bg::exterior_ring(boost_polygon))
    {
        clipper_path.emplace_back(
            static_cast<ClipperLib::cInt>(point.get<0>() * CLIPPER_SCALE_FACTOR),
            static_cast<ClipperLib::cInt>(point.get<1>() * CLIPPER_SCALE_FACTOR));
    }
    clipper_paths.push_back(clipper_path);

    // Convert interior rings
    for (const auto &ring : bg::interior_rings(boost_polygon))
    {
        clipper_path.clear();
        for (const auto &point : ring)
        {
            clipper_path.emplace_back(
                static_cast<ClipperLib::cInt>(point.get<0>() * CLIPPER_SCALE_FACTOR),
                static_cast<ClipperLib::cInt>(point.get<1>() * CLIPPER_SCALE_FACTOR));
        }
        clipper_paths.push_back(clipper_path);
    }

    return clipper_paths;
}

// Convert Clipper polygon back to Boost.Geometry polygon
polygon_t MapConstruction::clipperToBoost(const ClipperLib::Path &clipper_paths)
{
    polygon_t boost_polygon;

    for (const auto &point : clipper_paths)
    {
        bg::append(
            bg::exterior_ring(boost_polygon),
            point_t(point.X / CLIPPER_SCALE_FACTOR, point.Y / CLIPPER_SCALE_FACTOR));
    }
    bg::correct(boost_polygon); // Ensure the polygon is valid

    return boost_polygon;
}

multi_polygon_t MapConstruction::get_clean_map()
{
    if (!is_clean_map_created)
    {
        // clean_map.emplace_back(map_borders);

        for (auto polygon : obstacle_arr)
        {
            clean_map.emplace_back(polygon);
        }

        is_clean_map_created = true;
        return clean_map;
    }
    else
    {
        return clean_map;
    }
}

bool doPolygonsIntersect(const ClipperLib::Path &poly1, const ClipperLib::Path &poly2)
{
    // Step 1: Initialize Clipper
    ClipperLib::Clipper clipper;

    // Step 2: Add the polygons
    clipper.AddPath(poly1, ClipperLib::ptSubject, true); // "true" for closed polygons
    clipper.AddPath(poly2, ClipperLib::ptClip, true);

    // Step 3: Check for intersection
    ClipperLib::Paths intersectionResult;
    clipper.Execute(ClipperLib::ctIntersection, intersectionResult);

    // Step 4: Return true if intersection exists
    return !intersectionResult.empty();
}

multi_polygon_t MapConstruction::get_inflated_map()
{
    if (!is_inflated_map_created)
    {
        ClipperLib::Paths clipper_poly1 = this->boostToClipper(map_borders);
        ClipperLib::Paths clipper_result;
        ClipperLib::Clipper clipper_border;
        ClipperLib::ClipperOffset clipperOffset;
        clipperOffset.AddPaths(clipper_poly1, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
        ClipperLib::Paths paths_inflated_border;
        clipperOffset.Execute(paths_inflated_border, -SHELFINO_INFLATION * 1000);
        inflated_borders = (this->clipperToBoost(paths_inflated_border[0]));

        // inflate obstacles
        ClipperLib::Paths inflated_vect_paths;
        for (auto polygon : obstacle_arr)
        {
            ClipperLib::Paths inflated_path_obstacles;
            ClipperLib::Paths clipper_result;
            ClipperLib::Clipper clipper_inflate_obstacles;
            ClipperLib::Paths clipper_poly1 = this->boostToClipper(polygon);
            ClipperLib::ClipperOffset clipperOffset;
            clipperOffset.AddPaths(clipper_poly1, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
            clipperOffset.Execute(inflated_path_obstacles, SHELFINO_INFLATION * 1000);
            inflated_vect_paths.emplace_back(inflated_path_obstacles[0]);
        }

        std::vector<polygon_t> inflated_polygons;
        for (auto const &clip_poly : inflated_vect_paths)
        {
            inflated_polygons.emplace_back(this->clipperToBoost(clip_poly));
        }

        std::vector<polygon_t> new_inflated_polygons;

        for (auto const &poly : inflated_polygons)
        {
            std::vector<polygon_t> output;
            bg::intersection(inflated_borders, poly, output);
            new_inflated_polygons.emplace_back(output[0]);
        }
        inflated_polygons = new_inflated_polygons;

        for (auto const &poly : inflated_polygons)
        {
            inflated_obstacles.emplace_back(poly);
        }

        is_inflated_map_created = true;
        return inflated_obstacles;
    }
    else
    {
        return inflated_obstacles;
    }
}

multi_polygon_t MapConstruction::get_unionized_inflated_map()
{
    // union between polygons
    size_t intersection_detected = true;
    multi_polygon_t inflated_polygons = inflated_obstacles;
    while (intersection_detected)
    {
        multi_polygon_t new_polygons;
        intersection_detected = false;
        std::vector<size_t> index_to_exclude;
        // union of 2 polygons
        for (size_t i = 0; i < inflated_polygons.size(); i++)
        {
            bool intersected_polygon = false;
            if (count(index_to_exclude.begin(), index_to_exclude.end(), i) == 0)
            {
                for (size_t z = 0; z < inflated_polygons.size(); z++)
                {

                    if (i != z && count(index_to_exclude.begin(), index_to_exclude.end(), z) == 0)
                    {

                        std::vector<polygon_t> output_poly;
                        if (bg::intersects(inflated_polygons[i], inflated_polygons[z]))
                        {
                            bg::union_(inflated_polygons[i], inflated_polygons[z], output_poly);
                            new_polygons.insert(new_polygons.end(), output_poly.begin(), output_poly.end());

                            index_to_exclude.emplace_back(i);
                            index_to_exclude.emplace_back(z);
                            intersection_detected = true;
                            intersected_polygon = true;
                        }
                    }
                }
                if (!intersected_polygon)
                {
                    new_polygons.emplace_back(inflated_polygons[i]);
                }
            }
        }
        inflated_polygons = new_polygons;
    }
    return inflated_polygons;
}

polygon_t MapConstruction::get_inflated_border()
{
    return inflated_borders;
}