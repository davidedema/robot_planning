#include "graph_generator/graph_node.hpp"

GraphGenerator::GraphGenerator() : Node("GraphGenerator")
{

  // create the QoS for the subscribers

  borders_r_ = false;
  obstacles_r_ = false;
  gates_r_ = false;

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), qos_profile_custom);

  subsctiption_borders_ = this->create_subscription<geometry_msgs::msg::Polygon>(
      "/map_borders", qos, std::bind(&GraphGenerator::callback_borders, this, std::placeholders::_1));

  subsctiption_obstacles_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
      "/obstacles", qos, std::bind(&GraphGenerator::callback_obstacles, this, std::placeholders::_1));
  subsctiption_gates_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/gates_position", qos, std::bind(&GraphGenerator::callback_gates, this, std::placeholders::_1));
}

GraphGenerator::~GraphGenerator() {}

void GraphGenerator::callback_borders(const geometry_msgs::msg::Polygon::SharedPtr msg)
{
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

void GraphGenerator::callback_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
{
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

void GraphGenerator::callback_gates(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
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

void GraphGenerator::set_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg &msg)
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
        boost::geometry::append(aux_p.outer(), point_t(point.x, point.y));
      }
      obstacle_arr.push_back(aux_p);
      aux_p.clear();
    }
  }
}

std::vector<polygon_t> GraphGenerator::get_obstacles()
{
  return obstacle_arr;
}

void GraphGenerator::set_borders(const geometry_msgs::msg::Polygon &msg)
{
  for (auto const &point : msg.points)
  {
    boost::geometry::append(this->map_borders.outer(), point_t(point.x, point.y));
  }
}

polygon_t GraphGenerator::get_borders()
{
  return map_borders;
}

void GraphGenerator::set_gate(const geometry_msgs::msg::PoseArray &msg)
{
  for (const auto &pose : msg.poses)
  {
    gate.push_back(pose.position.x);
    gate.push_back(pose.position.y);
    gate.push_back(pose.orientation.x);
    gate.push_back(pose.orientation.y);
    gate.push_back(pose.orientation.z);
    gate.push_back(pose.orientation.w);
  }
}

pose_t GraphGenerator::get_gate()
{
  return gate;
}

polygon_t GraphGenerator::get_map()
{
  if (!is_map_created)
  {
    for (auto it = boost::begin(boost::geometry::exterior_ring(map_borders)); it != boost::end(boost::geometry::exterior_ring(map_borders)); ++it)
    {
      boost::geometry::append(map.outer(), *it);
    }
    map.inners().resize(5);
    int i = 0;
    // iterate through all obstacles
    for (const auto &obstacle : obstacle_arr)
    {
      for (const auto &point : obstacle.outer())
      {
        boost::geometry::append(map.inners()[i], point);
      }
      i++;
    }
    is_map_created = true;
    return map;
  }
  else
  {
    return map;
  }
}