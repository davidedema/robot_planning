#include "graph_generator/graph_node.hpp"

GraphGenerator::GraphGenerator() : Node("GraphGenerator")
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
      "/map_borders", qos, std::bind(&GraphGenerator::callback_borders, this, std::placeholders::_1));
  subscription_obstacles_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
      "/obstacles", qos, std::bind(&GraphGenerator::callback_obstacles, this, std::placeholders::_1));
  subscription_gates_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/gates", qos, std::bind(&GraphGenerator::callback_gates, this, std::placeholders::_1));
  subscription_position1_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/shelfino1/amcl_pose", qos, std::bind(&GraphGenerator::callback_pos1, this, std::placeholders::_1));
  subscription_position2_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/shelfino2/amcl_pose", qos, std::bind(&GraphGenerator::callback_pos2, this, std::placeholders::_1));
  subscription_position3_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/shelfino3/amcl_pose", qos, std::bind(&GraphGenerator::callback_pos3, this, std::placeholders::_1));
}

GraphGenerator::~GraphGenerator() {}

void GraphGenerator::callback_borders(const geometry_msgs::msg::Polygon::SharedPtr msg)
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

void GraphGenerator::callback_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
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

void GraphGenerator::callback_gates(const geometry_msgs::msg::PoseArray::SharedPtr msg)
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

void GraphGenerator::callback_pos1(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  subscription_position1_.reset();
  pos1_r_ = true;
  this->set_pos1(*msg);
}

void GraphGenerator::callback_pos2(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  subscription_position2_.reset();
  pos2_r_ = true;
  this->set_pos2(*msg);
}

void GraphGenerator::callback_pos3(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  subscription_position3_.reset();
  pos3_r_ = true;
  this->set_pos3(*msg);
}

pose_t GraphGenerator::get_pose1()
{
  return pos1;
}

pose_t GraphGenerator::get_pose2()
{
  return pos2;
}

pose_t GraphGenerator::get_pose3()
{
  return pos3;
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
        aux_p.outer().insert(aux_p.outer().begin(), point_t(point.x, point.y));
      }
      obstacle_arr.push_back(aux_p);
      aux_p.clear();
    }
  }

  this->inflate_obstacles();
}

void GraphGenerator::inflate_obstacles()
{
  for (auto &obstacle : obstacle_arr)
  {
    if (!boost::geometry::equals(obstacle.outer().front(), obstacle.outer().back()))
    {
      obstacle.outer().push_back(obstacle.outer().front());
    }

    boost::geometry::model::multi_polygon<polygon_t> inflated_polygon;
    // Inflate the polygon using Boost's buffer algorithm
    boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(SHELFINO_INFLATION);
    boost::geometry::strategy::buffer::join_round join_strategy(36);     // Round joins
    boost::geometry::strategy::buffer::end_round end_strategy(36);       // Round ends
    boost::geometry::strategy::buffer::point_circle circle_strategy(36); // Circle approximation
    boost::geometry::strategy::buffer::side_straight side_strategy;      // Straight sides

    boost::geometry::buffer(obstacle, inflated_polygon,
                            distance_strategy,
                            side_strategy,
                            join_strategy,
                            end_strategy,
                            circle_strategy);

    // Add the inflated polygon to the result array
    inflated_obstacles.push_back(inflated_polygon);
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

  // Inflate the polygon using Boost's buffer algorithm
  boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(-SHELFINO_INFLATION);
  boost::geometry::strategy::buffer::join_round join_strategy(36);     // Round joins
  boost::geometry::strategy::buffer::end_round end_strategy(36);       // Round ends
  boost::geometry::strategy::buffer::point_circle circle_strategy(36); // Circle approximation
  boost::geometry::strategy::buffer::side_straight side_strategy;      // Straight sides

  boost::geometry::buffer(map_borders, inflated_borders,
                          distance_strategy,
                          side_strategy,
                          join_strategy,
                          end_strategy,
                          circle_strategy);
}

polygon_t GraphGenerator::get_borders()
{
  return map_borders;
}

void GraphGenerator::set_gate(const geometry_msgs::msg::PoseArray &msg)
{
  for (const auto &pose : msg.poses)
  {
    std::vector<double> gate;
    gate.push_back(pose.position.x);
    gate.push_back(pose.position.y);
    tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    double r, p, y;
    tf2::Matrix3x3 m(q);
    m.getRPY(r, p, y);
    gate.push_back(y);
    gates.push_back(gate);
  }
}

void GraphGenerator::set_pos1(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
{
  pos1.push_back(msg.pose.pose.position.x);
  pos1.push_back(msg.pose.pose.position.y);
  tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  double r, p, y;
  tf2::Matrix3x3 m(q);
  m.getRPY(r, p, y);
  pos1.push_back(y);
}

void GraphGenerator::set_pos2(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
{
  pos2.push_back(msg.pose.pose.position.x);
  pos2.push_back(msg.pose.pose.position.y);
  tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  double r, p, y;
  tf2::Matrix3x3 m(q);
  m.getRPY(r, p, y);
  pos2.push_back(y);
}

void GraphGenerator::set_pos3(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
{
  pos3.push_back(msg.pose.pose.position.x);
  pos3.push_back(msg.pose.pose.position.y);
  tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  double r, p, y;
  tf2::Matrix3x3 m(q);
  m.getRPY(r, p, y);
  pos3.push_back(y);
}

pose_t GraphGenerator::get_gate()
{
  return gates.at(0);
}

boost::geometry::model::multi_polygon<polygon_t> GraphGenerator::get_map()
{
  if (!is_map_created)
  {
    // Copy the outer borders from inflated_borders into map
    for (const auto &polygon : inflated_borders)
    {
      polygon_t new_polygon;

      // Copy the exterior ring of each polygon in inflated_borders to a new polygon
      for (const auto &point : boost::geometry::exterior_ring(polygon))
      {
        boost::geometry::append(new_polygon.outer(), point);
      }

      // Add the newly created polygon to the map
      map.push_back(new_polygon);
    }

    // Resize the inner rings to accommodate all polygons in inflated_obstacles
    size_t total_inner_rings = 0;
    for (const auto &multi_polygon : inflated_obstacles)
    {
      total_inner_rings += boost::geometry::num_geometries(multi_polygon); // Count polygons in each multi-polygon
    }
    map[0].inners().resize(total_inner_rings);

    // Populate the inner rings with the obstacles
    size_t i = 0; // Index for the inner rings
    for (const auto &multi_polygon : inflated_obstacles)
    {
      for (const auto &polygon : multi_polygon)
      {
        for (const auto &point : polygon.outer())
        {
          boost::geometry::append(map[0].inners()[i], point);
        }
        i++; // Move to the next inner ring
      }
    }

    is_map_created = true;
    return map;
  }
  else
  {
    return map;
  }
}