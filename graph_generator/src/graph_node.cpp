/**
 * @class GraphGenerator
 * @brief A class to generate and manage a graph representation of a map including borders, obstacles, gates, and positions.
 *
 * This class provides functionality to:
 * - Subscribe to various ROS topics related to map borders, obstacles, gates, and positions.
 * - Process incoming data to construct a graph-based map representation.
 * - Inflate obstacles and borders for safe navigation.
 * - Retrieve specific map elements like borders, obstacles, and positions.
 */

#include "graph_generator/graph_node.hpp"

/**
 * @brief Constructor for GraphGenerator.
 *
 * Initializes the ROS2 node, sets up subscriptions with QoS policies, and initializes member variables.
 */
GraphGenerator::GraphGenerator() : Node("GraphGenerator")
{
  // Initialize flags for data reception
  borders_r_ = false;
  obstacles_r_ = false;
  gates_r_ = false;
  pos1_r_ = false;
  pos2_r_ = false;
  pos3_r_ = false;

  // Create QoS policy
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), qos_profile_custom);

  // Create subscribers
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
}

/**
 * @brief Destructor for GraphGenerator.
 */
GraphGenerator::~GraphGenerator() {}

/**
 * @brief Callback for processing border messages.
 * @param msg Shared pointer to the received Polygon message.
 */
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

/**
 * @brief Callback for processing obstacle messages.
 * @param msg Shared pointer to the received ObstacleArrayMsg message.
 */
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

/**
 * @brief Callback for processing gate messages.
 * @param msg Shared pointer to the received PoseArray message.
 */
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

/**
 * @brief Callback for processing position1 messages.
 * @param msg Shared pointer to the received PoseWithCovarianceStamped message.
 */
void GraphGenerator::callback_pos1(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  subscription_position1_.reset();
  pos1_r_ = true;
  this->set_pos1(*msg);
}

/**
 * @brief Callback for processing position2 messages.
 * @param msg Shared pointer to the received PoseWithCovarianceStamped message.
 */
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

void GraphGenerator::callback_pos3(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  subscription_position3_.reset();
  pos3_r_ = true;
  this->set_pos3(*msg);
}

/**
 * @brief Get the position of the first robot.
 * @return A pose_t containing the position of the first robot.
 */
pose_t GraphGenerator::get_pose1()
{
  return pos1;
}

/**
 * @brief Get the position of the second robot.
 * @return A pose_t containing the position of the second robot.
 */
pose_t GraphGenerator::get_pose2()
{
  return pos2;
}

pose_t GraphGenerator::get_pose3()
{
  return pos3;
}

pose_t GraphGenerator::get_pose3()
{
  return pos3;
}

/**
 * @brief Process and store obstacle data.
 * @param msg A constant reference to an ObstacleArrayMsg message.
 */
void GraphGenerator::set_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg &msg)
{
  polygon_t aux_p;

  for (const auto &obstacle : msg.obstacles)
  {
    if (obstacle.radius)
    {
      // Create circular obstacles using buffer strategies
      int points_per_circle = 10;
      boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
      boost::geometry::strategy::buffer::end_flat end_strategy;
      boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
      boost::geometry::strategy::buffer::side_straight side_strategy;

      double radius = obstacle.radius;
      boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(radius);

      point_t center(obstacle.polygon.points[0].x, obstacle.polygon.points[0].y);
      boost::geometry::model::multi_polygon<polygon_t> tmp;

      boost::geometry::buffer(center, tmp, distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);

      obstacle_arr.push_back(tmp[0]);
    }
    else
    {
      // Process polygonal obstacles
      for (const auto &point : obstacle.polygon.points)
      {
        aux_p.outer().push_back(point_t(point.x, point.y));
      }
      obstacle_arr.push_back(aux_p);
      aux_p.clear();
    }
  }

  this->inflate_obstacles();
}

/**
 * @brief Inflate obstacles to account for robot dimensions.
 */
void GraphGenerator::inflate_obstacles()
{
  for (auto &obstacle : obstacle_arr)
  {
    if (!boost::geometry::equals(obstacle.outer().front(), obstacle.outer().back()))
    {
      obstacle.outer().push_back(obstacle.outer().front());
    }

    boost::geometry::model::multi_polygon<polygon_t> inflated_polygon;
    boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(SHELFINO_INFLATION);
    boost::geometry::strategy::buffer::join_round join_strategy(36);
    boost::geometry::strategy::buffer::end_round end_strategy(36);
    boost::geometry::strategy::buffer::point_circle circle_strategy(36);
    boost::geometry::strategy::buffer::side_straight side_strategy;

    boost::geometry::buffer(obstacle, inflated_polygon, distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);

    inflated_obstacles.push_back(inflated_polygon);
  }
}

/**
 * @brief Get the list of obstacles.
 * @return A vector of polygons representing the obstacles.
 */
std::vector<polygon_t> GraphGenerator::get_obstacles()
{
  return obstacle_arr;
}

/**
 * @brief Process and store map borders.
 * @param msg A constant reference to a Polygon message.
 */
void GraphGenerator::set_borders(const geometry_msgs::msg::Polygon &msg)
{
  for (const auto &point : msg.points)
  {
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

  // Inflate borders
  boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(-SHELFINO_INFLATION);
  boost::geometry::strategy::buffer::join_round join_strategy(36);
  boost::geometry::strategy::buffer::end_round end_strategy(36);
  boost::geometry::strategy::buffer::point_circle circle_strategy(36);
  boost::geometry::strategy::buffer::side_straight side_strategy;

  boost::geometry::buffer(map_borders, inflated_borders, distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);
}

/**
 * @brief Get the map borders.
 * @return A polygon representing the map borders.
 */
polygon_t GraphGenerator::get_borders()
{
  return map_borders;
}

/**
 * @brief Process and store gate data.
 * @param msg A constant reference to a PoseArray message.
 */
void GraphGenerator::set_gate(const geometry_msgs::msg::PoseArray &msg)
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

/**
 * @brief Process and store position1 data.
 * @param msg A constant reference to a PoseWithCovarianceStamped message.
 */
void GraphGenerator::set_pos1(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
{
  pos1 = {msg.pose.pose.position.x, msg.pose.pose.position.y};

  tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  double r, p, y;
  tf2::Matrix3x3 m(q);
  m.getRPY(r, p, y);
  pos1.push_back(y);
}

/**
 * @brief Process and store position2 data.
 * @param msg A constant reference to a PoseWithCovarianceStamped message.
 */
void GraphGenerator::set_pos2(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
{
  pos2 = {msg.pose.pose.position.x, msg.pose.pose.position.y};

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

/**
 * @brief Get the first gate's data.
 * @return A pose_t representing the first gate.
 */
pose_t GraphGenerator::get_gate()
{
  return gates.at(0);
}

/**
 * @brief Get the constructed map including inflated borders and obstacles.
 * @return A multi-polygon representing the map.
 */
boost::geometry::model::multi_polygon<polygon_t> GraphGenerator::get_map()
{
  if (!is_map_created)
  {
    for (const auto &polygon : inflated_borders)
    {
      polygon_t new_polygon;
      for (const auto &point : polygon.outer())
      {
        boost::geometry::append(new_polygon.outer(), point);
      }
      map.push_back(new_polygon);
    }

    size_t total_inner_rings = 0;
    for (const auto &multi_polygon : inflated_obstacles)
    {
      total_inner_rings += boost::geometry::num_geometries(multi_polygon);
    }
    map[0].inners().resize(total_inner_rings);

    size_t i = 0;
    for (const auto &multi_polygon : inflated_obstacles)
    {
      for (const auto &polygon : multi_polygon)
      {
        for (const auto &point : polygon.outer())
        {
          boost::geometry::append(map[0].inners()[i], point);
        }
        i++;
      }
    }

    is_map_created = true;
  }

  return map;
}
