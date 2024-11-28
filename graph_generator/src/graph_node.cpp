#include "graph_generator/graph_node.hpp"




GraphGenerator::GraphGenerator() : Node("GraphGenerator") {

  // create the QoS for the subscribers

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), qos_profile_custom);

  subsctiption_borders_ = this->create_subscription<geometry_msgs::msg::Polygon>(
      "/map_borders", qos, std::bind(&GraphGenerator::callback_borders, this, std::placeholders::_1));

  subsctiption_obstacles_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
      "/obstacles", qos, std::bind(&GraphGenerator::callback_obstacles, this, std::placeholders::_1));
  subsctiption_gates_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/gates_position", qos, std::bind(&GraphGenerator::callback_gates, this, std::placeholders::_1));
}

GraphGenerator::~GraphGenerator(){}

void GraphGenerator::callback_borders(const geometry_msgs::msg::Polygon::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "#################### BORDERS ##################");

  int size = msg->points.size();
  if (!msg->points.empty())
  {
    for (int i = 0; i < size; i++)
    {
      RCLCPP_INFO(this->get_logger(), "I heard: [%f, %f]", msg->points[i].x, msg->points[i].y);
    }
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Received an empty polygon message!");
  }
}

void GraphGenerator::callback_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "#################### OBSTACLES ##################");

  if (!msg->obstacles.empty())
  {
    RCLCPP_INFO(this->get_logger(), "Received %ld obstacles", msg->obstacles.size());

    for (const auto &obstacle : msg->obstacles)
    {

      RCLCPP_INFO(this->get_logger(), "Obstacle has %ld vertices", obstacle.polygon.points.size());
      for (const auto &point : obstacle.polygon.points)
      {
        RCLCPP_INFO(this->get_logger(), "Vertex: [%f, %f]", point.x, point.y);
      }
    }
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Received an empty obstacle array message!");
  }
}

void GraphGenerator::callback_gates(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "#################### GATES ##################");

  if (!msg->poses.empty())
  {
    RCLCPP_INFO(this->get_logger(), "Received %ld gates", msg->poses.size());

    for (const auto &pose : msg->poses)
    {

      RCLCPP_INFO(this->get_logger(), "Gate position: [%f, %f]", pose.position.x, pose.position.y);
      RCLCPP_INFO(this->get_logger(), "Orientation: [%f, %f, %f, %f]",
                  pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w);
    }
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Received an empty pose array message!");
  }
}