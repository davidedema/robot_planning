#pragma once

#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

#define SHELFINO_INFLATION 0.5

typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_t;
typedef boost::geometry::model::polygon<point_t> polygon_t;
typedef std::vector<double> pose_t;

static const rmw_qos_profile_t qos_profile_custom = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    10,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};

class GraphGenerator : public rclcpp::Node
{
public:
  // constructor and distructor
  GraphGenerator();
  ~GraphGenerator();

  // callbacks for the subscribers
  void callback_borders(const geometry_msgs::msg::Polygon::SharedPtr msg);
  void callback_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg);
  void callback_gates(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  // flags for received msgs
  bool borders_r_;
  bool obstacles_r_;
  bool gates_r_;
  
  // flag for map creation
  bool is_map_created;

  // setter for map
  void set_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg &msg);
  void inflate_obstacles();
  void set_borders(const geometry_msgs::msg::Polygon &msg);
  void set_gate(const geometry_msgs::msg::PoseArray &msg);

  // getter for map
  std::vector<polygon_t> get_obstacles();
  polygon_t get_borders();
  pose_t get_gate();

  // get the map (polygon with holes replacing the obstacles)
  boost::geometry::model::multi_polygon<polygon_t> get_map();

private:
  // map values
  std::vector<polygon_t> obstacle_arr;
  std::vector<boost::geometry::model::multi_polygon<polygon_t>> inflated_obstacles;
  polygon_t map_borders;
  boost::geometry::model::multi_polygon<polygon_t> inflated_borders;
  pose_t gate;
  boost::geometry::model::multi_polygon<polygon_t> map;

  // subscriber
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subsctiption_borders_;
  rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subsctiption_obstacles_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subsctiption_gates_;
};