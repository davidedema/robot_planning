#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

static const rmw_qos_profile_t qos_profile_custom = {
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  10,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};

class GraphGenerator : public rclcpp::Node {
  public:
    // constructor and distructor
    GraphGenerator();
    ~GraphGenerator();

    
  private:
    // callbacks for the subscribers
    void callback_borders(const geometry_msgs::msg::Polygon::SharedPtr msg);
    void callback_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg);
    void callback_gates(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    // subscriber
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subsctiption_borders_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subsctiption_obstacles_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subsctiption_gates_;

};