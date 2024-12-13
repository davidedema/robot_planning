#include <unistd.h>

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"

using FollowPath = nav2_msgs::action::FollowPath;
class PathPublisher : public rclcpp::Node {
 public:
  PathPublisher() : Node("Nav2Client") {
    client_ptr_ =
        rclcpp_action::create_client<FollowPath>(this, "shelfino1/follow_path");

    _path_subscription = this->create_subscription<nav_msgs::msg::Path>(
        "shelfino1/plan1", 10,
        std::bind(&PathPublisher::store_path, this, std::placeholders::_1));

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
    }
    std::cout << "Action client ready" << std::endl;
  }
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_subscription;
  geometry_msgs::msg::TransformStamped t;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription1;
  rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;
  // geometry_msgs::msg::PoseArray waypoints;
  nav_msgs::msg::Path full_path;
  // RobotPosition robot_pose;
  bool waypoints_received = false;

  void store_path(const nav_msgs::msg::Path& msg) {
    if (waypoints_received) {
      return;
    }
    full_path = msg;
    std::cout << "Path stored!" << std::endl;
    waypoints_received = true;
    return;
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathPublisher>();
  while (rclcpp::ok()) {
    if (node->waypoints_received) {
      std::cout << "\033[1;32mSending path to controller\033[0m" << std::endl;
      // nav_msgs::msg::Path full_path;
      node->full_path.header.stamp = node->get_clock()->now();
      node->full_path.header.frame_id = "map";

      auto goal_msg = FollowPath::Goal();
      goal_msg.path = node->full_path;
      for (size_t i = 0; i < goal_msg.path.poses.size(); i++) {
        std::cout << "Position: " << goal_msg.path.poses[i].pose.position.x
                  << ", " << goal_msg.path.poses[i].pose.position.y
                  << std::endl;
        std::cout << "Orientation: "
                  << goal_msg.path.poses[i].pose.orientation.x << ", "
                  << goal_msg.path.poses[i].pose.orientation.y << ", "
                  << goal_msg.path.poses[i].pose.orientation.z << ", "
                  << goal_msg.path.poses[i].pose.orientation.w << std::endl;
        std::cout << "---------------------" << std::endl;
      }
      goal_msg.controller_id = "FollowPath";
      node->client_ptr_->async_send_goal(goal_msg);
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      // ensure that the path is sent
      node->client_ptr_->async_send_goal(goal_msg);
      node->waypoints_received = false;
    }
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();

  return 0;
}