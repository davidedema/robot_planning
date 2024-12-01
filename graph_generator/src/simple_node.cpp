#include <time.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include "graph_generator/graph_node.hpp"
#include "graph_generator/sampling_based/utils/rrt.hpp"
#include "graph_generator/sampling_based/utils/kdtree.hpp"

using namespace std;

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>

class PointMarkerNode : public rclcpp::Node
{
public:
  PointMarkerNode()
      : Node("point_marker_node"),
        timer_interval_ms_(500)
  { // Check for subscribers every 500 ms
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "visualization_marker", rclcpp::QoS(10));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_interval_ms_),
        std::bind(&PointMarkerNode::checkAndPublishMarker, this));

    RCLCPP_INFO(this->get_logger(), "Point Marker Node started");
  }

  void setPoints(const std::vector<std::vector<double>> &points)
  {
    std::vector<double> flat_points;

    // Flatten the points
    for (const auto &point : points)
    {
      if (point.size() == 2)
      {                                  // Ensure valid 2D point
        flat_points.push_back(point[0]); // x
        flat_points.push_back(point[1]); // y
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Skipping invalid point with size %zu", point.size());
      }
    }

    // Use the flattened points
    points_ = flat_points;
  }

private:
  void checkAndPublishMarker()
  {
    if (marker_pub_->get_subscription_count() > 0)
    {
      RCLCPP_INFO_ONCE(this->get_logger(),
                       "Subscriber detected. Publishing marker.");
      publishMarker();
    }
  }

  void publishMarker()
  {
    if (points_.empty())
    {
      RCLCPP_WARN(this->get_logger(), "No points to publish.");
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map"; // Adjust frame_id as needed
    marker.header.stamp = this->now();
    marker.ns = "points";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Define marker properties
    marker.scale.x = 0.1; // Sphere size
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // Fill marker points
    for (size_t i = 0; i < points_.size(); i += 2)
    {
      geometry_msgs::msg::Point point;
      point.x = points_[i];
      point.y = points_[i + 1];
      point.z = 0.0; // z is set to 0 for 2D points
      marker.points.push_back(point);
    }

    // Publish the marker
    marker_pub_->publish(marker);
    RCLCPP_INFO(this->get_logger(), "Published marker with %zu points",
                marker.points.size());
  }

  std::vector<double> points_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int timer_interval_ms_;
};

void print_nodes(const Kdtree::KdNodeVector &nodes)
{
  size_t i, j;
  for (i = 0; i < nodes.size(); ++i)
  {
    if (i > 0)
      cout << " ";
    cout << "(";
    for (j = 0; j < nodes[i].point.size(); j++)
    {
      if (j > 0)
        cout << ",";
      cout << nodes[i].point[j];
    }
    cout << ")";
  }
  cout << endl;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // Create the node
  auto node = std::make_shared<PointMarkerNode>();
  // Monitor execution time
  auto m = std::make_shared<GraphGenerator>();

  RCLCPP_INFO(m->get_logger(), "Waiting for obstacles, borders and gates...");
  while (!m->obstacles_r_ || !m->borders_r_)
  {
    rclcpp::spin_some(m->get_node_base_interface());
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(m->get_logger(), "\033[1;32m Map info obtained\033[0m");
  RCLCPP_INFO(m->get_logger(), "Building map");
  auto map = m->get_map();
  RCLCPP_INFO(m->get_logger(), "\033[1;32m Map builded\033[0m");
  RCLCPP_INFO(m->get_logger(), "Sampling test");

  RRT _rrt;

  std::vector<std::vector<double>> points;

  // start sampling nodes with rrt
  auto point_0 = _rrt.get_random_point(1, map);
  if (_rrt.add_node(point_0))
  {
    cout << "Added first node " << point_0.at(0) << point_0.at(1) << " , starting RRT..." << endl;
  }
  for (uint i = 2; i < 1500; i++)
  {
    std::cout << "----------------" << std::endl;
    auto point = _rrt.get_random_point(i, map);
    std::cout << point.at(0) << " " << point.at(1) << std::endl;
    auto nearest = _rrt.get_nn(point, 1);
    std::cout << nearest.at(0) << " " << nearest.at(1) << std::endl;
    auto new_point = _rrt.next_point(point, nearest, map);
    std::cout << new_point.at(0) << " " << new_point.at(1) << std::endl;
    if (_rrt.add_edge(new_point, nearest, map))
    {
      cout << "Added new node" << endl;
      points.push_back(new_point);
    }
  }

  // Pass points to the node
  node->setPoints(points);

  // Keep the node alive
  rclcpp::spin(node);

  rclcpp::shutdown();
  cout << "Done!" << endl;
  return 0;
}