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

#include "graph_generator/utils/dubins.hpp"

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>

using namespace std;

class PointMarkerNode : public rclcpp::Node
{
public:
  PointMarkerNode()
      : Node("point_marker_node"), timer_interval_ms_(500)
  {
    // Publisher for sampled points
    sampled_points_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "sampled_points_marker", rclcpp::QoS(10));

    // Publisher for the path
    path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "path_marker", rclcpp::QoS(10));

    // Timer to periodically check for subscribers and publish
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_interval_ms_),
        std::bind(&PointMarkerNode::checkAndPublishMarkers, this));

    RCLCPP_INFO(this->get_logger(), "Point Marker Node started");
  }

  void setSampledPoints(const std::vector<std::vector<double>> &sampled_points)
  {
    sampled_points_ = sampled_points;
  }

  void setPathPoints(const std::vector<std::vector<double>> &path_points)
  {
    path_points_ = path_points;
  }

private:
  void checkAndPublishMarkers()
  {
    if (sampled_points_pub_->get_subscription_count() > 0 ||
        path_pub_->get_subscription_count() > 0)
    {
      RCLCPP_INFO_ONCE(this->get_logger(), "Subscriber detected. Publishing markers.");
      publishMarkers();
    }
  }

  void publishMarkers()
  {
    publishSampledPointsMarker();
    publishPathMarker();
  }

  void publishSampledPointsMarker()
  {
    if (sampled_points_.empty())
    {
      RCLCPP_WARN(this->get_logger(), "No sampled points to publish.");
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map"; // Adjust frame_id as needed
    marker.header.stamp = this->now();
    marker.ns = "sampled_points";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST; // Individual points
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Define marker properties
    marker.scale.x = 0.05; // Point size
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0; // Blue color
    marker.color.a = 1.0;

    // Fill marker points
    for (const auto &point : sampled_points_)
    {
      if (point.size() == 2)
      {
        geometry_msgs::msg::Point p;
        p.x = point[0];
        p.y = point[1];
        p.z = 0.0;
        marker.points.push_back(p);
      }
    }

    // Publish the marker
    sampled_points_pub_->publish(marker);
    RCLCPP_INFO(this->get_logger(), "Published %zu sampled points.", marker.points.size());
  }

  void publishPathMarker()
  {
    if (path_points_.empty())
    {
      RCLCPP_WARN(this->get_logger(), "No path points to publish.");
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map"; // Adjust frame_id as needed
    marker.header.stamp = this->now();
    marker.ns = "path";
    marker.id = 2;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP; // Connected path
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Define marker properties
    marker.scale.x = 0.05; // Path width
    marker.color.r = 1.0;  // Red color
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Fill marker points
    for (const auto &point : path_points_)
    {
      if (point.size() == 2)
      {
        geometry_msgs::msg::Point p;
        p.x = point[0];
        p.y = point[1];
        p.z = 0.0;
        marker.points.push_back(p);
      }
    }

    // Publish the marker
    path_pub_->publish(marker);
    RCLCPP_INFO(this->get_logger(), "Published path with %zu points.", marker.points.size());
  }

  std::vector<std::vector<double>> sampled_points_;
  std::vector<std::vector<double>> path_points_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sampled_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int timer_interval_ms_;
};

int main(int argc, char **argv)
{

  // test dubins

  // std::vector<double> start = {0, 0, -2.09};
  // std::vector<double> end = {4, 0, 1.04};
  // double kmax = 2;

  // Dubins d(start, end, kmax);

  // auto dubins_curve = d.dubins_shortest_path(start.at(0), start.at(1), start.at(2), end.at(0), end.at(1), end.at(2), kmax);
  // cout << "A1" << endl;
  // cout << " x0 " << dubins_curve.a1.x0 << endl;
  // cout << " y0 " << dubins_curve.a1.y0 << endl;
  // cout << " th0 " << dubins_curve.a1.th0 << endl;
  // cout << " k " << dubins_curve.a1.k << endl;
  // cout << " L " << dubins_curve.a1.L << endl;
  // cout << " xf " << dubins_curve.a1.xf << endl;
  // cout << " yf " << dubins_curve.a1.yf << endl;
  // cout << " thf " << dubins_curve.a1.thf << endl;

  // cout << "A2" << endl;
  // cout << " x0 " << dubins_curve.a2.x0 << endl;
  // cout << " y0 " << dubins_curve.a2.y0 << endl;
  // cout << " th0 " << dubins_curve.a2.th0 << endl;
  // cout << " k " << dubins_curve.a2.k << endl;
  // cout << " L " << dubins_curve.a2.L << endl;
  // cout << " xf " << dubins_curve.a2.xf << endl;
  // cout << " yf " << dubins_curve.a2.yf << endl;
  // cout << " thf " << dubins_curve.a2.thf << endl;

  // cout << "A3" << endl;
  // cout << " x0 " << dubins_curve.a3.x0 << endl;
  // cout << " y0 " << dubins_curve.a3.y0 << endl;
  // cout << " th0 " << dubins_curve.a3.th0 << endl;
  // cout << " k " << dubins_curve.a3.k << endl;
  // cout << " L " << dubins_curve.a3.L << endl;
  // cout << " xf " << dubins_curve.a3.xf << endl;
  // cout << " yf " << dubins_curve.a3.yf << endl;
  // cout << " thf " << dubins_curve.a3.thf << endl;

  // cout << dubins_curve.L << endl;

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
  RCLCPP_INFO(m->get_logger(), "\033[1;32m Map built\033[0m");
  RCLCPP_INFO(m->get_logger(), "Sampling test");

  RRT _rrt;

  std::vector<std::vector<double>> sampled_points;
  std::vector<std::vector<double>> path_points;
  std::vector<std::vector<double>> path_points2;

  // Set start and goal
  KDNode_t start = {5, 5};
  KDNode_t goal = {-4, 2.6};
  vector<KDNode_t> path;

  _rrt.set_problem(start, goal);
  for (uint i = 1; i < 3000; i++)
  {
    auto point = _rrt.get_random_point(i, map);
    auto nearest = _rrt.get_nn(point, 1);
    auto new_point = _rrt.next_point(point, nearest, map);
    auto best_one = _rrt.get_best_neighbor(new_point, nearest, 2, map);
    if (_rrt.add_edge(new_point, best_one, map))
    {
      sampled_points.push_back(new_point);
    }
    _rrt.rewire(new_point, 2, map);
    if (_rrt.is_goal(new_point))
    {
      path = _rrt.get_path(new_point);
      // break;
    }
  }

  // Convert path to 2D points
  for (const auto &p : path)
  {
    path_points.push_back({p.at(0), p.at(1)});
  }

  // Set points in the node
  node->setSampledPoints(sampled_points);
  node->setPathPoints(path_points);

  // now find new smoothed path
  auto new_path = _rrt.smooth_path(path, map);

  // Convert path to 2D points
  for (const auto &p : new_path)
  {
    cout << p.at(0) << "  " << p.at(1) << endl;
    path_points2.push_back({p.at(0), p.at(1)});
  }
  node->setPathPoints(path_points2);
  // Output path
  // for (const auto &p : path)
  // {
  //   cout << p.at(0) << "  " << p.at(1) << endl;
  // }

  // Keep the node alive
  rclcpp::spin(node);

  rclcpp::shutdown();
  cout << "Done!" << endl;
  return 0;
}
