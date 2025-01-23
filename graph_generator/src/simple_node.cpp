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

    // Publisher for the Dubins curve
    dubins_curve_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "dubins_curve_marker", rclcpp::QoS(10));

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

  void setDubinsCurves(const std::vector<dubins_curve> &curves)
  {
    dubins_curves_ = curves;
  }

private:
  void checkAndPublishMarkers()
  {
    if (sampled_points_pub_->get_subscription_count() > 0 ||
        path_pub_->get_subscription_count() > 0 ||
        dubins_curve_pub_->get_subscription_count() > 0)
    {
      RCLCPP_INFO_ONCE(this->get_logger(), "Subscriber detected. Publishing markers.");
      publishMarkers();
    }
  }

  void publishMarkers()
  {
    publishSampledPointsMarker();
    publishPathMarker();
    publishDubinsCurveMarkers();
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
  void publishDubinsCurveMarkers()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map"; // Adjust frame_id as needed
    marker.header.stamp = this->now();
    marker.ns = "dubins_curve";
    marker.id = 3;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP; // Connected curves
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Define marker properties
    marker.scale.x = 0.05; // Curve line width
    marker.color.r = 0.0;
    marker.color.g = 1.0; // Green color
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Fill marker points for all Dubins curves
    for (const auto &curve : dubins_curves_)
    {
      auto add_arc_points = [&](const dubins_arc &arc)
      {
        double step = 0.1; // Step size for sampling points along the arc
        double theta = arc.th0;

        if (std::abs(arc.k) < 1e-6)
        {
          // Straight-line case
          for (double s = 0; s <= arc.L; s += step)
          {
            double x = arc.x0 + s * cos(theta);
            double y = arc.y0 + s * sin(theta);
            geometry_msgs::msg::Point p;
            p.x = x;
            p.y = y;
            p.z = 0.0;
            marker.points.push_back(p);
          }
        }
        else
        {
          // Curved case
          double radius = 1.0 / arc.k;
          for (double s = 0; s <= arc.L; s += step)
          {
            double x = arc.x0 + radius * (sin(theta + arc.k * s) - sin(theta));
            double y = arc.y0 + radius * (cos(theta) - cos(theta + arc.k * s));
            geometry_msgs::msg::Point p;
            p.x = x;
            p.y = y;
            p.z = 0.0;
            marker.points.push_back(p);
          }
        }
      };

      // Add points from all arcs in the Dubins curve
      add_arc_points(curve.a1);
      add_arc_points(curve.a2);
      add_arc_points(curve.a3);
    }

    // Publish the marker
    dubins_curve_pub_->publish(marker);
    RCLCPP_INFO(this->get_logger(), "Published Dubins curves with %zu points.", marker.points.size());
  }
  std::vector<std::vector<double>> sampled_points_;
  std::vector<std::vector<double>> path_points_;
  std::vector<struct dubins_curve> dubins_curves_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sampled_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr dubins_curve_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  int timer_interval_ms_;
};



int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);

  // Create the node
  auto node = std::make_shared<PointMarkerNode>();

  // test dubins

  // std::vector<double> start = {0, 0, 2.09};
  // std::vector<double> end = {4, 0, -1.04};
  // std::vector<std::vector<double>> points = {{1, 0}, {3, 0}};
  double kmax = 2;

  // Dubins d({0.0, 0.0}, {0.0, 0.0}, kmax);
  // auto dubins_curve = d.dubins_shortest_path(start.at(0), start.at(1), start.at(2), end.at(0), end.at(1), end.at(2), kmax);

  // // auto dubins_curves = d.dubins_multi_point(start.at(0), start.at(1), start.at(2), end.at(0), end.at(1), end.at(2), points, kmax);
  // cout << "finished" << endl;
  // for (const auto &curve : {dubins_curve})
  // {
  //   cout << "A1" << endl;
  //   cout << " x0 " << curve.a1.x0 << endl;
  //   cout << " y0 " << curve.a1.y0 << endl;
  //   cout << " th0 " << curve.a1.th0 << endl;
  //   cout << " k " << curve.a1.k << endl;
  //   cout << " L " << curve.a1.L << endl;
  //   cout << " xf " << curve.a1.xf << endl;
  //   cout << " yf " << curve.a1.yf << endl;
  //   cout << " thf " << curve.a1.thf << endl;

  //   cout << "A2" << endl;
  //   cout << " x0 " << curve.a2.x0 << endl;
  //   cout << " y0 " << curve.a2.y0 << endl;
  //   cout << " th0 " << curve.a2.th0 << endl;
  //   cout << " k " << curve.a2.k << endl;
  //   cout << " L " << curve.a2.L << endl;
  //   cout << " xf " << curve.a2.xf << endl;
  //   cout << " yf " << curve.a2.yf << endl;
  //   cout << " thf " << curve.a2.thf << endl;

  //   cout << "A3" << endl;
  //   cout << " x0 " << curve.a3.x0 << endl;
  //   cout << " y0 " << curve.a3.y0 << endl;
  //   cout << " th0 " << curve.a3.th0 << endl;
  //   cout << " k " << curve.a3.k << endl;
  //   cout << " L " << curve.a3.L << endl;
  //   cout << " xf " << curve.a3.xf << endl;
  //   cout << " yf " << curve.a3.yf << endl;
  //   cout << " thf " << curve.a3.thf << endl;
  //   cout << curve.L << endl;
  // }

  // std::vector<struct dubins_curve> a;
  // a.push_back(dubins_curve);

  // node->setDubinsCurves(a);

  // rclcpp::init(argc, argv);

  // // Create the node
  // auto node = std::make_shared<PointMarkerNode>();

  // Monitor execution time
  auto m = std::make_shared<GraphGenerator>();

  RCLCPP_INFO(m->get_logger(), "Waiting for obstacles, borders and gates...");
  while (!m->obstacles_r_ || !m->borders_r_ || !m->pos1_r_)
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
  KDNode_t start = {m->get_pose1().at(0), m->get_pose1().at(1)};
  KDNode_t goal = {m->get_gate().at(0), m->get_gate().at(1)};
  vector<KDNode_t> path;
  _rrt.set_problem(start, goal); // Dubins d({0.0, 0.0}, {0.0, 0.0}, kmax);

  for (uint i = 1; i < 3000; i++)
  {
    auto point = _rrt.get_random_point(i, map);
    auto nearest = _rrt.get_nn(point, 1);
    auto new_point = _rrt.next_point(point, nearest, map);
    auto best_one = _rrt.get_best_neighbor(new_point, nearest, 0.5, map);
    if (_rrt.add_edge(new_point, best_one, map))
    {
      sampled_points.push_back(new_point);
    }
    _rrt.rewire(new_point, 0.5, map);
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

  Dubins d;
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

  // rclcpp::sleep_for(std::chrono::milliseconds(1000)); // Sleep for 1 second
  node->setPathPoints(path_points2);
  // Output path
  // for (const auto &p : path)
  // {
  //   cout << p.at(0) << "  " << p.at(1) << endl;
  // }

  for (const auto &p : path_points2)
  {
    cout << p.at(0) << "  " << p.at(1) << endl;
  }
  cout << "  " << endl;
  path_points2.erase(path_points2.begin());
  path_points2.erase(path_points2.end());

  for (const auto &p : path_points2)
  {
    cout << p.at(0) << "  " << p.at(1) << endl;
  }

  auto dubins_curves = d.dubins_multi_point(start.at(0), start.at(1), m->get_pose1().at(2), goal.at(0), goal.at(1), m->get_gate().at(2), path_points2, kmax, map);
  node->setDubinsCurves(dubins_curves);

  // Keep the node alive
  rclcpp::spin(node);

  rclcpp::shutdown();
  cout << "Done!" << endl;
  return 0;
}
