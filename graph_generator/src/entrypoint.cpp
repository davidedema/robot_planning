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
#include "graph_generator/utils/orchestrator.hpp"

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>

#define DISPLAY_SAMPLES 0
#define DISPLAY_PATH_1 0
#define DISPLAY_PATH_2 0

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

    shelfino1_nav2_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "shelfino1/plan1", rclcpp::QoS(10));
    shelfino2_nav2_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "shelfino2/plan1", rclcpp::QoS(10));
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
  void send_nav2(const nav_msgs::msg::Path &path1, const nav_msgs::msg::Path &path2)
  {
    shelfino1_nav2_path_pub_->publish(path1);
    shelfino2_nav2_path_pub_->publish(path2);
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
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr shelfino1_nav2_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr shelfino2_nav2_path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int timer_interval_ms_;
};

nav_msgs::msg::Path convertDubinsPathToNavPath(const std::vector<dubins_curve> &dubins_curves)
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "map"; // Adjust the frame_id to your TF setup
  path_msg.header.stamp = rclcpp::Clock().now();

  for (const auto &curve : dubins_curves)
  {
    auto add_arc_points_to_path = [&](const dubins_arc &arc)
    {
      double step = 0.05; // Step size for sampling points along the arc
      double theta = arc.th0;

      if (std::abs(arc.k) < 1e-6)
      {
        // Straight-line case
        for (double s = 0; s <= arc.L; s += step)
        {
          geometry_msgs::msg::PoseStamped pose;
          pose.header = path_msg.header;
          pose.pose.position.x = arc.x0 + s * cos(theta);
          pose.pose.position.y = arc.y0 + s * sin(theta);
          pose.pose.position.z = 0.0;
          tf2::Quaternion quat(0, 0, sin(theta / 2), cos(theta / 2));
          pose.pose.orientation.x = quat.x();
          pose.pose.orientation.y = quat.y();
          pose.pose.orientation.z = quat.z();
          pose.pose.orientation.w = quat.w();

          path_msg.poses.push_back(pose);
        }
      }
      else
      {
        // Curved case
        double radius = 1.0 / arc.k;
        for (double s = 0; s <= arc.L; s += step)
        {
          geometry_msgs::msg::PoseStamped pose;
          pose.header = path_msg.header;
          pose.pose.position.x = arc.x0 + radius * (sin(theta + arc.k * s) - sin(theta));
          pose.pose.position.y = arc.y0 + radius * (cos(theta) - cos(theta + arc.k * s));
          pose.pose.position.z = 0.0;
          tf2::Quaternion quat(0, 0, sin(theta / 2), cos(theta / 2));
          pose.pose.orientation.x = quat.x();
          pose.pose.orientation.y = quat.y();
          pose.pose.orientation.z = quat.z();
          pose.pose.orientation.w = quat.w();

          path_msg.poses.push_back(pose);
        }
      }
    };

    // Add points from all arcs in the Dubins curve
    add_arc_points_to_path(curve.a1);
    add_arc_points_to_path(curve.a2);
    add_arc_points_to_path(curve.a3);
  }

  return path_msg;
}

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);

  // Create the node
  auto node = std::make_shared<PointMarkerNode>();

  double kmax = 2;

  // Monitor execution time
  auto m = std::make_shared<GraphGenerator>();

  RCLCPP_INFO(m->get_logger(), "Waiting for obstacles, borders and gates...");
  while (!m->obstacles_r_ || !m->borders_r_ || !m->gates_r_ || !m->pos1_r_ || !m->pos2_r_)
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

  // Pose for shelfino1
  KDNode_t start_shelfino1 = {m->get_pose1().at(0), m->get_pose1().at(1)};
  KDNode_t goal = {m->get_gate().at(0), m->get_gate().at(1)};
  vector<KDNode_t> path;
  _rrt.set_problem(start_shelfino1, goal);

  // Main tree -> created from shelfino1

  for (uint i = 1; i < 3000; i++)
  {

    auto point = _rrt.get_random_point(i, map);
    auto nearest = _rrt.get_nn(point, 1);
    auto new_point = _rrt.next_point(point, nearest, map);
    auto best_one = _rrt.get_best_neighbor(new_point, nearest, 0.3, map);
    if (_rrt.add_edge(new_point, best_one, map))
    {
      sampled_points.push_back(new_point);
    }
    _rrt.rewire(new_point, 0.3, map);
    if (_rrt.is_goal(new_point))
    {
      path = _rrt.get_path(new_point);
      break;
    }
  }

  RCLCPP_INFO(m->get_logger(), "\033[1;32m Founded first path \033[0m");

  if (DISPLAY_SAMPLES)
  {
    node->setSampledPoints(sampled_points);
  }

  // Pose for shelfino 2
  KDNode_t start_shelfino2 = {m->get_pose2().at(0), m->get_pose2().at(1)};
  
  RCLCPP_INFO(m->get_logger(), "Retrived position for shelfino 2");

  // Add shelfino 2 start_shelfino1 node to the graph
  auto nearest2 = _rrt.get_nn(start_shelfino2, 1);
  RCLCPP_INFO(m->get_logger(), "Get nn for shelfino 2");
  if (_rrt.attach_node(start_shelfino2, nearest2, map))
  {
    RCLCPP_INFO(m->get_logger(), "\033[1;32m Added start shelfino 2\033[0m");
  }

  RCLCPP_INFO(m->get_logger(), "\033[1;32m Starting finding the joint plan...\033[0m");
  //* Multi agent path finding
  Orchestrator _orchestrator_shelfino1(_rrt.get_graph(), _rrt.get_lookup());
  Orchestrator _orchestrator_shelfino2(_rrt.get_graph(), _rrt.get_lookup());

  // find the two shortest path
  RCLCPP_INFO(m->get_logger(), "A* search for shelfino 1 started");
  auto path_astar1 = _orchestrator_shelfino1.astar_search(start_shelfino1, *(path.end() - 1));
  RCLCPP_INFO(m->get_logger(), "A* search for shelfino 1 ended");
  RCLCPP_INFO(m->get_logger(), "A* search for shelfino 2 started");
  auto path_astar2 = _orchestrator_shelfino2.astar_search(start_shelfino2, *(path.end() - 1));
  RCLCPP_INFO(m->get_logger(), "A* search for shelfino 2 ended");

  // smooth it
  RCLCPP_INFO(m->get_logger(), "Smoothing the paths");
  RCLCPP_INFO(m->get_logger(), "Smoothing the paths");
  auto shelfino1_path = _rrt.smooth_path(path_astar1, map);
  auto shelfino2_path = _rrt.smooth_path(path_astar2, map);

  // Display path
  if (DISPLAY_PATH_1)
  {
    node->setPathPoints(shelfino1_path);
  }
  if (DISPLAY_PATH_2)
  {
    node->setPathPoints(shelfino2_path);
  }
  RCLCPP_INFO(m->get_logger(), "\033[1;32m Path founded! \033[0m");

  //* Create dubinized version of the path
  RCLCPP_INFO(m->get_logger(), "\033[1;32m Dubinising paths \033[0m");
  Dubins d;
  // For creating the dubins path, we need to remove the start and the goal
  shelfino1_path.erase(shelfino1_path.begin());
  shelfino1_path.erase(shelfino1_path.end());
  shelfino2_path.erase(shelfino2_path.begin());
  shelfino2_path.erase(shelfino2_path.end());

  auto shelfino1_d_path = d.dubins_multi_point(start_shelfino1.at(0), start_shelfino1.at(1), m->get_pose1().at(2), goal.at(0), goal.at(1), m->get_gate().at(2), shelfino1_path, kmax, map);
  RCLCPP_INFO(m->get_logger(), "Dubins path for shelfino 1 created");
  auto shelfino2_d_path = d.dubins_multi_point(start_shelfino2.at(0), start_shelfino2.at(1), m->get_pose2().at(2), goal.at(0), goal.at(1), m->get_gate().at(2), shelfino2_path, kmax, map);
  RCLCPP_INFO(m->get_logger(), "Dubins path for shelfino 2 created");
  auto shelfino1_nav2 = convertDubinsPathToNavPath(shelfino1_d_path);
  auto shelfino2_nav2 = convertDubinsPathToNavPath(shelfino2_d_path);

  shelfino1_path.push_back({goal.at(0), goal.at(1)});
  shelfino1_path.insert(shelfino1_path.begin(), {start_shelfino1.at(0), start_shelfino1.at(1)});
  shelfino2_path.push_back({goal.at(0), goal.at(1)});
  shelfino2_path.insert(shelfino2_path.begin(), {start_shelfino2.at(0), start_shelfino2.at(1)});

  // Check collisions between the two paths
  RCLCPP_INFO(m->get_logger(), "\033[1;32m Collision checking \033[0m");
  bool collision = true;
  bool first_reschedule = true;
  while (collision)
  {
    int collision_index = _orchestrator_shelfino1.checkIntersection(shelfino1_nav2, shelfino2_nav2);
    if (collision_index != -1)
    {
      RCLCPP_INFO(m->get_logger(), "\033[1;33m Found a collision \033[0m");
      // get the collision point as a KDNode_t
      KDNode_t collision_point = {shelfino1_nav2.poses.at(collision_index).pose.position.x, shelfino1_nav2.poses.at(collision_index).pose.position.y};
      double score1 = _orchestrator_shelfino1.compute_score(shelfino1_nav2, collision_index);
      double score2 = _orchestrator_shelfino2.compute_score(shelfino2_nav2, collision_index);
      // print scores

      // who has the largest score has to deviate the path
      if (score1 > score2)
      {
        shelfino1_path = _orchestrator_shelfino1.reschedule_path(shelfino1_path, collision_point, 0.5, first_reschedule);
        shelfino1_path.erase(shelfino1_path.begin());
        shelfino1_path.erase(shelfino1_path.end());
        shelfino1_d_path = d.dubins_multi_point(start_shelfino1.at(0), start_shelfino1.at(1), m->get_pose1().at(2), goal.at(0), goal.at(1), m->get_gate().at(2), shelfino1_path, kmax, map);
        shelfino1_nav2 = convertDubinsPathToNavPath(shelfino1_d_path);
        shelfino1_path.push_back({goal.at(0), goal.at(1)});
        shelfino1_path.insert(shelfino1_path.begin(), {start_shelfino1.at(0), start_shelfino1.at(1)});
      }
      else
      {
        // reschedule for shelfino 2
        shelfino2_path = _orchestrator_shelfino2.reschedule_path(shelfino2_path, collision_point, 0.5, first_reschedule);
        shelfino2_path.erase(shelfino2_path.begin());
        shelfino2_path.erase(shelfino2_path.end());
        shelfino2_d_path = d.dubins_multi_point(start_shelfino2.at(0), start_shelfino2.at(1), m->get_pose2().at(2), goal.at(0), goal.at(1), m->get_gate().at(2), shelfino2_path, kmax, map);
        shelfino2_nav2 = convertDubinsPathToNavPath(shelfino2_d_path);
        shelfino2_path.push_back({goal.at(0), goal.at(1)});
        shelfino2_path.insert(shelfino2_path.begin(), {start_shelfino2.at(0), start_shelfino2.at(1)});
      }
      first_reschedule = false;
    }
    else
    {
      RCLCPP_INFO(m->get_logger(), "\033[1;32m No collisions \033[0m");
      collision = false;
    }
  }
  RCLCPP_INFO(m->get_logger(), "\033[1;32m Sending paths to nav2 controller \033[0m");
  if (shelfino1_nav2.poses.size() == 0 || shelfino2_nav2.poses.size() == 0)
  {
    throw std::runtime_error("Empty path");
    return -1;
  }
  node->send_nav2(shelfino1_nav2, shelfino2_nav2);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
