#include <time.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include "graph_generator/map_generator.hpp"
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

#include <chrono>

#define DISPLAY_SAMPLES 0
#define DISPLAY_PATH_1 0
#define DISPLAY_PATH_2 0

using namespace std;
using namespace std::chrono;

class PublishNav2Path : public rclcpp::Node
{
public:
  PublishNav2Path()
      : Node("publish_nav2"), timer_interval_ms_(500)
  {

    shelfino1_nav2_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "shelfino1/plan1", rclcpp::QoS(10));
    shelfino2_nav2_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "shelfino2/plan1", rclcpp::QoS(10));
    // Timer to periodically check for subscribers and publish
  }

 
  void send_nav2(const nav_msgs::msg::Path &path1, const nav_msgs::msg::Path &path2)
  {
    shelfino1_nav2_path_pub_->publish(path1);
    shelfino2_nav2_path_pub_->publish(path2);
  }

private:
  
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr shelfino1_nav2_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr shelfino2_nav2_path_pub_;
  int timer_interval_ms_;
};

nav_msgs::msg::Path convertDubinsPathToNavPath(const std::vector<dubins_curve> &dubins_curves)
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "map";
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
  // SETUP ---------------------------------------------------------------------
  rclcpp::init(argc, argv);

  // Create the node
  auto node = std::make_shared<PublishNav2Path>();

  // curvature max for the shelfinos
  double kmax = 3;

  // flag for checking the addition of the shelfino 2
  bool shelfino2_added = false;

  // BUILD MAP -----------------------------------------------------------------
  auto start = high_resolution_clock::now();
  // Used for generating the map
  auto m = std::make_shared<MapGenerator>();

  // Wait that all the info for building the map are received
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
  auto stop = high_resolution_clock::now();

  auto duration = duration_cast<seconds>(stop - start);

  RCLCPP_INFO(m->get_logger(), "Time taken by function: %ld seconds", duration.count());

  RCLCPP_INFO(m->get_logger(), "Sampling test");

  // PRE MAPF -------------------------------------------------------------------
  RCLCPP_INFO(m->get_logger(), "Calculating distances and scoring...");

  // Get obstacles and positions
  auto obstacles = m->get_obstacles();
  auto shelfino1_pos = m->get_pose1(); // [x, y, theta]
  auto shelfino2_pos = m->get_pose2();

  // Function to calculate distance from a point to all obstacles
  auto calculate_distance = [](const pose_t &shelfino_pos, const std::vector<polygon_t> &obstacles) -> double
  {
    double min_distance = std::numeric_limits<double>::max();
    for (const auto &obstacle : obstacles)
    {
      for (const auto &pt : obstacle.outer())
      {
        double dist = boost::geometry::distance(point_t(shelfino_pos[0], shelfino_pos[1]), pt);
        if (dist < min_distance)
        {
          min_distance = dist;
        }
      }
    }
    return min_distance;
  };

  // Calculate distances
  double shelfino1_distance = calculate_distance(shelfino1_pos, obstacles);
  double shelfino2_distance = calculate_distance(shelfino2_pos, obstacles);

  // Scoring logic
  double shelfino1_score = shelfino1_distance > 1.0 ? shelfino1_distance * 10 : 0; // Example scoring rule
  double shelfino2_score = shelfino2_distance > 1.0 ? shelfino2_distance * 10 : 0;

  // Print results
  RCLCPP_INFO(m->get_logger(), "Shelfino 1 Distance: %f, Score: %f", shelfino1_distance, shelfino1_score);
  RCLCPP_INFO(m->get_logger(), "Shelfino 2 Distance: %f, Score: %f", shelfino2_distance, shelfino2_score);

  // RRT -----------------------------------------------------------------------
  start = high_resolution_clock::now();
  // Create the RRT object
  RRT _rrt;

  // Variables for storing the sampled points and the paths
  std::vector<std::vector<double>> sampled_points;
  std::vector<std::vector<double>> path;

  // Retrive pose for shelfino 1
  KDNode_t start_shelfino1 = {m->get_pose1().at(0), m->get_pose1().at(1)};
  KDNode_t goal = {m->get_gate().at(0), m->get_gate().at(1)};
  vector<KDNode_t> rrt_path;
  _rrt.set_problem(start_shelfino1, goal);

  // Main tree -> created from shelfino1 (sample 8000 points)

  for (uint i = 1; i < 8000; i++)
  {
    auto point = _rrt.get_random_point(i, map);                           // sample random point
    auto nearest = _rrt.get_nn(point, 1);                                 // get nn
    auto new_point = _rrt.next_point(point, nearest, map);                // get next point by steering
    auto best_one = _rrt.get_best_neighbor(new_point, nearest, 0.3, map); // get best neighbor
    if (_rrt.add_edge(new_point, best_one, map))                          // add edge if the new point is not in collision
    {
      sampled_points.push_back(new_point);
      _rrt.rewire(new_point, 0.3, map); // rewire the tree
      if (_rrt.is_goal(new_point))      // if goal is reached break
      {
        rrt_path = _rrt.get_path(new_point);
        break;
      }
    }
  }

  // Check if a path is found
  if (rrt_path.empty())
  {
    RCLCPP_ERROR(m->get_logger(), "No path found, try sampling more points");
    return -1;
  }

  RCLCPP_INFO(m->get_logger(), "\033[1;32m Founded first path \033[0m");

  // Retrive pose for shelfino 2
  KDNode_t start_shelfino2 = {m->get_pose2().at(0), m->get_pose2().at(1)};

  RCLCPP_INFO(m->get_logger(), "Retrived position for shelfino 2");

  RRT _rrt2;
  vector<KDNode_t> rrt_path2;
  _rrt2.set_problem(start_shelfino2, goal);
  for (uint i = 1; i < 8000; i++)
  {
    auto point = _rrt2.get_random_point(i, map);                           // sample random point
    auto nearest = _rrt2.get_nn(point, 1);                                 // get nn
    auto new_point = _rrt2.next_point(point, nearest, map);                // get next point by steering
    auto best_one = _rrt2.get_best_neighbor(new_point, nearest, 0.3, map); // get best neighbor
    if (_rrt2.add_edge(new_point, best_one, map))                          // add edge if the new point is not in collision
    {
      sampled_points.push_back(new_point);
      _rrt2.rewire(new_point, 0.3, map); // rewire the tree
      if (_rrt2.is_goal(new_point))      // if goal is reached break
      {
        rrt_path2 = _rrt2.get_path(new_point);
        break;
      }
    }
  }
  if (rrt_path2.empty())
  {
    RCLCPP_ERROR(m->get_logger(), "No path found, try sampling more points");
    return -1;
  }
  RCLCPP_INFO(m->get_logger(), "\033[1;32m Founded second path \033[0m");

  // instantiate the two orchestrators
  Orchestrator _orchestrator_shelfino1(_rrt.get_graph(), _rrt.get_lookup());
  Orchestrator _orchestrator_shelfino2(_rrt2.get_graph(), _rrt2.get_lookup());

  auto path_astar1_shelfino1 = _orchestrator_shelfino1.astar_search(start_shelfino1, *(rrt_path.end() - 1));
  path_astar1_shelfino1.push_back(goal);
  auto path_astar2_shelfino2 = _orchestrator_shelfino2.astar_search(start_shelfino2, *(rrt_path2.end() - 1));
  path_astar2_shelfino2.push_back(goal);

  auto shelfino1_path1 = _rrt.smooth_path(path_astar1_shelfino1, map);
  auto shelfino2_path2 = _rrt2.smooth_path(path_astar2_shelfino2, map);

  auto nearest_shelfino1 = _rrt2.get_nn(start_shelfino1, 1);
  auto nearest_shelfino2 = _rrt.get_nn(start_shelfino2, 1);
  std::vector<KDNode_t> path_astar1_shelfino2;
  std::vector<KDNode_t> path_astar2_shelfino1;
  std::vector<KDNode_t> shelfino1_path2;
  std::vector<KDNode_t> shelfino2_path1;

  if (_rrt.attach_node(start_shelfino2, nearest_shelfino2, map))
  {
    path_astar1_shelfino2 = _orchestrator_shelfino1.astar_search(start_shelfino2, *(rrt_path.end() - 1));
    path_astar1_shelfino2.push_back(goal);
    shelfino2_path1 = _rrt.smooth_path(path_astar1_shelfino2, map);
  }

  if (_rrt2.attach_node(start_shelfino1, nearest_shelfino2, map))
  {
    path_astar2_shelfino1 = _orchestrator_shelfino2.astar_search(start_shelfino1, *(rrt_path2.end() - 1));
    path_astar2_shelfino1.push_back(goal);
    shelfino1_path2 = _rrt2.smooth_path(path_astar2_shelfino1, map);
  }
  std::vector<std::pair<std::vector<KDNode_t>, std::vector<KDNode_t>>> possible_paths;
  possible_paths.push_back({shelfino1_path1, shelfino2_path1});
  possible_paths.push_back({shelfino1_path1, shelfino2_path2});
  possible_paths.push_back({shelfino1_path2, shelfino2_path1});
  possible_paths.push_back({shelfino1_path2, shelfino2_path2});
  stop = high_resolution_clock::now();

  duration = duration_cast<seconds>(stop - start);

  RCLCPP_INFO(m->get_logger(), "Time taken by function: %ld seconds", duration.count());

  // DUBINS -------------------------------------------------------------------
  RCLCPP_INFO(m->get_logger(), "\033[1;32m Dubinising paths \033[0m");
  start = high_resolution_clock::now();
  Dubins d;
  std::vector<struct dubins_curve> shelfino1_d_path;
  std::vector<struct dubins_curve> shelfino2_d_path;
  std::vector<KDNode_t> shelfino1_path;
  std::vector<KDNode_t> shelfino2_path;
  nav_msgs::msg::Path shelfino1_nav2;
  nav_msgs::msg::Path shelfino2_nav2;
  for (auto path : possible_paths)
  {
    shelfino1_path = path.first;
    shelfino2_path = path.second;

    // output the paths
    for (const auto &p : shelfino1_path)
    {
      RCLCPP_INFO(m->get_logger(), "Shelfino 1 path: %f %f", p[0], p[1]);
    }

    for (const auto &p : shelfino2_path)
    {
      RCLCPP_INFO(m->get_logger(), "Shelfino 2 path: %f %f", p[0], p[1]);
    }
    // log the sizes of the paths
    RCLCPP_INFO(m->get_logger(), "Shelfino 1 path size: %d", shelfino1_path.size());
    RCLCPP_INFO(m->get_logger(), "Shelfino 2 path size: %d", shelfino2_path.size());

    if (shelfino1_path.size() == 0 || shelfino2_path.size() == 0)
    {
      continue;
    }

    shelfino1_path.erase(shelfino1_path.begin());
    shelfino1_path.erase(shelfino1_path.end());
    shelfino2_path.erase(shelfino2_path.begin());
    shelfino2_path.erase(shelfino2_path.end());
    try
    {
      shelfino1_d_path = d.dubins_multi_point(start_shelfino1.at(0), start_shelfino1.at(1), m->get_pose1().at(2), goal.at(0), goal.at(1), m->get_gate().at(2), shelfino1_path, kmax, map);
      shelfino2_d_path = d.dubins_multi_point(start_shelfino2.at(0), start_shelfino2.at(1), m->get_pose2().at(2), goal.at(0), goal.at(1), m->get_gate().at(2), shelfino2_path, kmax, map);
      RCLCPP_INFO(m->get_logger(), "Builded Dubins");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(m->get_logger(), "Error in Dubins");
      continue;
    }
    break;
  }

  shelfino1_path.push_back({goal.at(0), goal.at(1)});
  shelfino1_path.insert(shelfino1_path.begin(), {start_shelfino1.at(0), start_shelfino1.at(1)});
  shelfino2_path.push_back({goal.at(0), goal.at(1)});
  shelfino2_path.insert(shelfino2_path.begin(), {start_shelfino2.at(0), start_shelfino2.at(1)});
  shelfino1_nav2 = convertDubinsPathToNavPath(shelfino1_d_path);
  shelfino2_nav2 = convertDubinsPathToNavPath(shelfino2_d_path);

  stop = high_resolution_clock::now();

  duration = duration_cast<seconds>(stop - start);

  RCLCPP_INFO(m->get_logger(), "Time taken by function: %ld seconds", duration.count());

  // Check collisions between the two paths
  RCLCPP_INFO(m->get_logger(), "\033[1;32m Collision checking \033[0m");

  start = high_resolution_clock::now();
  bool solved_collision = false;

  while (!solved_collision)
  {
    int collision_index = _orchestrator_shelfino1.checkIntersection(shelfino1_nav2, shelfino2_nav2);
    if (collision_index != -1)
    {
      RCLCPP_INFO(m->get_logger(), "\033[1;33m Found a collision \033[0m");
      // get the collision point as a KDNode_t
      KDNode_t collision_point = {shelfino1_nav2.poses.at(collision_index).pose.position.x, shelfino1_nav2.poses.at(collision_index).pose.position.y};

      // who has the largest score has to deviate the path
      if (shelfino1_score < shelfino2_score)
      {
        shelfino1_path = _orchestrator_shelfino1.reschedule_path(shelfino1_path, start_shelfino1, map);
        shelfino1_path.erase(shelfino1_path.begin());
        shelfino1_path.erase(shelfino1_path.end());
        try
        {
          shelfino1_d_path = d.dubins_multi_point(start_shelfino1.at(0), start_shelfino1.at(1), m->get_pose1().at(2), goal.at(0), goal.at(1), m->get_gate().at(2), shelfino1_path, kmax, map);
        }
        catch (const std::exception &e)
        {
          RCLCPP_ERROR(m->get_logger(), "Error in Dubins");
          continue;
        }
        shelfino1_nav2 = convertDubinsPathToNavPath(shelfino1_d_path);
        shelfino1_path.push_back({goal.at(0), goal.at(1)});
        shelfino1_path.insert(shelfino1_path.begin(), {start_shelfino1.at(0), start_shelfino1.at(1)});
      }
      else
      {
        // reschedule for shelfino 2
        shelfino2_path = _orchestrator_shelfino1.reschedule_path(shelfino2_path, start_shelfino2, map);
        shelfino2_path.erase(shelfino2_path.begin());
        shelfino2_path.erase(shelfino2_path.end());
        try
        {
          shelfino2_d_path = d.dubins_multi_point(start_shelfino2.at(0), start_shelfino2.at(1), m->get_pose2().at(2), goal.at(0), goal.at(1), m->get_gate().at(2), shelfino2_path, kmax, map);
        }
        catch (const std::exception &e)
        {
          RCLCPP_ERROR(m->get_logger(), "Error in Dubins");
          continue;
        }
        shelfino2_nav2 = convertDubinsPathToNavPath(shelfino2_d_path);
        shelfino2_path.push_back({goal.at(0), goal.at(1)});
        shelfino2_path.insert(shelfino2_path.begin(), {start_shelfino2.at(0), start_shelfino2.at(1)});
      }
    }
    else
    {
      solved_collision = true;
      RCLCPP_INFO(m->get_logger(), "\033[1;32m No collisions \033[0m");
    }
  }
  stop = high_resolution_clock::now();

  duration = duration_cast<seconds>(stop - start);

  RCLCPP_INFO(m->get_logger(), "Time taken by function: %ld seconds", duration.count());

  RCLCPP_INFO(m->get_logger(), "\033[1;32m Sending paths to nav2 controller \033[0m");

  if (shelfino2_nav2.poses.size() == 0)
  {
    throw std::runtime_error("Empty path 2");
    return -1;
  }
  if (shelfino1_nav2.poses.size() == 0)
  {
    throw std::runtime_error("Empty path 1");
    return -1;
  }
  node->send_nav2(shelfino1_nav2, shelfino2_nav2);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}