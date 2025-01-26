#include "graph_generator/combinatorial_based/utilities.hpp"
#include "graph_generator/combinatorial_based/map_utilities.hpp"
#include "graph_generator/combinatorial_based/map_construction.hpp"
#include "graph_generator/marker_publishers/map_edges_publisher.hpp"
#include "graph_generator/combinatorial_based/shortest_graph.hpp"

#include "graph_generator/utils/orchestrator.hpp"

#include "graph_generator/graph_node.hpp"

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

    // parameters
    this->declare_parameter<int>("points_to_sample", 3000);

    int points_to_sample = this->get_parameter("points_to_sample").as_int();

    RCLCPP_INFO(this->get_logger(), "Point Marker Node started");
  }

  int get_points_to_sample()
  {
    return points_to_sample;
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
    marker.header.frame_id = "map";
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
  int points_to_sample;
};

template <typename... Args>
void log_message(rclcpp::Logger logger, rclcpp::Logger::Level level, Args &&...args)
{
  std::ostringstream oss;
  (oss << ... << std::forward<Args>(args)); // Fold expression to concatenate args
  switch (level)
  {
  case rclcpp::Logger::Level::Debug:
    RCLCPP_DEBUG(logger, "%s", oss.str().c_str());
    break;
  case rclcpp::Logger::Level::Info:
    RCLCPP_INFO(logger, "%s", oss.str().c_str());
    break;
  case rclcpp::Logger::Level::Warn:
    RCLCPP_WARN(logger, "%s", oss.str().c_str());
    break;
  case rclcpp::Logger::Level::Error:
    RCLCPP_ERROR(logger, "%s", oss.str().c_str());
    break;
  case rclcpp::Logger::Level::Fatal:
    RCLCPP_FATAL(logger, "%s", oss.str().c_str());
    break;
  default:
    RCLCPP_INFO(logger, "%s", oss.str().c_str());
  }
}

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

size_t checkIntersection(const nav_msgs::msg::Path &path1, const nav_msgs::msg::Path &path2)
{
  // Get the sizes of the paths
  size_t path1_size = path1.poses.size();
  size_t path2_size = path2.poses.size();

  // Get the smaller path size to prevent out-of-bounds access
  size_t min_size = std::min(path1_size, path2_size);

  // Loop through points in the two paths
  for (size_t i = 0; i < min_size; i++)
  {
    const auto &p1 = path1.poses.at(i).pose.position;
    const auto &p2 = path2.poses.at(i).pose.position;

    // Check if the distance between points is less than twice the robot radius
    double distance = std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    if (distance < 2 * 0.4)
    {
      return i; // Collision detected at this time step
    }
  }

  // Return a special value to indicate no collision
  return -1; // No collision detected
}

double compute_score(const nav_msgs::msg::Path &path, size_t collision_point)
{
  if (collision_point >= path.poses.size())
  {
    throw std::out_of_range("Collision point is out of bounds.");
  }

  // Compute the distance along the path from the collision point to the goal
  double total_distance = 0.0;
  for (size_t i = collision_point; i < path.poses.size() - 1; ++i)
  {
    const auto &current_pose = path.poses[i].pose.position;
    const auto &next_pose = path.poses[i + 1].pose.position;

    double dx = next_pose.x - current_pose.x;
    double dy = next_pose.y - current_pose.y;
    total_distance += std::sqrt(dx * dx + dy * dy);
  }

  // Compute the score; higher score for shorter distances
  // Avoid division by zero if total_distance is very small
  return (total_distance > 1e-6) ? (1.0 / total_distance) : std::numeric_limits<double>::infinity();
}

std::vector<KDNode_t> reschedule_path(std::vector<KDNode_t> path, KDNode_t collision_point, double step_size)
{
  std::vector<KDNode_t> new_path = path;
  // if the path is point to point
  if (path.size() == 2)
  {
    // take the mid point between collision and start
    KDNode_t mid_point = {(path.at(0).at(0) + collision_point.at(0)) / 2, (path.at(0).at(1) + collision_point.at(1)) / 2};
    // find the direction orthogonal to the goal
    KDNode_t direction = {path.at(0).at(1) - collision_point.at(1), collision_point.at(0) - path.at(0).at(0)};
    // normalize the direction
    double norm = sqrt(pow(direction.at(0), 2) + pow(direction.at(1), 2));
    direction.at(0) /= norm;
    direction.at(1) /= norm;
    // shift wrt the direction the mid point of a step size
    // KDNode_t new_point = {mid_point.at(0) - step_size * direction.at(0), mid_point.at(1) - step_size * direction.at(1)};
    KDNode_t new_point = {path.at(0).at(0) - step_size * direction.at(0), path.at(0).at(1) - step_size * direction.at(1)};
    new_path.insert(new_path.begin() + 1, collision_point);
    new_path.insert(new_path.begin() + 1, new_point);
  }
  // if the path has more point
  else
  {
    // take mid point from start and first point in path
    KDNode_t mid_point = {(path.at(0).at(0) + path.at(1).at(0)) / 2, (path.at(0).at(1) + path.at(1).at(1)) / 2};
    // find the direction orthogonal to the goal
    KDNode_t direction = {path.at(0).at(1) - path.at(1).at(1), path.at(1).at(0) - path.at(0).at(0)};
    // normalize the direction
    double norm = sqrt(pow(direction.at(0), 2) + pow(direction.at(1), 2));
    direction.at(0) /= norm;
    direction.at(1) /= norm;
    // shift wrt the direction the mid point of a step size
    // KDNode_t new_point = {mid_point.at(0) + step_size * direction.at(0), mid_point.at(1) + step_size * direction.at(1)};
    KDNode_t new_point = {path.at(0).at(0) + step_size * direction.at(0), path.at(0).at(1) + step_size * direction.at(1)};
    // insert the new point as a first point
    new_path.insert(new_path.begin() + 1, collision_point);
    new_path.insert(new_path.begin() + 1, new_point);
  }

  return new_path;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto mappa = std::make_shared<GraphGenerator>();
  RCLCPP_INFO(mappa->get_logger(), "Waiting for obstacles, borders and gates...");
  while (!mappa->obstacles_r_ || !mappa->borders_r_ || !mappa->gates_r_ || !mappa->pos1_r_ || !mappa->pos2_r_)
  {
    rclcpp::spin_some(mappa->get_node_base_interface());
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  auto map = mappa->get_map();

  auto m = std::make_shared<MapConstruction>();
  auto logger = m->get_logger();
  auto log_level = rclcpp::Logger::Level::Info;
  log_message(logger, log_level, "Waiting for obstacles, borders and gates...");
  while (!m->obstacles_r_ || !m->borders_r_ || !m->gates_r_ || !m->pos1_r_)
  {
    rclcpp::spin_some(m->get_node_base_interface());
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  log_message(logger, log_level, "\033[1;32m Map info obtained\033[0m \n Building map");
  auto inflated_obstacles = m->get_inflated_map();
  auto inflated_obstacles_unionized = m->get_unionized_inflated_map();
  auto inflated_border = m->get_inflated_border();
  auto clean_map = m->get_clean_map();
  auto pose_shellfino1 = m->get_pose1();
  auto pose_shellfino2 = m->get_pose2();
  auto pose_gate = m->get_gate();

  point_t position_shellfino_1 = point_t({pose_shellfino1[0], pose_shellfino1[1]});
  point_t position_shellfino_2 = point_t({pose_shellfino2[0], pose_shellfino2[1]});
  point_t position_gate = point_t({pose_gate[0], pose_gate[1]});

  log_message(logger, log_level, "\033[1;32m Number of obstacles", m->get_obstacles().size(), "\033[0m");
  log_message(logger, log_level, "\033[1;32m Map built\033[0m");
  log_message(logger, log_level, "clean map objects : ", clean_map.size(), " inflated objects : ", inflated_obstacles.size());
  log_message(logger, log_level, "\033[1;32m Cutting edges \033[0m");

  // Edge Calculations----------------------------------------------------------------
  auto cuts = get_cut_lines(clean_map, inflated_obstacles, SHELFINO_INFLATION);
  log_message(logger, log_level, "\033[1;32m Generating Graph \033[0m");

  auto cutted_map_poly = cutted_map(inflated_obstacles_unionized, cuts);
  auto map_in_lines = map_to_lines(cutted_map_poly, inflated_border);
  auto shellfino1_to_map = find_shellfino_map_links(inflated_obstacles_unionized, position_shellfino_1, position_gate);
  auto shellfino2_to_map = find_shellfino_map_links(inflated_obstacles_unionized, position_shellfino_2, position_gate);
  auto gate_to_map = find_point_map_links(inflated_obstacles_unionized, position_gate);

  std::vector<line_t> bitangents_lines = find_links(cutted_map_poly);

  log_message(logger, log_level, "\033[1;32m Number of edges", bitangents_lines.size(), "\033[0m");

  // Graph Search------------------------------------------------------------------------------
  auto graph_generator = ShortestGraph(shellfino1_to_map, shellfino2_to_map, gate_to_map, map_in_lines, position_shellfino_1, position_shellfino_2, position_gate);
  auto points_path1 = graph_generator.get_shellfino_1_path();
  auto points_path2 = graph_generator.get_shellfino_2_path();

  log_message(logger, log_level, "\033[1;32m Dubinizing \033[0m");

  // Conversion to Dubins---------------------------------------------------------------------------
  auto dubins_publisher = std::make_shared<PointMarkerNode>();
  auto converted_path1 = convert_points(points_path1);
  auto converted_path2 = convert_points(points_path2);

  std::reverse(converted_path1.begin(), converted_path1.end());
  converted_path1.erase(converted_path1.begin()); // TODO FIX THE DOUBLE START
  converted_path1.erase(converted_path1.begin()); // TODO FIX THE DOUBLE START
  converted_path1.erase(converted_path1.end());

  std::reverse(converted_path2.begin(), converted_path2.end());
  converted_path2.erase(converted_path2.begin()); // TODO FIX THE DOUBLE START
  converted_path2.erase(converted_path2.begin()); // TODO FIX THE DOUBLE STARTù
  converted_path2.erase(converted_path2.end());

  Dubins d;
  RCLCPP_INFO(m->get_logger(), "\033[1;32m Converted points\033[0m");
  for (auto converted_point : converted_path1)
  {
    log_message(logger, log_level, "x ", converted_point.at(0), " y ", converted_point.at(1));
  }

  auto dubins_path_1 = d.dubins_multi_point(pose_shellfino1.at(0), pose_shellfino1.at(1), m->get_pose1().at(2), pose_gate.at(0), pose_gate.at(1), m->get_gate().at(2), converted_path1, 2, map);
  auto dubins_path_2 = d.dubins_multi_point(pose_shellfino2.at(0), pose_shellfino2.at(1), m->get_pose2().at(2), pose_gate.at(0), pose_gate.at(1), m->get_gate().at(2), converted_path2, 2, map);

  auto shelfino1_nav2 = convertDubinsPathToNavPath(dubins_path_1);
  auto shelfino2_nav2 = convertDubinsPathToNavPath(dubins_path_2);

  // Collision checking-----------------------------------------------------------------------
  converted_path1.push_back({pose_gate.at(0), pose_gate.at(1)});
  converted_path1.insert(converted_path1.begin(), {pose_shellfino1.at(0), pose_shellfino1.at(1)});
  converted_path2.push_back({pose_gate.at(0), pose_gate.at(1)});
  converted_path2.insert(converted_path2.begin(), {pose_shellfino2.at(0), pose_shellfino2.at(1)});

  int collision_index = checkIntersection(shelfino1_nav2, shelfino2_nav2);
  if (collision_index != -1)
  {
    RCLCPP_INFO(m->get_logger(), "\033[1;33m Found a collision \033[0m");
    // get the collision point as a KDNode_t
    KDNode_t collision_point = {shelfino1_nav2.poses.at(collision_index).pose.position.x, shelfino1_nav2.poses.at(collision_index).pose.position.y};
    double score1 = compute_score(shelfino1_nav2, collision_index);
    double score2 = compute_score(shelfino2_nav2, collision_index);
    // print scores

    // who has the largest score has to deviate the path
    if (score1 > score2)
    {
      converted_path1 = reschedule_path(converted_path1, collision_point, 0.5);
      converted_path1.erase(converted_path1.begin());
      converted_path1.erase(converted_path1.end());
      dubins_path_1 = d.dubins_multi_point(pose_shellfino1.at(0), pose_shellfino1.at(1), m->get_pose1().at(2), pose_gate.at(0), pose_gate.at(1), m->get_gate().at(2), converted_path1, 2.0, map);
      shelfino1_nav2 = convertDubinsPathToNavPath(dubins_path_1);
      converted_path1.push_back({pose_gate.at(0), pose_gate.at(1)});
      converted_path1.insert(converted_path1.begin(), {pose_shellfino1.at(0), pose_shellfino1.at(1)});
    }
    else
    {
      // reschedule for shelfino 2
      converted_path2 = reschedule_path(converted_path2, collision_point, 0.5);
      converted_path2.erase(converted_path2.begin());
      converted_path2.erase(converted_path2.end());
      dubins_path_2 = d.dubins_multi_point(pose_shellfino2.at(0), pose_shellfino2.at(1), m->get_pose2().at(2), pose_gate.at(0), pose_gate.at(1), m->get_gate().at(2), converted_path2, 2.0, map);
      shelfino2_nav2 = convertDubinsPathToNavPath(dubins_path_2);
      converted_path2.push_back({pose_gate.at(0), pose_gate.at(1)});
      converted_path2.insert(converted_path2.begin(), {pose_shellfino2.at(0), pose_shellfino2.at(1)});
    }
  }
  else
  {
    RCLCPP_INFO(m->get_logger(), "\033[1;32m No collisions \033[0m");
  }

  dubins_publisher->send_nav2(shelfino1_nav2, shelfino2_nav2);
  //-----------------------------------------------------------------------
  std::vector<line_t>
      lines_path1;
  for (size_t i = 0; i < points_path1.size() - 1; i++)
  {
    lines_path1.emplace_back(line_t({points_path1[i], points_path1[i + 1]}));
  }

  auto inflated_unionized_publisher = std::make_shared<MapEdgePublisherNode>(inflated_obstacles_unionized, "inflated_unionized_map_edges");
  auto inflated_publisher = std::make_shared<MapEdgePublisherNode>(inflated_obstacles, "inflated_map_edges");
  auto clean_publisher = std::make_shared<MapEdgePublisherNode>(clean_map, "clean_map_edges");
  auto cutted_map_publisher = std::make_shared<MapEdgePublisherNode>(cutted_map_poly, "cutted_map_edges");
  auto graph_publisher = std::make_shared<SimpleEdgePublisherNode>(bitangents_lines, "bitangent_graph");
  auto map_in_lines_publisher = std::make_shared<SimpleEdgePublisherNode>(map_in_lines, "map_in_lines_graph");
  auto point_to_map_publisher = std::make_shared<SimpleEdgePublisherNode>(shellfino1_to_map, "point_to_map_edges");
  auto path_1_publisher = std::make_shared<SimpleEdgePublisherNode>(lines_path1, "path_graph");
  auto point_publisher = std::make_shared<SimpleEdgePublisherNode>(cuts, "cuts_publisher");
  log_message(logger, log_level, "\033[1;32m Start publishing \033[0m");

  while (true)
  {
    // rclcpp::spin_some(dubins_publisher);

    rclcpp::spin_some(clean_publisher);
    rclcpp::spin_some(inflated_unionized_publisher);
    rclcpp::spin_some(inflated_publisher);
    rclcpp::spin_some(graph_publisher);
    rclcpp::spin_some(point_publisher);
    rclcpp::spin_some(map_in_lines_publisher);
    rclcpp::spin_some(cutted_map_publisher);
    rclcpp::spin_some(point_to_map_publisher);
    rclcpp::spin_some(path_1_publisher);

    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  rclcpp::shutdown();
  return 0;
}
