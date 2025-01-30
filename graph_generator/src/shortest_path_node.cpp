#include "graph_generator/combinatorial_based/utilities.hpp"
#include "graph_generator/combinatorial_based/map_utilities.hpp"
#include "graph_generator/combinatorial_based/map_construction.hpp"
#include "graph_generator/marker_publishers/map_edges_publisher.hpp"
#include "graph_generator/marker_publishers/PointMarker.hpp"

#include "graph_generator/combinatorial_based/path_utilities.hpp"
#include "graph_generator/combinatorial_based/shortest_graph.hpp"

std::vector<KDNode_t> reschedule_path(std::vector<KDNode_t> path, KDNode_t start_point, boost::geometry::model::multi_polygon<polygon_t> &map)
{
  std::vector<KDNode_t> new_path = path;
  // sample U [0, 1]
  bool valid = false;
  double x = 0.0;
  double y = 0.0;
  do
  {
    float U = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    // sample theta random [0, 2pi)
    float theta = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (2 * M_PI)));
    double rho = 0.5 * sqrt(U);

    // find new x and y
    x = start_point.at(0) + rho * cos(theta);
    y = start_point.at(1) + rho * sin(theta);

    // check if the new point is valid
    if (boost::geometry::within(point_t(x, y), map))
    {
      valid = true;
    }
  } while (!valid);

  std::cout << "New point: " << start_point.at(0) << ", " << start_point.at(1) << std::endl;

  // insert the new point as a first point
  new_path.insert(new_path.begin() + 1, {x, y});

  return new_path;
}

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

#define TOLLERANCE SHELFINO_INFLATION * 2

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto m = std::make_shared<MapConstruction>();
  auto log_level = rclcpp::Logger::Level::Info;

  auto logger_class = m->get_logger();
  // Map for nav2---------------------------------------------------
  auto mappa = std::make_shared<MapGenerator>();
  log_message(logger_class, log_level, "Waiting for obstacles Map for nav2...");

  while (!mappa->obstacles_r_ || !mappa->borders_r_ || !mappa->gates_r_ || !mappa->pos1_r_ || !mappa->pos2_r_)
  {
    rclcpp::spin_some(mappa->get_node_base_interface());
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  auto map = mappa->get_map();

  // Map for shortest_path---------------------------------------------------

  log_message(logger_class, log_level, "Waiting for obstacles Map for shortest_path");
  while (!m->obstacles_r_ || !m->borders_r_ || !m->gates_r_ || !m->pos1_r_ || !m->pos2_r_)
  {
    rclcpp::spin_some(m->get_node_base_interface());
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  auto raw_obstacles = m->get_clean_map();
  auto pose_shellfino1 = m->get_pose1();
  auto pose_shellfino2 = m->get_pose2();
  auto pose_gate = m->get_gate();
  log_message(logger_class, log_level, "\033[1;32m Map info obtained\033[0m \n Building map");

  auto inflated_obstacles = m->get_inflated_map();
  auto inflated_border = m->get_inflated_border();
  auto cleaned_obstacles = m->get_unionized_inflated_map();
  map_edge_difference(cleaned_obstacles, inflated_border);
  log_message(logger_class, log_level, "\033[1;32m Building map \033[0m");

  point_t position_shellfino_1 = point_t({pose_shellfino1[0], pose_shellfino1[1]});
  point_t position_shellfino_2 = point_t({pose_shellfino2[0], pose_shellfino2[1]});
  point_t position_gate = point_t({pose_gate[0], pose_gate[1]});

  log_message(logger_class, log_level, "\033[1;32m Number of obstacles ", m->get_obstacles().size(), "\033[0m");
  log_message(logger_class, log_level, "\033[1;32m Map built\033[0m");
  log_message(logger_class, log_level, "clean map objects : ", raw_obstacles.size(), " inflated objects : ", inflated_obstacles.size());
  log_message(logger_class, log_level, "\033[1;32m Cutting edges \033[0m");

  // Cuts Calculations----------------------------------------------------------------
  auto cuts = get_cut_lines(raw_obstacles, inflated_obstacles, TOLLERANCE);
  log_message(logger_class, log_level, "\033[1;32m Generating Graph \033[0m");

  auto cutted_obstacles = apply_cuts_to_map(cleaned_obstacles, cuts);
  multi_polygon_t map_poly;
  for (auto poly : cutted_obstacles)
  {
    map_poly.emplace_back(poly);
  }
  // map_poly.emplace_back(inflated_border);

  auto map_in_lines = poly_to_lines(map_poly, inflated_border);

  auto shellfino1_edges_to_map = find_shellfino_map_links(cleaned_obstacles, position_shellfino_1, position_gate);
  auto shellfino2_edges_to_map = find_shellfino_map_links(cleaned_obstacles, position_shellfino_2, position_gate);
  auto gate_edges_to_map = find_point_map_links(cleaned_obstacles, position_gate);

  std::vector<line_t>
      edges_between_obstacles = find_edges_between_obstacles(cutted_obstacles);

  log_message(logger_class, log_level, "\033[1;32m Number of edges", edges_between_obstacles.size(), "\033[0m");

  // Graph Search------------------------------------------------------------------------------
  auto graph_generator = ShortestGraph(shellfino1_edges_to_map, shellfino2_edges_to_map, gate_edges_to_map, map_in_lines, position_shellfino_1, position_shellfino_2, position_gate);
  auto points_path1 = graph_generator.get_shellfino_1_path();
  auto points_path2 = graph_generator.get_shellfino_2_path();

  log_message(logger_class, log_level, "\033[1;32m Dubinizing \033[0m");

  // From optimal theoretical path -> physical movement---------------------------------------------------------------------------
  KDNode_t start_shelfino1 = {mappa->get_pose1().at(0), mappa->get_pose1().at(1)};
  KDNode_t start_shelfino2 = {mappa->get_pose2().at(0), mappa->get_pose2().at(1)};
  KDNode_t goal = {mappa->get_gate().at(0), mappa->get_gate().at(1)};
  auto converted_path1 = convert_points(points_path1);
  auto converted_path2 = convert_points(points_path2);
  Dubins d;
  std::vector<struct dubins_curve> shelfino1_d_path;
  std::vector<struct dubins_curve> shelfino2_d_path;

  std::reverse(converted_path1.begin(), converted_path1.end());
  converted_path1.erase(converted_path1.begin());
  converted_path1.erase(converted_path1.end());

  std::reverse(converted_path2.begin(), converted_path2.end());
  converted_path2.erase(converted_path2.begin());
  converted_path2.erase(converted_path2.end());

  try
  {
    shelfino1_d_path = d.dubins_multi_point(start_shelfino1.at(0), start_shelfino1.at(1), mappa->get_pose1().at(2), goal.at(0), goal.at(1), mappa->get_gate().at(2), converted_path1, 3, map);
    shelfino2_d_path = d.dubins_multi_point(start_shelfino2.at(0), start_shelfino2.at(1), mappa->get_pose2().at(2), goal.at(0), goal.at(1), mappa->get_gate().at(2), converted_path2, 3, map);
    RCLCPP_INFO(m->get_logger(), "Builded Dubins");
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(m->get_logger(), "Error in Dubins");
  }
  converted_path1.push_back({goal.at(0), goal.at(1)});
  converted_path1.insert(converted_path1.begin(), {start_shelfino1.at(0), start_shelfino1.at(1)});
  converted_path2.push_back({goal.at(0), goal.at(1)});
  converted_path2.insert(converted_path2.begin(), {start_shelfino2.at(0), start_shelfino2.at(1)});

  nav_msgs::msg::Path shelfino1_nav2, shelfino2_nav2;

  shelfino1_nav2 = convertDubinsPathToNavPath(shelfino1_d_path);
  shelfino2_nav2 = convertDubinsPathToNavPath(shelfino2_d_path);

  auto obstacles = m->get_obstacles();
  //  genearateMovementPath(shelfino1_nav2, shelfino2_nav2, points_path1, points_path2, pose_shellfino1, pose_shellfino2, pose_gate, map);

  // Check collisions between the two paths
  RCLCPP_INFO(m->get_logger(), "\033[1;32m Collision checking \033[0m");

  bool solved_collision = false;

  // Calculate distances
  double shelfino1_distance = calculate_distance(pose_shellfino1, obstacles);
  double shelfino2_distance = calculate_distance(pose_shellfino2, obstacles);

  // Scoring logic
  double shelfino1_score = shelfino1_distance > 1.0 ? shelfino1_distance * 10 : 0; // Example scoring rule
  double shelfino2_score = shelfino2_distance > 1.0 ? shelfino2_distance * 10 : 0;

  while (!solved_collision)
  {
    int collision_index = checkIntersection(shelfino1_nav2, shelfino2_nav2);
    if (collision_index != -1)
    {
      RCLCPP_INFO(m->get_logger(), "\033[1;33m Found a collision \033[0m");
      // get the collision point as a KDNode_t
      KDNode_t collision_point = {shelfino1_nav2.poses.at(collision_index).pose.position.x, shelfino1_nav2.poses.at(collision_index).pose.position.y};

      // who has the largest score has to deviate the path
      if (shelfino1_score < shelfino2_score)
      {
        converted_path1 = reschedule_path(converted_path1, start_shelfino1, map);
        converted_path1.erase(converted_path1.begin());
        converted_path1.erase(converted_path1.end());
        try
        {
          shelfino1_d_path = d.dubins_multi_point(start_shelfino1.at(0), start_shelfino1.at(1), m->get_pose1().at(2), goal.at(0), goal.at(1), m->get_gate().at(2), converted_path1, 3, map);
        }
        catch (const std::exception &e)
        {
          RCLCPP_ERROR(m->get_logger(), "Error in Dubins");
          continue;
        }
        shelfino1_nav2 = convertDubinsPathToNavPath(shelfino1_d_path);
        converted_path1.push_back({goal.at(0), goal.at(1)});
        converted_path1.insert(converted_path1.begin(), {start_shelfino1.at(0), start_shelfino1.at(1)});
      }
      else
      {
        // reschedule for shelfino 2
        converted_path2 = reschedule_path(converted_path2, start_shelfino2, map);
        converted_path2.erase(converted_path2.begin());
        converted_path2.erase(converted_path2.end());
        try
        {
          shelfino2_d_path = d.dubins_multi_point(start_shelfino2.at(0), start_shelfino2.at(1), m->get_pose2().at(2), goal.at(0), goal.at(1), m->get_gate().at(2), converted_path1, 3, map);
        }
        catch (const std::exception &e)
        {
          RCLCPP_ERROR(m->get_logger(), "Error in Dubins");
          continue;
        }
        shelfino2_nav2 = convertDubinsPathToNavPath(shelfino2_d_path);
        converted_path2.push_back({goal.at(0), goal.at(1)});
        converted_path2.insert(converted_path2.begin(), {start_shelfino2.at(0), start_shelfino2.at(1)});
      }
    }
    else
    {
      solved_collision = true;
      RCLCPP_INFO(m->get_logger(), "\033[1;32m No collisions \033[0m");
    }
  }


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

  //-----------------------------------------------------------------------
  std::vector<line_t>
      lines_path1;
  for (size_t i = 0; i < points_path1.size() - 1; i++)
  {
    lines_path1.emplace_back(line_t({points_path1[i], points_path1[i + 1]}));
  }
  std::vector<line_t>
      lines_path2;
  for (size_t i = 0; i < points_path2.size() - 1; i++)
  {
    lines_path2.emplace_back(line_t({points_path2[i], points_path2[i + 1]}));
  }

  auto dubins_publisher = std::make_shared<PointMarkerNode>();
  dubins_publisher->send_nav2(shelfino1_nav2, shelfino2_nav2);

  auto cleaned_obstacles_publisher = std::make_shared<MapEdgePublisherNode>(cleaned_obstacles, "cleaned_obstacles");
  multi_polygon_t pollo;
  pollo.emplace_back(inflated_border);
  auto inflated_publisher = std::make_shared<MapEdgePublisherNode>(inflated_obstacles, "inflated_map_border");
  auto raw_obstacles_publisher = std::make_shared<MapEdgePublisherNode>(raw_obstacles, "raw_obstacles");
  auto cutted_obstacles_publisher = std::make_shared<MapEdgePublisherNode>(cutted_obstacles, "cutted_obstacles");
  auto inflated_border_publisher = std::make_shared<MapEdgePublisherNode>(pollo, "cleaned_border");

  auto edges_between_obstacles_publisher = std::make_shared<SimpleEdgePublisherNode>(edges_between_obstacles, "edges_between_obstacles");
  auto gate_publisher = std::make_shared<SimpleEdgePublisherNode>(gate_edges_to_map, "gate_edges");
  auto map_in_lines_publisher = std::make_shared<SimpleEdgePublisherNode>(map_in_lines, "map_in_lines_graph");
  auto point_to_map_publisher = std::make_shared<SimpleEdgePublisherNode>(shellfino1_edges_to_map, "point_to_map_edges");
  auto path_1_publisher = std::make_shared<SimpleEdgePublisherNode>(lines_path1, "path_1");
  auto path_2_publisher = std::make_shared<SimpleEdgePublisherNode>(lines_path2, "path_2");

  auto point_publisher = std::make_shared<SimpleEdgePublisherNode>(cuts, "cuts_publisher");
  log_message(logger_class, log_level, "\033[1;32m Start publishing \033[0m");

  while (true)
  {
    rclcpp::spin_some(dubins_publisher);
    rclcpp::spin_some(inflated_border_publisher);

    rclcpp::spin_some(raw_obstacles_publisher);
    rclcpp::spin_some(cleaned_obstacles_publisher);
    rclcpp::spin_some(inflated_publisher);
    rclcpp::spin_some(edges_between_obstacles_publisher);
    rclcpp::spin_some(gate_publisher);

    rclcpp::spin_some(point_publisher);
    rclcpp::spin_some(map_in_lines_publisher);
    rclcpp::spin_some(cutted_obstacles_publisher);
    rclcpp::spin_some(point_to_map_publisher);
    rclcpp::spin_some(path_1_publisher);
    rclcpp::spin_some(path_2_publisher);

    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  rclcpp::shutdown();
  return 0;
}
