#include "graph_generator/combinatorial_based/utilities.hpp"
#include "graph_generator/combinatorial_based/map_utilities.hpp"
#include "graph_generator/combinatorial_based/map_construction.hpp"
#include "graph_generator/marker_publishers/map_edges_publisher.hpp"
#include "graph_generator/marker_publishers/PointMarker.hpp"

#include "graph_generator/combinatorial_based/path_utilities.hpp"
#include "graph_generator/combinatorial_based/shortest_graph.hpp"

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

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto m = std::make_shared<MapConstruction>();
  auto log_level = rclcpp::Logger::Level::Info;

  auto logger_class = m->get_logger();
  // Map for nav2---------------------------------------------------
  auto mappa = std::make_shared<GraphGenerator>();
  log_message(logger_class, log_level, "Waiting for obstacles Map for nav2...");

  while (!mappa->obstacles_r_ || !mappa->borders_r_ || !mappa->gates_r_ || !mappa->pos1_r_ || !mappa->pos3_r_)
  {
    rclcpp::spin_some(mappa->get_node_base_interface());
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  auto map = mappa->get_map();

  // Map for shortest_path---------------------------------------------------

  log_message(logger_class, log_level, "Waiting for obstacles Map for shortest_path");
  while (!m->obstacles_r_ || !m->borders_r_ || !m->gates_r_ || !m->pos1_r_ || !m->pos3_r_)
  {
    rclcpp::spin_some(m->get_node_base_interface());
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  auto clean_map = m->get_clean_map();
  auto pose_shellfino1 = m->get_pose1();
  auto pose_shellfino2 = m->get_pose3();
  auto pose_gate = m->get_gate();
  log_message(logger_class, log_level, "\033[1;32m Map info obtained\033[0m \n Building map");

  auto inflated_obstacles = m->get_inflated_map();
  auto inflated_border = m->get_inflated_border();
  auto inflated_obstacles_unionized = m->get_unionized_inflated_map();
  map_edge_difference(inflated_obstacles_unionized, inflated_border);
  log_message(logger_class, log_level, "\033[1;32m Inflated Maps Building map");

  point_t position_shellfino_1 = point_t({pose_shellfino1[0], pose_shellfino1[1]});
  point_t position_shellfino_2 = point_t({pose_shellfino2[0], pose_shellfino2[1]});
  point_t position_gate = point_t({pose_gate[0], pose_gate[1]});

  log_message(logger_class, log_level, "\033[1;32m Number of obstacles", m->get_obstacles().size(), "\033[0m");
  log_message(logger_class, log_level, "\033[1;32m Map built\033[0m");
  log_message(logger_class, log_level, "clean map objects : ", clean_map.size(), " inflated objects : ", inflated_obstacles.size());
  log_message(logger_class, log_level, "\033[1;32m Cutting edges \033[0m");

  // Edge Calculations----------------------------------------------------------------
  auto cuts = get_cut_lines(clean_map, inflated_obstacles, SHELFINO_INFLATION);
  log_message(logger_class, log_level, "\033[1;32m Generating Graph \033[0m");

  auto cutted_map_poly = cutted_map(inflated_obstacles_unionized, cuts);
  multi_polygon_t map_poly;
  for (auto poly : cutted_map_poly)
  {
    map_poly.emplace_back(poly);
  }
  //map_poly.emplace_back(inflated_border);



  auto map_in_lines = map_to_lines(map_poly, inflated_border);



  auto shellfino1_to_map = find_shellfino_map_links(inflated_obstacles_unionized, position_shellfino_1, position_gate);
  auto shellfino2_to_map = find_shellfino_map_links(inflated_obstacles_unionized, position_shellfino_2, position_gate);
  auto gate_to_map = find_point_map_links(inflated_obstacles_unionized, position_gate);



  std::vector<line_t>
      bitangents_lines = find_links(cutted_map_poly);

  log_message(logger_class, log_level, "\033[1;32m Number of edges", bitangents_lines.size(), "\033[0m");

  // Graph Search------------------------------------------------------------------------------
  auto graph_generator = ShortestGraph(shellfino1_to_map, shellfino2_to_map, gate_to_map, map_in_lines, position_shellfino_1, position_shellfino_2, position_gate);
  auto points_path1 = graph_generator.get_shellfino_1_path();
  auto points_path2 = graph_generator.get_shellfino_2_path();

  log_message(logger_class, log_level, "\033[1;32m Dubinizing \033[0m");

  // From optimal theoretical path -> physical movement---------------------------------------------------------------------------

  nav_msgs::msg::Path shelfino1_nav2, shelfino2_nav2;
  genearateMovementPath(shelfino1_nav2, shelfino2_nav2, points_path1, points_path2, pose_shellfino1, pose_shellfino2, pose_gate, map);

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

  auto inflated_unionized_publisher = std::make_shared<MapEdgePublisherNode>(inflated_obstacles_unionized, "inflated_unionized_map_edges");
  multi_polygon_t pollo;
  pollo.emplace_back(inflated_border);
  auto inflated_publisher = std::make_shared<MapEdgePublisherNode>(inflated_obstacles, "inflated_map_edges");
  auto clean_publisher = std::make_shared<MapEdgePublisherNode>(clean_map, "clean_map_edges");
  auto cutted_map_publisher = std::make_shared<MapEdgePublisherNode>(cutted_map_poly, "cutted_map_edges");
  auto inflated_border_publisher = std::make_shared<MapEdgePublisherNode>(pollo, "inflated_border");

  auto graph_publisher = std::make_shared<SimpleEdgePublisherNode>(bitangents_lines, "bitangent_graph");
  auto map_in_lines_publisher = std::make_shared<SimpleEdgePublisherNode>(map_in_lines, "map_in_lines_graph");
  auto point_to_map_publisher = std::make_shared<SimpleEdgePublisherNode>(shellfino1_to_map, "point_to_map_edges");
  auto path_1_publisher = std::make_shared<SimpleEdgePublisherNode>(lines_path1, "path_1_graph");
  auto path_2_publisher = std::make_shared<SimpleEdgePublisherNode>(lines_path2, "path_2_graph");

  auto point_publisher = std::make_shared<SimpleEdgePublisherNode>(cuts, "cuts_publisher");
  log_message(logger_class, log_level, "\033[1;32m Start publishing \033[0m");

  while (true)
  {
    rclcpp::spin_some(dubins_publisher);
    rclcpp::spin_some(inflated_border_publisher);

    rclcpp::spin_some(clean_publisher);
    rclcpp::spin_some(inflated_unionized_publisher);
    rclcpp::spin_some(inflated_publisher);
    rclcpp::spin_some(graph_publisher);
    rclcpp::spin_some(point_publisher);
    rclcpp::spin_some(map_in_lines_publisher);
    rclcpp::spin_some(cutted_map_publisher);
    rclcpp::spin_some(point_to_map_publisher);
    rclcpp::spin_some(path_1_publisher);
    rclcpp::spin_some(path_2_publisher);

    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  rclcpp::shutdown();
  return 0;
}
