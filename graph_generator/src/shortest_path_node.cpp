#include "graph_generator/combinatorial_based/utilities.hpp"
#include "graph_generator/combinatorial_based/map_utilities.hpp"
#include "graph_generator/combinatorial_based/map_construction.hpp"
#include "graph_generator/marker_publishers/map_edges_publisher.hpp"
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
  std::stringstream ss;
  std::string sService;

  rclcpp::init(argc, argv);

  auto m = std::make_shared<MapConstruction>();

  RCLCPP_INFO(m->get_logger(), "Waiting for obstacles, borders and gates...");
  ss.str(std::string());

  while (!m->obstacles_r_ || !m->borders_r_ || !m->pos1_r_)
  {
    rclcpp::spin_some(m->get_node_base_interface());
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(m->get_logger(), "\033[1;32m Map info obtained\033[0m");
  RCLCPP_INFO(m->get_logger(), "Building map");

  auto inflated_obstacles = m->get_inflated_map();
  auto inflated_obstacles_unionized = m->get_unionized_inflated_map();

  auto inflated_border = m->get_inflated_border();
  auto clean_map = m->get_clean_map();

  auto pose_shellfino1 = m->get_pose1();
  auto pose_shellfino2 = m->get_pose3();
  auto pose_gate = m->get_gate();

  point_t position_shellfino_1 = point_t({pose_shellfino1[0], pose_shellfino1[1]});
  point_t position_shellfino_2 = point_t({pose_shellfino2[0], pose_shellfino2[1]});
  point_t position_gate = point_t({pose_gate[0], pose_gate[1]});

  ss
      << "\033[1;32m Number of obstacles " << m->get_obstacles().size() << "\033[0m";
  sService = ss.str();
  RCLCPP_INFO(m->get_logger(), sService.c_str());
  ss.str(std::string());
  RCLCPP_INFO(m->get_logger(), "\033[1;32m Map built\033[0m");

  ss << "clean map objects: " << clean_map.size() << "inflated objects:" << inflated_obstacles.size() << "";
  sService = ss.str();
  RCLCPP_INFO(m->get_logger(), sService.c_str());
  ss.str(std::string());
  RCLCPP_INFO(m->get_logger(), "\033[1;32m Cutting edges \033[0m");

  auto cuts = get_cut_lines(clean_map, inflated_obstacles, SHELFINO_INFLATION);

  RCLCPP_INFO(m->get_logger(), "\033[1;32m Generating Graph\033[0m");

  auto cutted_map_poly = cutted_map(inflated_obstacles_unionized, cuts);

  // MAP CONVERSION
  auto map_in_lines = map_to_lines(cutted_map_poly, inflated_border);

  auto shellfino1_to_map = find_shellfino_map_links(inflated_obstacles_unionized, position_shellfino_1, position_gate);
  auto shellfino2_to_map = find_shellfino_map_links(inflated_obstacles_unionized, position_shellfino_2, position_gate);
  auto gate_to_map = find_point_map_links(inflated_obstacles_unionized, position_gate);
  RCLCPP_INFO(m->get_logger(), "\033[1;32m Converted to lines\033[0m");

  std::vector<line_t>
      bitangents_lines = find_links(cutted_map_poly);

  ss << "\033[1;32m Graph size " << bitangents_lines.size() << "\033[0m";
  sService = ss.str();
  RCLCPP_INFO(m->get_logger(), sService.c_str());
  ss.str(std::string());

  auto graph_generator = ShortestGraph(shellfino1_to_map, shellfino2_to_map, gate_to_map, map_in_lines, position_shellfino_1, position_shellfino_2, position_gate);
  auto points_path1 = graph_generator.get_shellfino_1_path();
  std::vector<line_t> lines_path1;
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

  while (true)
  {
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
