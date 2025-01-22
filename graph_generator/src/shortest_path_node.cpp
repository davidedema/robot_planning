#include "graph_generator/combinatorial_based/utilities.hpp"
#include "graph_generator/combinatorial_based/graph_builder.hpp"
#include "graph_generator/combinatorial_based/map_construction.hpp"
#include "graph_generator/marker_publishers/map_edges_publisher.hpp"



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

  auto inflated_map = m->get_inflated_map();
  auto clean_map = m->get_clean_map();

  ss << "\033[1;32m Number of obstacles " << m->get_obstacles().size() << "\033[0m";
  sService = ss.str();
  RCLCPP_INFO(m->get_logger(), sService.c_str());
  ss.str(std::string());
  RCLCPP_INFO(m->get_logger(), "\033[1;32m Map built\033[0m");

  ss << "[1;32m  " << clean_map.size() << "inflated:" << inflated_map.size() << "[0m";
  sService = ss.str();
  RCLCPP_INFO(m->get_logger(), sService.c_str());
  ss.str(std::string());

  auto points = get_cut_points(clean_map, inflated_map);

  RCLCPP_INFO(m->get_logger(), "\033[1;32m Generating Graph\033[0m");

  auto remapped = convertMultiPolygon(inflated_map);

  ss << "\033[1;32m Number of obstacles " << remapped[0].inners().size() << "\033[0m";
  sService = ss.str();
  RCLCPP_INFO(m->get_logger(), sService.c_str());
  ss.str(std::string());

  std::vector<line_xy_t> line;
  line = find_bitangents(remapped);

  ss << "\033[1;32m Graph size " << line.size() << "\033[0m";
  sService = ss.str();
  RCLCPP_INFO(m->get_logger(), sService.c_str());
  ss.str(std::string());

  auto inflated_publisher = std::make_shared<MapEdgePublisherNode>(inflated_map, "inflated_map_edges");
  auto clean_publisher = std::make_shared<MapEdgePublisherNode>(clean_map, "clean_map_edges");

  auto graph_publisher = std::make_shared<SimpleEdgePublisherNode>(line, "bitangent_graph");
  auto point_publisher = std::make_shared<SimplePointPublisherNode>(points, "point_publisher");

  while (true)
  {
    rclcpp::spin_some(clean_publisher);
    rclcpp::spin_some(inflated_publisher);
    rclcpp::spin_some(graph_publisher);
    rclcpp::spin_some(point_publisher);
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  rclcpp::shutdown();
  return 0;
}
