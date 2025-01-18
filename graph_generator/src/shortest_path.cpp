#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <graph_generator/marker_publishers/map_edges_publisher.hpp>
#include "graph_generator/combinatorial_based/map_construction.hpp"

namespace bg = boost::geometry;

using multi_linestring_type = boost::geometry::model::multi_linestring<line_xy_t>;

/////////////////////// GRAPH_BUILDER

// Function to check tangency

bool is_tangent(const line_xy_t &line, const polygon_xy_t &border, const multipolygon_xy_t &inner_polygons)
{
  // Multi-point to collect intersections
  multi_linestring_type intersections;

  // Check intersection with the outer border
  bg::intersection(line, border, intersections);

  // If the line is tangent to the outer border, it must touch it exactly twice
  if (intersections.size() != 2)
  {
    return false;
  }

  // Verify that the line does not pass through the interior of the inner polygons
  for (const auto &polygon : inner_polygons)
  {
    multi_linestring_type inner_intersections;
    bg::intersection(line, polygon, inner_intersections);

    // If the line intersects any inner polygon at more than two points, it's not tangent
    if (!inner_intersections.empty())
    {
      return false;
    }
  }

  return true;
}

// Function to find bitangent lines
std::vector<line_xy_t> find_bitangents(const multipolygon_xy_t &multipolygon)
{
  std::vector<line_xy_t> bitangents;
  auto inner_rings = multipolygon[0].inners();

  for (const auto &ring1 : inner_rings)
  {
    if (!bg::is_valid(ring1))
    {
      std::cerr << "The ring is not valid!" << std::endl;
    }
    for (const auto &ring2 : inner_rings)
    {
      if (!bg::is_valid(ring2))
      {
        std::cerr << "The ring is not valid!" << std::endl;
      }
      if (&ring1 != &ring2)
      {
        for (const auto point1 : ring1)
        {
          for (const auto point2 : ring2)
          {
            line_xy_t line = line_xy_t{point1, point2};

            bg::model::multi_point<point_xy_t> intersections;
            bool unwanted_intersection = false;
            
            for (auto &other_ring : inner_rings)
            {
              if (&ring1 != &other_ring && &ring2 != &other_ring)
                for (size_t i = 0; i < other_ring.size() - 1 && !unwanted_intersection; ++i) // check if there is internal intersection
                {
                  line_xy_t line_v = line_xy_t{other_ring[i], other_ring[i + 1]};
                  if (bg::intersects(line, line_v))
                  {
                    unwanted_intersection = true;
                  }
                }
            }

            for (size_t i = 0; i < ring1.size() - 1 && !unwanted_intersection; ++i) // check if there is internal intersection
            {
              line_xy_t line_v = line_xy_t{ring1[i], ring1[i + 1]};
              bool to_ignore1 = (bg::equals(ring1[i], point1) || bg::equals(ring1[i + 1], point1));
              bool to_ignore2 = (bg::equals(ring1[i], point2) || bg::equals(ring1[i + 1], point2));
              if (!to_ignore1 && !to_ignore2 && bg::intersects(line, line_v))
              {
                unwanted_intersection = true;
              }
            }

            for (size_t i = 0; i < ring2.size() - 1 && !unwanted_intersection; ++i)
            {
              line_xy_t line_v = line_xy_t{ring2[i], ring2[i + 1]}; // Create a segment for the edge
              bool to_ignore1 = (bg::equals(ring2[i], point1) || bg::equals(ring2[i + 1], point1));
              bool to_ignore2 = (bg::equals(ring2[i], point2) || bg::equals(ring2[i + 1], point2));
              if (!to_ignore1 && !to_ignore2 && bg::intersects(line, line_v))
              {
                unwanted_intersection = true;
              }
            }

            if (!unwanted_intersection)
              bitangents.emplace_back(line);
          }
        }
      }
    }
  }

  return bitangents;
}

polygon_xy_t convertPolygon(const polygon_t &polygon)
{
  polygon_xy_t polygon_xy;

  // Convert the outer ring
  const auto &outer_ring = polygon.outer();
  auto &outer_ring_xy = polygon_xy.outer();
  for (const auto &point : outer_ring)
  {
    outer_ring_xy.emplace_back(point.get<0>(), point.get<1>());
  }

  // Convert the inner rings
  const auto &inner_rings = polygon.inners();
  for (const auto &inner_ring : inner_rings)
  {
    typename polygon_xy_t::ring_type inner_ring_xy;
    for (const auto &point : inner_ring)
    {
      inner_ring_xy.emplace_back(point.get<0>(), point.get<1>());
    }
    polygon_xy.inners().emplace_back(std::move(inner_ring_xy));
  }

  return polygon_xy;
}

multipolygon_xy_t convertMultiPolygon(const multi_polygon_t &multipolygon)
{
  multipolygon_xy_t multipolygon_xy;
  for (const auto &polygon : multipolygon)
  {
    multipolygon_xy.emplace_back(convertPolygon(polygon));
  }

  return multipolygon_xy;
}

int main(int argc, char **argv)
{
  std::stringstream ss;
  std::string sService;

  rclcpp::init(argc, argv);

  auto m = std::make_shared<MapConstruction>();

  RCLCPP_INFO(m->get_logger(), "Waiting for obstacles, borders and gates...");
  while (!m->obstacles_r_ || !m->borders_r_ || !m->pos1_r_)
  {
    rclcpp::spin_some(m->get_node_base_interface());
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(m->get_logger(), "\033[1;32m Map info obtained\033[0m");
  RCLCPP_INFO(m->get_logger(), "Building map");
  auto map = m->get_map();

  ss << "\033[1;32m Number of obstacles " << m->get_obstacles().size() << "\033[0m";
  sService = ss.str();
  RCLCPP_INFO(m->get_logger(), sService.c_str());

  RCLCPP_INFO(m->get_logger(),"\033[1;32m Map built\033[0m");
  rclcpp::spin(std::make_shared<MapEdgePublisherNode>(map, "map_edges"));

  RCLCPP_INFO(m->get_logger(), "\033[1;32m Generating Graph\033[0m");

  auto remapped = convertMultiPolygon(map);


  ss << "\033[1;32m Graph size " << remapped[0].inners().size() << "\033[0m";

  sService = ss.str();
  RCLCPP_INFO(m->get_logger(), sService.c_str());

  std::vector<line_xy_t> line;
  line = find_bitangents(remapped);
  rclcpp::spin(std::make_shared<SimpleEdgePublisherNode>(line, "bitangent_graph"));

  rclcpp::shutdown();
  return 0;
}
