#include "graph_generator/combinatorial_based/utilities.hpp"



line_t get_points_at_distance(std::vector<line_t> inflated_shell, point_t inflated_center, point_t clean_center, double distance);
std::vector<line_t> get_cut_lines(multi_polygon_t clean, multi_polygon_t inflated, double tolerance);
std::vector<line_t> find_edges_between_obstacles(const multi_polygon_t &multipolygon);
std::vector<line_t> find_shellfino_map_links(const multi_polygon_t &multipolygon, const point_t point, const point_t goal_point);
std::vector<line_t> find_point_map_links(const multi_polygon_t &multipolygon, const point_t point);

std::vector<line_t> poly_to_lines(const multi_polygon_t &multipolygon, const polygon_t border);
multi_polygon_t apply_cuts_to_map(const multi_polygon_t &multipolygon, const std::vector<line_t> cut_lines);
void map_edge_difference(multi_polygon_t &obstacles, polygon_t &edge);
std::vector<line_t> find_border_point_map_links(const multi_polygon_t &multipolygon, const polygon_t &border_poly, const point_t point);
