#include "graph_generator/combinatorial_based/utilities.hpp"



std::vector<point_t> get_points_at_distance(std::vector<line_t> inflated_shell, point_t inflated_center, point_t clean_center, double distance);
std::vector<point_t> get_cut_points(multi_polygon_t clean, multi_polygon_t inflated);
std::vector<line_xy_t> find_bitangents(const multipolygon_xy_t &multipolygon);
