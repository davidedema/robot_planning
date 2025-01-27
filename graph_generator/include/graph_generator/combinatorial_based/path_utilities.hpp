
#include "graph_generator/combinatorial_based/utilities.hpp"

nav_msgs::msg::Path convertDubinsPathToNavPath(const std::vector<dubins_curve> &dubins_curves);
size_t checkIntersection(const nav_msgs::msg::Path &path1, const nav_msgs::msg::Path &path2);
double compute_score(const nav_msgs::msg::Path &path, size_t collision_point);
std::vector<KDNode_t> reschedule_path(std::vector<KDNode_t> path, KDNode_t collision_point, double step_size);
void genearateMovementPath(nav_msgs::msg::Path &shelfino1_nav2, nav_msgs::msg::Path &shelfino2_nav2, std::vector<point_t> points_path1, std::vector<point_t> points_path2, pose_t pose_shellfino1, pose_t pose_shellfino2, pose_t pose_gate, multi_polygon_t map);
