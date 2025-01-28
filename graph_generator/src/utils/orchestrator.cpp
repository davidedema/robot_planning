#include "graph_generator/utils/orchestrator.hpp"

Orchestrator::Orchestrator(Graph g, std::map<KDNode_t, vertex_t> gl)
{
  this->g = g;
  this->graph_lookup = gl;
}

Orchestrator::~Orchestrator() {}

double Orchestrator::get_total_cost(KDNode_t &node)
{
  return cost_lookup[node] + g[graph_lookup[node]].l2_dist_h;
}

double Orchestrator::get_distance(KDNode_t &current, KDNode_t &child)
{
  return sqrt(pow(current.at(0) - child.at(0), 2) + pow(current.at(1) - child.at(1), 2));
}

plan_t Orchestrator::astar_search(KDNode_t &start, KDNode_t &goal)
{
  plan_t plan;

  std::vector<KDNode_t> open;  // open list
  std::vector<KDNode_t> close; // closed list

  open.push_back(start); // init open list
  cost_lookup[start] = 0;

  // A* search loop
  while (!open.empty())
  {
    // Take node with least f from open list
    auto current_it = open.begin();
    auto current = *current_it;
    for (auto it = open.begin(); it != open.end(); it++)
    {
      auto node = *it;
      if (this->get_total_cost(node) <= this->get_total_cost(current))
      {
        current = node;
        current_it = it;
      }
    }
    // If current is goal, reconstruct path and return
    if (current == goal)
    {
      // Reconstruct path from goal to start
      while (current != start)
      {
        plan.push_back(current);
        current = parent_lookup[current];
      }
      plan.push_back(start);                  // Add the start node
      std::reverse(plan.begin(), plan.end()); // Reverse the path to go from start to goal
      return plan;
    }

    // Add current to closed list and remove from open list
    close.push_back(current);
    open.erase(current_it);

    // Process each child of the current node
    for (auto child_it = g[graph_lookup[current]].childs.begin();
         child_it != g[graph_lookup[current]].childs.end();
         child_it++)
    {
      auto child = *child_it;
      // Skip if child is in the closed list
      if (std::find(close.begin(), close.end(), child) != close.end())
      {
        continue;
      }

      // Calculate the cost to the child
      double cost = cost_lookup[current] + this->get_distance(current, child);

      // If child is in open list with higher cost, update its cost and parent
      if (std::find(open.begin(), open.end(), child) != open.end() && cost < cost_lookup[child])
      {
        cost_lookup[child] = cost;
        parent_lookup[child] = current;
        continue;
      }

      // If child is neither in open nor closed, add it to open
      if (std::find(close.begin(), close.end(), child) == close.end() &&
          std::find(open.begin(), open.end(), child) == open.end())
      {
        open.push_back(child);
        cost_lookup[child] = cost;
        parent_lookup[child] = current;
      }
    }
  }

  // If we reach here, no path was found
  return plan; // Empty plan
}

size_t Orchestrator::checkIntersection(const nav_msgs::msg::Path &path1, const nav_msgs::msg::Path &path2)
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

std::vector<KDNode_t> Orchestrator::reschedule_path(std::vector<KDNode_t> path, KDNode_t collision_point, double step_size)
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
    // new_path.insert(new_path.begin() + 1, collision_point);
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
    // new_path.insert(new_path.begin() + 1, collision_point);
    new_path.insert(new_path.begin() + 1, new_point);
  }

  return new_path;
}