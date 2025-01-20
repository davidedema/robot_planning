#include "graph_generator/utils/cbs.hpp"

CBS::CBS(Graph g, std::map<KDNode_t, vertex_t> gl)
{
  this->g = g;
  this->graph_lookup = gl;
}

CBS::~CBS() {}

void CBS::check_conflicts(plan_t planA, plan_t planB)
{
}

double CBS::get_total_cost(KDNode_t &node)
{
  return cost_lookup[node] + g[graph_lookup[node]].l2_dist_h;
}

double CBS::get_distance(KDNode_t &current, KDNode_t &child)
{
  return sqrt(pow(current.at(0) - child.at(0), 2) + pow(current.at(1) - child.at(1), 2));
}

plan_t CBS::astar_search(KDNode_t &start, KDNode_t &goal)
{
  plan_t plan;

  std::vector<KDNode_t> open;                           // open list
  std::vector<KDNode_t> close;                          // closed list

  // std::cout << "THE GOAL SPECIFIED IS: " << goal.at(0) << "  " << goal.at(1) << std::endl;

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
      // std::cout << "Possible node: ";
      // std::cout << node.at(0) << "  " << node.at(1);
      // std::cout << " with total cost: " << this->get_total_cost(node) << std::endl;
      if (this->get_total_cost(node) <= this->get_total_cost(current))
      {
        current = node;
        current_it = it;
      }
    }
    // std::cout << "\tCURRENT NODE IS: ";
    // std::cout << current.at(0) << "  " << current.at(1) << std::endl;
    // std::cout << "\tWITH TOTAL COST: " << this->get_total_cost(current) << std::endl;
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
      // std::cout << "\t\t CHILD NODE IS: ";
      // std::cout << "\t\t" << child.at(0) << "  " << child.at(1) << std::endl;
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
