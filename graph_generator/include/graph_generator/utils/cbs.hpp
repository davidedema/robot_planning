#pragma once

#include <vector>
#include <time.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

#include "graph_generator/sampling_based/utils/rrt.hpp"

typedef std::vector<KDNode_t> plan_t;

class CBS
{
  private:
    Graph g;
    std::map<KDNode_t, vertex_t> graph_lookup;
    std::map<KDNode_t, double> cost_lookup;
    std::map<KDNode_t, KDNode_t> parent_lookup;

    std::vector<KDNode_t> plan1;
    std::vector<KDNode_t> plan2;

  public:
    // Constructor
    CBS(Graph g, std::map<KDNode_t, vertex_t> gl);
    // Distructor
    ~CBS();


    // High level search
    void check_conflicts(plan_t planA, plan_t planB);


    // Low level search
    plan_t astar_search(KDNode_t &start, KDNode_t &goal);
    double get_total_cost(KDNode_t &node);
    double get_distance(KDNode_t &current, KDNode_t &child);
};