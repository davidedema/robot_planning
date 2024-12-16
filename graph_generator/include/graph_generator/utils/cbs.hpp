#pragma once

#include <vector>
#include <time.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

typedef std::vector<double> KDNode_t;

struct node
{
  KDNode_t node;
  std::vector<KDNode_t> parents;
  double cost;
};

// Define the graph type
using Graph = boost::adjacency_list<
    boost::vecS,        // Store edges in a vector
    boost::vecS,        // Store vertices in a vector
    boost::undirectedS, // Undirected graph
    struct node         // Vertex properties
    >;

typedef boost::graph_traits<Graph>::vertex_descriptor vertex_t;

class CBS
{
  private:
    Graph g;
    std::map<KDNode_t, vertex_t> graph_lookup;

    std::vector<KDNode_t> plan1;
    std::vector<KDNode_t> plan2;

  public:
    // Constructor
    CBS();
    // Distructor
    ~CBS();


    // High level search

    // Low level search
    std::vector<KDNode_t> astar_search(KDNode_t &start, KDNode_t &goal)
}