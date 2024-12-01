#pragma once

#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <time.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

#include "graph_generator/sampling_based/utils/kdtree.hpp"

typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_t;
typedef boost::geometry::model::polygon<point_t> polygon_t;
typedef boost::geometry::model::segment<point_t> segment_t;
typedef std::vector<double> KDNode_t;

// Define the graph type
using Graph = boost::adjacency_list<
    boost::vecS,        // Store edges in a vector
    boost::vecS,        // Store vertices in a vector
    boost::undirectedS, // Undirected graph
    KDNode_t            // Vertex properties
    >;

class RRT
{

private:
  uint samples = 100;
  double d_max = 0.5;
  double d = 0.25;
  Graph g;

public:
  // KDTREE things
  Kdtree::KdNodeVector nodes;

  std::vector<KDNode_t> rrt_tree;

  // constructor
  RRT();
  // distructor
  ~RRT();

  // main functions
  KDNode_t get_random_point(int index, polygon_t &map);
  KDNode_t next_point(KDNode_t &sampled_point, KDNode_t &nearest, polygon_t &map);
  KDNode_t get_parent(KDNode_t &child);
  KDNode_t get_nn(KDNode_t &sampled_point, int n_k = 1);
  void add_kd_node(KDNode_t &node);
  Graph::vertex_descriptor find_vertex_by_node(const Graph &g, const KDNode_t &node);
  bool are_nodes_equal(const KDNode_t &a, const KDNode_t &b);
  bool add_edge(KDNode_t &new_node, KDNode_t &nearest_node, polygon_t &map);
  bool add_node(KDNode_t &new_node);
  bool valid_point(KDNode_t &result, polygon_t &map);
  bool valid_segment(KDNode_t &start, KDNode_t &end, polygon_t &map);
  void create_inflated_polygon(const point_t &p1, const point_t &p2, double epsilon, polygon_t &polygon);
};