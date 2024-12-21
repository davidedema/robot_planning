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

struct node
{
  KDNode_t node;
  std::vector<KDNode_t> parents;
  std::vector<KDNode_t> childs;
  double cost;
  double l2_dist_h;
};

// Define the graph type
using Graph = boost::adjacency_list<
    boost::vecS,        // Store edges in a vector
    boost::vecS,        // Store vertices in a vector
    boost::undirectedS, // Undirected graph
    struct node         // Vertex properties
    >;

typedef boost::graph_traits<Graph>::vertex_descriptor vertex_t;

class RRT
{

private:
  uint samples = 100;
  double d_max = 0.5;
  double d = 0.25;
  KDNode_t start;
  KDNode_t goal;
  std::map<KDNode_t, vertex_t> graph_lookup;
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
  Graph get_graph();
  std::map<KDNode_t, vertex_t> get_lookup();
  void set_problem(KDNode_t &start, KDNode_t &goal);
  KDNode_t get_random_point(int index, boost::geometry::model::multi_polygon<polygon_t> &map);
  KDNode_t next_point(KDNode_t &sampled_point, KDNode_t &nearest, boost::geometry::model::multi_polygon<polygon_t> &map);
  KDNode_t get_best_neighbor(KDNode_t &new_point, KDNode_t &old_neigh, double range, boost::geometry::model::multi_polygon<polygon_t> &map);
  void rewire(KDNode_t &new_point, double range, boost::geometry::model::multi_polygon<polygon_t> &map);
  KDNode_t get_nn(KDNode_t &sampled_point, int n_k = 1);
  void add_kd_node(KDNode_t &node);
  bool are_nodes_equal(const KDNode_t &a, const KDNode_t &b);
  bool add_edge(KDNode_t &new_node, KDNode_t &nearest_node, boost::geometry::model::multi_polygon<polygon_t> &map);
  bool add_node(KDNode_t &new_node);
  bool valid_point(KDNode_t &result, boost::geometry::model::multi_polygon<polygon_t> &map);
  bool valid_segment(KDNode_t &start, KDNode_t &end, boost::geometry::model::multi_polygon<polygon_t> &map);
  bool is_goal(KDNode_t &point);
  void create_inflated_polygon(const point_t &p1, const point_t &p2, double epsilon, polygon_t &polygon);
  std::vector<KDNode_t> get_path(KDNode_t &start);
  std::vector<KDNode_t> smooth_path(std::vector<KDNode_t> &path, boost::geometry::model::multi_polygon<polygon_t> &map);
};