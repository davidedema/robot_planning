#pragma once

#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "graph_generator/sampling_based/utils/kdtree.hpp"

typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_t;
typedef boost::geometry::model::polygon<point_t> polygon_t;
typedef std::vector<double> KDNode_t;

class RRT{
  
  private:

    uint samples = 100;

  public:

    std::vector<KDNode_t> rrt_tree;

    // constructor
    RRT(KDNode_t &init);
    // distructor
    ~RRT();

    // main functions
    KDNode_t get_random_point(int index, polygon_t &map);
    KDNode_t next_point(KDNode_t &sampled_point, KDNode_t &nearest);
    KDNode_t get_parent(KDNode_t &child);
    void add_node(KDNode_t &new_node, KDNode_t &nearest_node);
    bool valid_point(KDNode_t &result, polygon_t &map);

};  