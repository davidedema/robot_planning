#include "graph_generator/sampling_based/utils/rrt.hpp"

RRT::RRT(KDNode_t &init)
{
  this->rrt_tree.push_back(init);
}

RRT::~RRT() {}

KDNode_t RRT::get_random_point(int index, polygon_t &map)
{
  // generate Halton sequence value for a given index and base
  KDNode_t sampled_point(2);
  double result, f = 0;
  int j = 0;
  std::vector<int> bases = {2, 3};
  bool valid = false;
  while (!valid)
  {
    for (uint i = 0; i < 2; i++)
    {
      result = f = j = 0;
      f = 1 / bases.at(i);
      j = index;
      while (j > 0)
      {
        result += f * (j % bases.at(i));
        j = std::floor(j / bases.at(i));
        f /= bases.at(i);
      }
      sampled_point.push_back(result);
    }

    if (this->valid_point(sampled_point, map))
    {
      valid = true;
    }
  }
  return sampled_point;
}

KDNode_t RRT::next_point(KDNode_t &sampled_point, KDNode_t &nearest)
{
  return {0.0};
}
KDNode_t RRT::get_parent(KDNode_t &child)
{
  return {0.0};
}
void RRT::add_node(KDNode_t &new_node, KDNode_t &nearest_node)
{
}

bool RRT::valid_point(KDNode_t &sampled_point, polygon_t &map)
{
  point_t aux_point;
  aux_point.x(sampled_point.at(0));
  aux_point.1(sampled_point.at(1));

  return boost::geometry::within(aux_point, map);
}