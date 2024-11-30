#include "graph_generator/sampling_based/utils/rrt.hpp"

RRT::RRT()
{
  // initialize the tree
}

RRT::~RRT() {}

KDNode_t RRT::get_random_point(int index, polygon_t &map)
{
  // generate Halton sequence value for a given index and base
  KDNode_t sampled_point(2);
  double result = 0.0, f = 0.0;
  std::vector<int> bases = {2, 3};
  bool valid = false;

  while (!valid)
  {
    sampled_point.clear(); // Ensure sampled_point is reset for each attempt
    for (size_t i = 0; i < 2; i++)
    {
      result = 0.0; // Reset result for each base
      f = 1.0 / bases.at(i);
      int j = index; // Initialize j with the index
      while (j > 0)
      {
        result += f * (j % bases.at(i));
        j = std::floor(j / bases.at(i));
        f /= bases.at(i);
      }
      result = result * 20 - 10;
      sampled_point.push_back(result);
    }

    if (this->valid_point(sampled_point, map))
    {
      valid = true;
    }
    else
    {
      sampled_point.clear();
      std::cout
          << "Sample not valid!\n";
      sampled_point.push_back(-1000);
      sampled_point.push_back(-1000);
      return sampled_point;
    }
  }
  return sampled_point;
}

KDNode_t RRT::next_point(KDNode_t &sampled_point, KDNode_t &nearest, polygon_t &map)
{
  // from sampled point and nearest point get the candidate point
  // check if it exceedes d_max
  float distance = sqrt(pow(sampled_point.at(0) - nearest.at(0), 2) + pow(sampled_point.at(1) - nearest.at(1), 2));
  if (distance < d_max)
  {
    return sampled_point;
  }
  // distance is > d_max get a point in between
  // find direction of the point
  KDNode_t direction = {sampled_point.at(0) - nearest.at(0),
                        sampled_point.at(1) - nearest.at(1)};
  // normalization factor
  double magnitude = sqrt(pow(direction.at(0), 2) + pow(direction.at(1), 2));
  // normalize direction
  direction.at(0) = direction.at(0) / magnitude;
  direction.at(1) = direction.at(1) / magnitude;
  // find offset
  direction.at(0) = direction.at(0) * d;
  direction.at(1) = direction.at(1) * d;
  KDNode_t new_point = {sampled_point.at(0) + direction.at(0), sampled_point.at(1) + direction.at(1)};
  // if it is valid
  if (this->valid_point(new_point, map))
  {
    return new_point;
  }
  std::cout << "Point not valid" << std::endl;
  return {-1000, -1000};
}

KDNode_t RRT::get_parent(KDNode_t &child)
{
  return {0.0};
}

Graph::vertex_descriptor RRT::find_vertex_by_node(const Graph &g, const KDNode_t &node)
{
  for (const auto &v : boost::make_iterator_range(boost::vertices(g)))
  {
    if (are_nodes_equal(g[v], node))
    { // Use the node comparison function
      return v;
    }
  }
  throw std::runtime_error("Parent node not found in the graph");
}

bool RRT::are_nodes_equal(const KDNode_t& a, const KDNode_t& b) {
    // Check if sizes are equal
    if (a.size() != b.size()) return false;

    // Compare each element
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

bool RRT::add_node(KDNode_t &new_node, KDNode_t &nearest_node, polygon_t &map)
{
  //  add path between new_node and nearest_node
  //! if the path and the points are inside the polygon
  if (valid_segment(new_node, nearest_node, map))
  {
    auto v_new = boost::add_vertex(new_node, g);
    auto v_parent = find_vertex_by_node(g, nearest_node);
    boost::add_edge(v_parent, v_new, g);
    //  update the graph and KDTree
    this->add_kd_node(new_node);
    return true;
  }
  std::cout << "Unfeasable path" << std::endl;
  return false;

}


bool RRT::valid_point(KDNode_t &sampled_point, polygon_t &map)
{
  point_t aux_point;
  aux_point.set<0>(sampled_point.at(0));
  aux_point.set<1>(sampled_point.at(1));

  return boost::geometry::within(aux_point, map);
}

bool RRT::valid_segment(KDNode_t &start, KDNode_t &end, polygon_t &map)
{
  // check if a segment with start and end points whole within the polygon map

  point_t start_point = point_t(start.at(0), start.at(1));
  point_t end_point = point_t(end.at(0), end.at(1));

  segment_t segment = segment_t(start_point, end_point);
  if (!boost::geometry::intersects(segment, map))
  {
    return true;
  }
  return false;
}

void RRT::add_kd_node(KDNode_t &node)
{
  nodes.push_back(node);
}

KDNode_t RRT::get_nn(KDNode_t &sampled_point, int n_k)
{
  Kdtree::KdTree tree(&nodes);
  Kdtree::KdNodeVector result;
  tree.k_nearest_neighbors(sampled_point, n_k, &result);
  // TODO: molto hardcoded questa parte
  KDNode_t ret{result[0].point[0], result[0].point[1]};
  return ret;
}