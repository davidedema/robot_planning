#include "graph_generator/sampling_based/utils/rrt.hpp"

RRT::RRT()
{
  // initialize the tree
}

RRT::~RRT() {}

KDNode_t RRT::get_random_point(int index, boost::geometry::model::multi_polygon<polygon_t> &map)
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
      sampled_point.push_back(-1000);
      sampled_point.push_back(-1000);
      return sampled_point;
    }
  }
  return sampled_point;
}

KDNode_t RRT::next_point(KDNode_t &sampled_point, KDNode_t &nearest, boost::geometry::model::multi_polygon<polygon_t> &map)
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
  KDNode_t new_point = {nearest.at(0) + direction.at(0), nearest.at(1) + direction.at(1)};
  // if it is valid
  if (this->valid_point(new_point, map))
  {
    return new_point;
  }
  return {-1000, -1000};
}

bool RRT::are_nodes_equal(const KDNode_t &a, const KDNode_t &b)
{
  // Check if sizes are equal
  if (a.size() != b.size())
    return false;

  // Compare each element
  for (size_t i = 0; i < a.size(); ++i)
  {
    if (a[i] != b[i])
    {
      return false;
    }
  }
  return true;
}

bool RRT::add_edge(KDNode_t &new_node, KDNode_t &nearest_node, boost::geometry::model::multi_polygon<polygon_t> &map)
{
  //  add path between new_node and nearest_node
  //! if the path and the points are inside the polygon
  if (valid_segment(new_node, nearest_node, map))
  {
    auto v_new = boost::add_vertex(g);
    g[v_new].node = new_node;
    g[v_new].parents.push_back(nearest_node);
    this->graph_lookup[new_node] = v_new;
    auto v_parent = this->graph_lookup[nearest_node];
    double distance = sqrt(pow(new_node.at(0) - nearest_node.at(0), 2) + pow(new_node.at(1) - nearest_node.at(1), 2));
    g[v_new].cost = g[v_parent].cost + distance;
    g[v_new].l2_dist_h = sqrt(pow(new_node.at(0) - goal.at(0), 2) + pow(new_node.at(1) - goal.at(1), 2));
    g[v_parent].childs.push_back(new_node);
    boost::add_edge(v_parent, v_new, g);

    // do the double connection
    g[v_new].childs.push_back(nearest_node);
    g[v_parent].parents.push_back(new_node);

    //  update the graph and KDTree
    this->add_kd_node(new_node);
    return true;
  }
  return false;
}

bool RRT::attach_node(KDNode_t &new_node, KDNode_t &nearest_node, boost::geometry::model::multi_polygon<polygon_t> &map)
{
  //  add path between new_node and nearest_node
  //! if the path and the points are inside the polygon
  if (valid_segment(new_node, nearest_node, map))
  {
    auto v_new = boost::add_vertex(g);
    g[v_new].node = new_node;
    g[v_new].parents.push_back(new_node);
    g[v_new].childs.push_back(nearest_node);
    this->graph_lookup[new_node] = v_new;
    auto v_child = this->graph_lookup[nearest_node];
    g[v_new].cost = 0;
    g[v_new].l2_dist_h = sqrt(pow(new_node.at(0) - goal.at(0), 2) + pow(new_node.at(1) - goal.at(1), 2));
    g[v_child].parents.push_back(new_node);
    boost::add_edge(v_child, v_new, g);

    // double connection
    g[v_child].childs.push_back(new_node);
    g[v_new].parents.push_back(nearest_node);

    //  update the graph and KDTree
    this->add_kd_node(new_node);
    return true;
  }
  return false;
}

bool RRT::add_node(KDNode_t &new_node)
{
  auto v_new = boost::add_vertex(g);
  g[v_new].node = new_node;
  g[v_new].parents.push_back(new_node);
  g[v_new].cost = 0.0;
  g[v_new].l2_dist_h = sqrt(pow(new_node.at(0) - goal.at(0), 2) + pow(new_node.at(1) - goal.at(1), 2));
  this->add_kd_node(new_node);
  return true;
}

bool RRT::valid_point(KDNode_t &sampled_point, boost::geometry::model::multi_polygon<polygon_t> &map)
{
  point_t aux_point;
  aux_point.set<0>(sampled_point.at(0));
  aux_point.set<1>(sampled_point.at(1));

  return boost::geometry::within(aux_point, map);
}

bool RRT::valid_segment(KDNode_t &start, KDNode_t &end, boost::geometry::model::multi_polygon<polygon_t> &map)
{
  // check if a segment with start and end points whole within the polygon map
  // will approximate the segment with a polygon so the function within can be used

  point_t start_point = point_t(start.at(0), start.at(1));
  point_t end_point = point_t(end.at(0), end.at(1));
  polygon_t segment;
  this->create_inflated_polygon(start_point, end_point, 0.001, segment);
  if (boost::geometry::within(segment, map))
  {
    return true;
  }
  return false;
}

void RRT::create_inflated_polygon(
    const point_t &p1,
    const point_t &p2,
    double epsilon,
    polygon_t &polygon)
{
  // Extract coordinates
  double x1 = boost::geometry::get<0>(p1);
  double y1 = boost::geometry::get<1>(p1);
  double x2 = boost::geometry::get<0>(p2);
  double y2 = boost::geometry::get<1>(p2);

  // Compute the direction vector (dx, dy)
  double dx = x2 - x1;
  double dy = y2 - y1;

  // Normalize the direction vector
  double length = std::sqrt(dx * dx + dy * dy);
  dx /= length;
  dy /= length;

  // Compute the perpendicular vector (px, py)
  double px = -dy * epsilon; // Perpendicular and scaled by epsilon
  double py = dx * epsilon;

  // Compute the four corners of the rectangle
  point_t corner1(x1 + px, y1 + py);
  point_t corner2(x1 - px, y1 - py);
  point_t corner3(x2 - px, y2 - py);
  point_t corner4(x2 + px, y2 + py);

  // Construct the polygon
  polygon.outer().clear();
  polygon.outer().push_back(corner1);
  polygon.outer().push_back(corner2);
  polygon.outer().push_back(corner3);
  polygon.outer().push_back(corner4);
  polygon.outer().push_back(corner1); // Close the polygon
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

bool RRT::is_goal(KDNode_t &point)
{
  double goal_radius = 0.85;
  double dx = point.at(0) - goal.at(0);
  double dy = point.at(1) - goal.at(1);

  double distance = sqrt(dx * dx + dy * dy);

  return distance <= goal_radius;
}

void RRT::set_problem(KDNode_t &start, KDNode_t &goal)
{
  this->start = start;
  this->goal = goal;
  this->add_node(start);
}

std::vector<KDNode_t> RRT::get_path(KDNode_t &start)
{
  std::vector<KDNode_t> path;
  KDNode_t parent;
  KDNode_t current = start;
  bool done = false;

  while (!done)
  {
    parent = g[this->graph_lookup[current]].parents.at(0);
    path.insert(path.begin(), current);
    if (current == this->start)
    {
      done = true;
    }
    current = parent;
  }
  return path;
}

Graph RRT::get_graph()
{
  return this->g;
}

//----------- RRT* METHODS -----------//

KDNode_t RRT::get_best_neighbor(KDNode_t &new_point, KDNode_t &old_neigh, double range, boost::geometry::model::multi_polygon<polygon_t> &map)
{
  double current_cost, distance, best_cost;
  best_cost = 999;
  KDNode_t best = old_neigh;
  // 1) find K range nns
  Kdtree::KdTree tree(&nodes);
  Kdtree::KdNodeVector result;
  tree.range_nearest_neighbors(new_point, range, &result);
  // 2) wire with the one that produces less cost rather then the nearest
  for (auto point : result)
  {
    distance = sqrt(pow(new_point.at(0) - point.point.at(0), 2) + pow(new_point.at(1) - point.point.at(1), 2));
    auto index = graph_lookup[point.point];
    current_cost = g[index].cost + distance;
    if (current_cost < best_cost && valid_segment(new_point, point.point, map))
    {
      best = point.point;
    }
  }

  return best;
}

void RRT::rewire(KDNode_t &new_point, double range, boost::geometry::model::multi_polygon<polygon_t> &map)
{
  auto node_index = graph_lookup[new_point];
  double node_cost = g[node_index].cost;
  double candidate_cost, current_cost, distance;
  // 1) find K range nns
  Kdtree::KdTree tree(&nodes);
  Kdtree::KdNodeVector result;
  tree.range_nearest_neighbors(new_point, range, &result);
  // 2) If cost from new node to the node is less than the actual cost => rewire
  for (auto point : result)
  {
    distance = sqrt(pow(new_point.at(0) - point.point.at(0), 2) + pow(new_point.at(1) - point.point.at(1), 2));
    auto index = graph_lookup[point.point];
    current_cost = g[index].cost;
    candidate_cost = node_cost + distance;
    if (candidate_cost < current_cost && valid_segment(new_point, point.point, map))
    {
      // REWIRE -> cancel old edge and create new one
      auto parent = g[index].parents.at(0);
      auto parent_id = graph_lookup[parent];
      boost::remove_edge(parent_id, index, g);
      g[index].parents.clear();
      g[index].parents.push_back(new_point);
      boost::add_edge(node_index, index, g);
    }
  }
}

// ------------- SMOOTH ---------- //

std::vector<KDNode_t> RRT::smooth_path(std::vector<KDNode_t> &path, boost::geometry::model::multi_polygon<polygon_t> &map)
{
  // try to smooth the path
  std::vector<KDNode_t> new_path;
  KDNode_t point_a = path.at(0);
  KDNode_t point_b = path.back();
  bool finish = false;
  size_t i = path.size() - 1;

  while (!finish)
  {
    if (this->valid_segment(point_a, point_b, map))
    {
      new_path.push_back(point_a);
      i = path.size() - 1;
      if (point_b == path.back())
      {
        new_path.push_back(point_b);
        finish = true;
      }
      else
      {
        point_a = point_b;
        point_b = path.back();
      }
    }
    else
    {
      --i;
      point_b = path.at(i);

      // try{
      //   point_b = path.at(i);
      // }
      // catch(const std::out_of_range& e){
      //   finish = true;
      //   new_path = path;
      // }
    }
  }

  return new_path;
}

std::map<KDNode_t, vertex_t> RRT::get_lookup()
{
  return this->graph_lookup;
}
