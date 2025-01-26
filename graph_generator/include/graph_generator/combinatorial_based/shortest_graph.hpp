#include "graph_generator/combinatorial_based/utilities.hpp"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/exception.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <stdexcept>

// Define a straight-line distance heuristic

typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS,
                              boost::no_property, boost::property<boost::edge_weight_t, float>>
    Graph_s;

typedef boost::graph_traits<Graph_s>::vertex_descriptor Vertex;

// Exception to signal that the goal has been found
struct goal_found : public std::exception
{
};
struct PointComparator
{
    bool operator()(const point_t &p1, const point_t &p2) const
    {
        if (bg::get<0>(p1) != bg::get<0>(p2))
        { // Compare x-coordinates
            return bg::get<0>(p1) < bg::get<0>(p2);
        }
        return bg::get<1>(p1) < bg::get<1>(p2); // Compare y-coordinates if x is equal
    }
};

// Straight-line distance heuristic
class straight_line_heuristic : public boost::astar_heuristic<Graph_s, float>
{
public:
    straight_line_heuristic(const std::map<Vertex, point_t> &vertex_points, Vertex goal)
        : vertex_points(vertex_points), goal(goal) {}

    float operator()(Vertex u)
    {
        const point_t &p1 = vertex_points.at(u);
        const point_t &p2 = vertex_points.at(goal);
        return std::sqrt(boost::geometry::distance(p1, p2));
    }

private:
    const std::map<Vertex, point_t> &vertex_points;
    Vertex goal;
};

// Visitor to stop the search when the goal is foundw
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
    astar_goal_visitor(Vertex goal) : goal(goal) {}

    template <typename Graph>
    void examine_vertex(Vertex u, Graph &)
    {
        if (u == goal)
        {
            throw goal_found();
        }
    }

private:
    Vertex goal;
};

class ShortestGraph
{
private:
    std::vector<line_t> lines_shellfino_1;
    std::vector<line_t> lines_shellfino_2;
    std::vector<line_t> lines_gate;
    std::vector<line_t> lines_map;
    point_t start_shellfino1;
    point_t start_shellfino2;
    point_t goal_point;
    Graph_s
    convert_to_graph(const std::vector<line_t> &multi_linestring, std::map<Vertex, point_t> &vertex_points);
    Vertex find_vertex_by_point(const point_t &target_point, const std::map<Vertex, point_t> &vertex_points);
    std::vector<point_t> calculate_path(std::vector<line_t> lines_to_transform);

public:
    ShortestGraph(
        std::vector<line_t> lines_shellfino_1, std::vector<line_t> lines_shellfino_2, std::vector<line_t> lines_gate, std::vector<line_t> lines_map, point_t start_shellfino1, point_t start_shellfino2, point_t goal);
    std::vector<point_t> get_shellfino_1_path();
    std::vector<point_t> get_shellfino_2_path();
    };