
#include "graph_generator/combinatorial_based/shortest_graph.hpp"

ShortestGraph::ShortestGraph(std::vector<line_t> lines_shellfino_1, std::vector<line_t> lines_shellfino_2, std::vector<line_t> lines_gate, std::vector<line_t> lines_map, point_t start_shellfino1, point_t start_shellfino2, point_t goal)
{
    this->lines_shellfino_1 = lines_shellfino_1;
    this->lines_shellfino_2 = lines_shellfino_2;
    this->lines_gate = lines_gate;
    this->lines_map = lines_map;
    this->start_shellfino1 = start_shellfino1;
    this->start_shellfino2 = start_shellfino2;
    this->goal_point = goal;
}

Graph_s ShortestGraph::convert_to_graph(const std::vector<line_t> &multi_linestring, std::map<Vertex, point_t> &vertex_points)
{
    Graph_s graph;

    // Map to store points and their corresponding vertex indices
    std::map<point_t, size_t, PointComparator> point_to_vertex;

    // Add edges to the graph based on LineStrings
    for (const line_t &line : multi_linestring)
    {
        point_t p1 = point_t({bg::get<0, 0>(line), bg::get<0, 1>(line)});
        point_t p2 = point_t({bg::get<1, 0>(line), bg::get<1, 1>(line)});

        // Add vertices for p1 and p2 if not already in the graph
        if (point_to_vertex.find(p1) == point_to_vertex.end())
        {
            size_t vertex_index = boost::add_vertex(graph);
            point_to_vertex[p1] = vertex_index;
            vertex_points[vertex_index] = p1;
        }
        if (point_to_vertex.find(p2) == point_to_vertex.end())
        {
            size_t vertex_index = boost::add_vertex(graph);
            point_to_vertex[p2] = vertex_index;
            vertex_points[vertex_index] = p2;
        }

        // Add an edge between p1 and p2 with a weight (distance)
        float distance = boost::geometry::distance(p1, p2);
        boost::add_edge(point_to_vertex[p1], point_to_vertex[p2], distance, graph);
    }

    return graph;
}

Vertex ShortestGraph::find_vertex_by_point(const point_t &target_point, const std::map<Vertex, point_t> &vertex_points)
{
    for (const auto &[vertex, point] : vertex_points)
    {
        if (boost::geometry::equals(target_point, point))
        {
            return vertex; // Return the vertex index if the points match
        }
    }
    throw std::runtime_error("Point not found in vertex_points map");
}
std::vector<point_t> ShortestGraph::calculate_path(std::vector<line_t> lines_to_transform)
{
    // Convert to graph
    std::map<Vertex, point_t>
        vertex_points;
    Graph_s graph = convert_to_graph(lines_to_transform, vertex_points);

    // Define start and goal points (corresponding to vertices)

    // A* search
    std::vector<Vertex> predecessors(boost::num_vertices(graph)); // Store shortest path tree
    std::vector<float> distances(boost::num_vertices(graph));     // Store distances
    Vertex start = find_vertex_by_point(this->start_shellfino1, vertex_points);
    Vertex goal = find_vertex_by_point(this->goal_point, vertex_points);
    std::vector<point_t> result;

    try
    {
        boost::astar_search(
            graph, start,
            straight_line_heuristic(vertex_points, goal),
            boost::predecessor_map(&predecessors[0])
                .distance_map(&distances[0])
                .visitor(astar_goal_visitor(goal)));
    }
    catch (goal_found &)
    {

        // Goal found, reconstruct the path
        std::cout << "Path found from start to goal:\n";
        result.emplace_back(vertex_points[goal]);

        for (Vertex v = goal; v != start; v = predecessors[v])
        {
            result.emplace_back(vertex_points[predecessors[v]]);
            std::cout
                << v << " <- ";
        }

        result.emplace_back(vertex_points[predecessors[start]]);

        std::cout
            << start << "\n";
    }

    return result;
}

std::vector<point_t> ShortestGraph::get_shellfino_1_path()
{
    std::vector<line_t> lines_to_transform;
    lines_to_transform.insert(lines_to_transform.end(), this->lines_map.begin(), this->lines_map.end());
    lines_to_transform.insert(lines_to_transform.end(), this->lines_gate.begin(), this->lines_gate.end());
    lines_to_transform.insert(lines_to_transform.end(), this->lines_shellfino_1.begin(), this->lines_shellfino_1.end());
    // Map to store vertex -> Point mappings

    return calculate_path(lines_to_transform);
}

std::vector<point_t> ShortestGraph::get_shellfino_2_path()
{
    std::vector<line_t> lines_to_transform;
    lines_to_transform.insert(lines_to_transform.end(), this->lines_map.begin(), this->lines_map.end());
    lines_to_transform.insert(lines_to_transform.end(), this->lines_gate.begin(), this->lines_gate.end());
    lines_to_transform.insert(lines_to_transform.end(), this->lines_shellfino_2.begin(), this->lines_shellfino_2.end());
    // Map to store vertex -> Point mappings

    return calculate_path(lines_to_transform);
}