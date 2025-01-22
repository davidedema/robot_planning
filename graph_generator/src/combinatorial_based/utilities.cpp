/////////////////////// GRAPH_BUILDER

// Function to check tangency
#include "graph_generator/combinatorial_based/utilities.hpp"

bool is_tangent(const line_xy_t &line, const polygon_xy_t &border, const multipolygon_xy_t &inner_polygons)
{
    // Multi-point to collect intersections
    multi_linestring_type intersections;

    // Check intersection with the outer border
    bg::intersection(line, border, intersections);

    // If the line is tangent to the outer border, it must touch it exactly twice
    if (intersections.size() != 2)
    {
        return false;
    }

    // Verify that the line does not pass through the interior of the inner polygons
    for (const auto &polygon : inner_polygons)
    {
        multi_linestring_type inner_intersections;
        bg::intersection(line, polygon, inner_intersections);

        // If the line intersects any inner polygon at more than two points, it's not tangent
        if (!inner_intersections.empty())
        {
            return false;
        }
    }

    return true;
}



polygon_xy_t convertPolygon(const polygon_t &polygon)
{
    polygon_xy_t polygon_xy;

    // Convert the outer ring
    const auto &outer_ring = polygon.outer();
    auto &outer_ring_xy = polygon_xy.outer();
    for (const auto &point : outer_ring)
    {
        outer_ring_xy.emplace_back(point.get<0>(), point.get<1>());
    }

    // Convert the inner rings
    const auto &rings = polygon.inners();
    for (const auto &inner_ring : rings)
    {
        typename polygon_xy_t::ring_type inner_ring_xy;
        for (const auto &point : inner_ring)
        {
            inner_ring_xy.emplace_back(point.get<0>(), point.get<1>());
        }
        polygon_xy.inners().emplace_back(std::move(inner_ring_xy));
    }

    return polygon_xy;
}

multipolygon_xy_t convertMultiPolygon(const multi_polygon_t &multipolygon)
{
    multipolygon_xy_t multipolygon_xy;
    for (const auto &polygon : multipolygon)
    {
        multipolygon_xy.emplace_back(convertPolygon(polygon));
    }

    return multipolygon_xy;
}
