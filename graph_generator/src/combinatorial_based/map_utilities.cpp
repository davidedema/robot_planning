#include "graph_generator/combinatorial_based/map_utilities.hpp"

bool lineExists(const std::vector<line_t> &lines, const line_t &lineToCheck)
{
    // Assume lineToCheck has exactly 2 points

    for (const auto &line : lines)
    {
        if ((bg::get<0, 0>(line) == bg::get<0, 0>(lineToCheck) && bg::get<0, 1>(line) == bg::get<0, 1>(lineToCheck) && bg::get<1, 0>(line) == bg::get<1, 0>(lineToCheck) && bg::get<1, 1>(line) == bg::get<1, 1>(lineToCheck)) ||
            (bg::get<1, 0>(line) == bg::get<0, 0>(lineToCheck) && bg::get<1, 1>(line) == bg::get<0, 1>(lineToCheck) && bg::get<0, 0>(line) == bg::get<1, 0>(lineToCheck) && bg::get<0, 1>(line) == bg::get<1, 1>(lineToCheck)))
        {
            return true;
        }
    }
    return false;
}

// Function to find bitangent lines
std::vector<line_t> find_edges_between_obstacles(const multi_polygon_t &multipolygon)
{
    std::vector<line_t> links;
    std::vector<std::vector<point_t>> rings;
    size_t cycle_counter = 0;

    for (const auto &poly : multipolygon)
    {
        std::vector<point_t> points;
        for (auto point : poly.outer())
        {
            points.emplace_back(point);
        }

        rings.emplace_back(points);
    }

    for (const auto &poly1 : multipolygon)
    {
        auto ring1 = poly1.outer();

        if (!bg::is_valid(poly1))
        {
            std::cerr << "First ring is not valid!" << std::endl;
        }
        for (const auto &poly2 : multipolygon)
        {
            auto ring2 = poly2.outer();

            if (!bg::is_valid(poly2))
            {
                std::cerr << "Second ring is not valid!" << std::endl;
            }
            if (&poly1 != &poly2)
            {
                for (const auto point1 : ring1)
                {

                    for (const auto point2 : ring2)
                    {
                        cycle_counter++;
                        line_t line = line_t{point1, point2};
                        if (!lineExists(links, line))
                        {

                            bool unwanted_intersection = false;

                            for (const auto &other_poly : multipolygon)
                            {
                                if (&poly1 != &other_poly && &poly2 != &other_poly)
                                    if (bg::intersects(line, other_poly))
                                    {
                                        unwanted_intersection = true;
                                    }
                            }

                            for (size_t i = 0; i < ring1.size() - 1 && !unwanted_intersection; ++i) // check if there is internal intersection
                            {
                                line_t line_v = line_t{ring1[i], ring1[i + 1]};
                                bool to_ignore1 = (bg::equals(ring1[i], point1) || bg::equals(ring1[i + 1], point1));
                                bool to_ignore2 = (bg::equals(ring1[i], point2) || bg::equals(ring1[i + 1], point2));
                                if (!to_ignore1 && !to_ignore2 && bg::intersects(line, line_v))
                                {
                                    unwanted_intersection = true;
                                }
                            }

                            for (size_t i = 0; i < ring2.size() - 1 && !unwanted_intersection; ++i)
                            {
                                line_t line_v = line_t{ring2[i], ring2[i + 1]}; // Create a segment for the edge
                                bool to_ignore1 = (bg::equals(ring2[i], point1) || bg::equals(ring2[i + 1], point1));
                                bool to_ignore2 = (bg::equals(ring2[i], point2) || bg::equals(ring2[i + 1], point2));
                                if (!to_ignore1 && !to_ignore2 && bg::intersects(line, line_v))
                                {
                                    unwanted_intersection = true;
                                }
                            }

                            if (!unwanted_intersection)
                                links.emplace_back(line);
                        }
                    }
                }
            }
        }
    }
    std::cout << "Number of cycles " << cycle_counter << std::endl;
    return links;
}

// Function to find bitangent lines
std::vector<line_t> find_shellfino_map_links(const multi_polygon_t &multipolygon, const point_t point, const point_t goal_point)
{
    std::vector<line_t> links;
    std::vector<boost::geometry::model::ring<point_t, true, true, std::vector, std::allocator>, std::allocator<boost::geometry::model::ring<point_t, true, true, std::vector, std::allocator>>> rings;
    for (const auto &poly : multipolygon)
    {
        rings.emplace_back(poly.outer());
    }
    for (const auto &poly1 : multipolygon)
    {
        auto ring1 = poly1.outer();
        if (!bg::is_valid(ring1))
        {
            std::cerr << "The ring is not valid!" << std::endl;
        }

        for (const auto point1 : ring1)
        {

            line_t line = line_t{point1, point};

            bg::model::multi_point<point_t> intersections;
            bool unwanted_intersection = false;

            for (auto &other_poly : multipolygon)
            {
                if (&poly1 != &other_poly)

                    if (bg::intersects(line, other_poly))
                    {
                        unwanted_intersection = true;
                    }
            }

            for (size_t i = 0; i < ring1.size() - 1 && !unwanted_intersection; ++i) // check if there is internal intersection
            {
                line_t line_v = line_t{ring1[i], ring1[i + 1]};
                bool to_ignore1 = (bg::equals(ring1[i], point1) || bg::equals(ring1[i + 1], point1));
                if (!to_ignore1 && bg::intersects(line, line_v))
                {
                    unwanted_intersection = true;
                }
            }

            if (!unwanted_intersection)
                links.emplace_back(line);
        }
    }

    { // direct link check
        line_t line = line_t{goal_point, point};

        bg::model::multi_point<point_t> intersections;
        bool unwanted_intersection = false;

        for (auto &other_poly : multipolygon)
        {

            if (bg::intersects(line, other_poly))
            {
                unwanted_intersection = true;
            }
        }

        if (!unwanted_intersection)
            links.emplace_back(line);
    }

    return links;
}

std::vector<line_t> find_point_map_links(const multi_polygon_t &multipolygon, const point_t point)
{
    std::vector<line_t> links;
    std::vector<boost::geometry::model::ring<point_t, true, true, std::vector, std::allocator>, std::allocator<boost::geometry::model::ring<point_t, true, true, std::vector, std::allocator>>> rings;
    for (const auto &poly : multipolygon)
    {
        rings.emplace_back(poly.outer());
    }
    for (const auto &poly1 : multipolygon)
    {
        auto ring1 = poly1.outer();
        if (!bg::is_valid(ring1))
        {
            std::cerr << "The ring is not valid!" << std::endl;
        }

        for (const auto point1 : ring1)
        {

            line_t line = line_t{point1, point};

            bg::model::multi_point<point_t> intersections;
            bool unwanted_intersection = false;

            for (auto &other_poly : multipolygon)
            {
                if (&poly1 != &other_poly)

                    if (bg::intersects(line, other_poly))
                    {
                        unwanted_intersection = true;
                    }
            }

            for (size_t i = 0; i < ring1.size() - 1 && !unwanted_intersection; ++i) // check if there is internal intersection
            {
                line_t line_v = line_t{ring1[i], ring1[i + 1]};
                bool to_ignore1 = (bg::equals(ring1[i], point1) || bg::equals(ring1[i + 1], point1));
                if (!to_ignore1 && bg::intersects(line, line_v))
                {
                    unwanted_intersection = true;
                }
            }

            if (!unwanted_intersection)
                links.emplace_back(line);
        }
    }

    return links;
}

// A separate function since it has to be used only when connecting to borders
std::vector<line_t> find_border_point_map_links(const multi_polygon_t &multipolygon, const polygon_t &border_poly, const point_t point)
{
    using linestring = bg::model::linestring<point_t>;
    linestring border_line;
    bg::assign_points(border_line, border_poly.outer());

    std::vector<line_t> links;
    std::vector<boost::geometry::model::ring<point_t, true, true, std::vector, std::allocator>, std::allocator<boost::geometry::model::ring<point_t, true, true, std::vector, std::allocator>>> rings;
    for (const auto &poly : multipolygon)
    {
        rings.emplace_back(poly.outer());
    }
    for (const auto &poly1 : multipolygon)
    {
        auto ring1 = poly1.outer();
        if (!bg::is_valid(ring1))
        {
            std::cerr << "The ring is not valid!" << std::endl;
        }

        for (const auto point1 : ring1)
        {
            if (bg::intersects(point1, border_poly))
            {

                line_t line = line_t{point1, point};

                bg::model::multi_point<point_t> intersections;
                bool unwanted_intersection = false;

                for (auto &other_poly : multipolygon)
                {
                    if (&poly1 != &other_poly)

                        if (bg::intersects(line, other_poly))
                        {
                            unwanted_intersection = true;
                        }
                }

                for (size_t i = 0; i < ring1.size() - 1 && !unwanted_intersection; ++i) // check if there is internal intersection
                {
                    line_t line_v = line_t{ring1[i], ring1[i + 1]};
                    bool to_ignore1 = (bg::equals(ring1[i], point1) || bg::equals(ring1[i + 1], point1));
                    if (!to_ignore1 && bg::intersects(line, line_v))
                    {
                        unwanted_intersection = true;
                    }
                }

                if (!unwanted_intersection)
                    links.emplace_back(line);
            }
        }
    }

    return links;
}

line_t get_points_at_distance(std::vector<line_t> inflated_shell, point_t inflated_center, point_t clean_center, double distance)
{

    // Define points A and B
    point_t A(bg::get<0>(clean_center), bg::get<1>(clean_center));
    point_t B(bg::get<0>(inflated_center), bg::get<1>(inflated_center));

    // Calculate the vector from A to B
    double dx = bg::get<0>(B) - bg::get<0>(A);
    double dy = bg::get<1>(B) - bg::get<1>(A);

    // Calculate the length of the vector AB
    double length_AB = std::sqrt(dx * dx + dy * dy);

    // Normalize the vector (dx, dy)
    double unit_dx = dx / length_AB;
    double unit_dy = dy / length_AB;

    // Scale the vector to the desired distance d
    double scaled_dx = unit_dx * distance;
    double scaled_dy = unit_dy * distance;

    std::stringstream ss;
    std::string sService;

    // Calculate point C
    point_t C(bg::get<0>(A) + scaled_dx, bg::get<1>(A) + scaled_dy);
    if (isnan(bg::get<0>(C)) && isnan(bg::get<1>(C)))
        std::cout << "NAN DETECTED A:" << bg::get<0>(A) << " y" << bg::get<1>(A) << " |B:" << bg::get<0>(B) << " y" << bg::get<1>(B) << " |C:" << bg::get<0>(C) << " y" << bg::get<1>(C) << std::endl;

    // Calculate the orthogonal vector (-dy, dx)
    double ortho_dx = -unit_dy; // Negative y component of the unit vector
    double ortho_dy = unit_dx;  // Positive x component of the unit vector

    // Scale the orthogonal vector to half the length of g
    double half_length = distance;
    double scaled_ortho_dx = ortho_dx * half_length;
    double scaled_ortho_dy = ortho_dy * half_length;

    // Calculate the endpoints of g
    point_t g1(bg::get<0>(C) + scaled_ortho_dx, bg::get<1>(C) + scaled_ortho_dy);
    point_t g2(bg::get<0>(C) - scaled_ortho_dx, bg::get<1>(C) - scaled_ortho_dy);
    line_t line = line_t({g1, g2});
    std::vector<point_t> points;
    line_t line_out;
    for (auto shell_line : inflated_shell)
    {
        std::vector<point_t> intersecting_points;
        bg::intersection(shell_line, line, intersecting_points);
        if (intersecting_points.size() > 0)
        {
            points.insert(points.end(), intersecting_points.begin(), intersecting_points.end());
        }
    }
    if (points.size() != 2)
    {
        std::cout << "Intersection with != 2 points A:" << bg::get<0>(A) << " y" << bg::get<1>(A) << " |B:" << bg::get<0>(B) << " y" << bg::get<1>(B) << " |C:" << bg::get<0>(C) << " y" << bg::get<1>(C) << std::endl;
        throw std::runtime_error("error");
    }
    else
        line_out = line_t({points[0], points[1]});

    return line;
}

std::vector<line_t> get_cut_lines(multi_polygon_t clean, multi_polygon_t inflated, double tolerance)
{
    std::vector<line_t> cut_lines;
    for (size_t i = 1; i < clean.size(); i++)
    {
        auto clean_poly = clean[i];
        auto inflated_poly = inflated[i];
        auto clean_ring = clean_poly.outer();
        auto inflated_ring = inflated_poly.outer();
        std::vector<line_t> inflated_shell;

        for (size_t x = 0; x < inflated_ring.size() - 1; x++)
        {
            inflated_shell.emplace_back(line_t({inflated_ring[x], inflated_ring[x + 1]}));
        }
        inflated_shell.emplace_back(line_t({inflated_ring.back(), inflated_ring[0]}));

        std::sort(clean_ring.begin(), clean_ring.end(), [](const point_t &a, const point_t &b)
                  { return (bg::get<0>(a) + bg::get<1>(b)) < (bg::get<0>(b) + bg::get<1>(b)); });
        std::sort(inflated_ring.begin(), inflated_ring.end(), [](const point_t &a, const point_t &b)
                  { return (bg::get<0>(a) + bg::get<1>(b)) < (bg::get<0>(b) + bg::get<1>(b)); });

        for (size_t j = 0; j < clean_ring.size(); j++)
        {

            int pc = j;
            try
            {
                if (bg::distance(inflated_ring[pc], clean_ring[pc]) > tolerance)
                {
                    auto line = get_points_at_distance(inflated_shell, inflated_ring[pc], clean_ring[pc], SHELFINO_INFLATION);
                    cut_lines.emplace_back(line);
                }
            }
            catch (...)
            {
                std::cout << "get_points_at_distance exiting with exception\n";
            }
        }
    }
    return cut_lines;
}

std::vector<line_t> poly_to_lines(const multi_polygon_t &multipolygon, const polygon_t border_polygon)
{
    using linestring = bg::model::linestring<point_t>;
    linestring border_line;
    bg::assign_points(border_line, border_polygon.outer());

    std::vector<line_t> lines;
    for (auto poly : multipolygon)
    {
        if (!bg::intersects(poly, border_line))
        {
            auto ring = poly.outer();
            std::vector<point_t> intersection_points;
            bg::intersection(poly, border_polygon, intersection_points);

            for (size_t i = 1; i < ring.size() + 1; i++)
            {
                int pa, pb; // point after, point before
                pb = i - 1;
                (i == ring.size()) ? pa = 0 : pa = i;
                line_t line = line_t({ring[pb], ring[pa]});
                point_t midpoint(
                    (bg::get<0>(ring[pb]) + bg::get<0>(ring[pa])) / 2.0, // Average x-coordinates
                    (bg::get<1>(ring[pb]) + bg::get<1>(ring[pa])) / 2.0  // Average y-coordinates
                );
                // if is the line id
                if (!(bg::intersects(ring[pb], border_line) || bg::intersects(ring[pa], border_line)))
                    lines.emplace_back(line);
            }
        }
    }

    auto ring = border_polygon.outer();
    for (size_t i = 0; i < ring.size(); i++)
    {
        int pa, pb; // point after, point before
        pb = i - 1;
        (i == ring.size()) ? pa = 0 : pa = i;

        line_t line = line_t({ring[pb], ring[pa]});
        lines.emplace_back(line);
    }

    for (auto border_point : border_polygon.outer())
    {
        auto border_lines = find_border_point_map_links(multipolygon, border_polygon, border_point);
        lines.insert(lines.end(), border_lines.begin(), border_lines.end());
    }

    return lines;
}

multi_polygon_t apply_cuts_to_map(const multi_polygon_t &multipolygon, const std::vector<line_t> cut_lines)
{
    multi_polygon_t cutted_map;
    for (auto poly : multipolygon)
    {
        polygon_t new_poly;
        auto ring = poly.outer();
        std::vector<point_t> points;
        for (size_t i = 0; i < ring.size(); i++)
        {
            int pa, pb; // point after, point before
            pb = i;
            (i == ring.size() - 1) ? pa = 0 : pa = i + 1;

            std::vector<point_t> intersecting_points;
            line_t line = line_t({ring[pb], ring[pa]});
            for (auto cut_line : cut_lines)
            {
                std::vector<point_t> output;

                bg::intersection(cut_line, line, output);
                intersecting_points.insert(intersecting_points.end(), output.begin(), output.end());
            }
            if (intersecting_points.size() > 0)
            {
                point_t pb_point = ring[pb];
                std::sort(intersecting_points.begin(), intersecting_points.end(), [&pb_point](const point_t &a, const point_t &b)
                          { return bg::distance(pb_point, a) < bg::distance(pb_point, b); });
                points.emplace_back(ring[pb]);
                points.emplace_back(intersecting_points[0]);
                size_t z = 0;
                for (; z < intersecting_points.size(); z++)
                {
                    points.emplace_back(intersecting_points[z]);
                }
                points.emplace_back(ring[pa]);
            }
            else
            {
                points.emplace_back(ring[pb]);
                points.emplace_back(ring[pa]);
            }
        }
        for (const auto &p : points)
        {
            bg::append(new_poly.outer(), p);
        }

        if (!bg::is_valid(new_poly))
            std::cout << "Created not Valid" << std::endl;
        bg::correct(new_poly);
        if (!bg::is_valid(new_poly))
            std::cout << "Still not Valid" << std::endl;
        cutted_map.emplace_back(new_poly);
    }
    return cutted_map;
}

void map_edge_difference(multi_polygon_t &obstacles, polygon_t &edge)
{
    for (auto poly : obstacles)
    {
        if (bg::intersects(edge, poly))
        {
            multi_polygon_t output;
            bg::difference(edge, poly, output);
            std::sort(output.begin(), output.end(), [](const polygon_t &a, const polygon_t &b)
                      { return bg::area(a) > bg::area(b); });
            edge = output[0];
        }
    }
}