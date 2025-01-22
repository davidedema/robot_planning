#include "graph_generator/combinatorial_based/graph_builder.hpp"

// Function to find bitangent lines
std::vector<line_xy_t> find_bitangents(const multipolygon_xy_t &multipolygon)
{
    std::vector<line_xy_t> bitangents;
    std::vector<boost::geometry::model::ring<point_xy_t, true, true, std::vector, std::allocator>, std::allocator<boost::geometry::model::ring<point_xy_t, true, true, std::vector, std::allocator>>> rings;
    for (const auto &poly : multipolygon)
    {
        rings.emplace_back(poly.outer());
    }

    for (const auto &ring1 : rings)
    {
        if (!bg::is_valid(ring1))
        {
            std::cerr << "The ring is not valid!" << std::endl;
        }
        for (const auto &ring2 : rings)
        {
            if (!bg::is_valid(ring2))
            {
                std::cerr << "The ring is not valid!" << std::endl;
            }
            if (&ring1 != &ring2)
            {
                for (const auto point1 : ring1)
                {
                    for (const auto point2 : ring2)
                    {
                        line_xy_t line = line_xy_t{point1, point2};

                        bg::model::multi_point<point_xy_t> intersections;
                        bool unwanted_intersection = false;

                        for (auto &other_ring : rings)
                        {
                            if (&ring1 != &other_ring && &ring2 != &other_ring)
                                for (size_t i = 0; i < other_ring.size() - 1 && !unwanted_intersection; ++i) // check if there is internal intersection
                                {
                                    line_xy_t line_v = line_xy_t{other_ring[i], other_ring[i + 1]};
                                    if (bg::intersects(line, line_v))
                                    {
                                        unwanted_intersection = true;
                                    }
                                }
                        }

                        for (size_t i = 0; i < ring1.size() - 1 && !unwanted_intersection; ++i) // check if there is internal intersection
                        {
                            line_xy_t line_v = line_xy_t{ring1[i], ring1[i + 1]};
                            bool to_ignore1 = (bg::equals(ring1[i], point1) || bg::equals(ring1[i + 1], point1));
                            bool to_ignore2 = (bg::equals(ring1[i], point2) || bg::equals(ring1[i + 1], point2));
                            if (!to_ignore1 && !to_ignore2 && bg::intersects(line, line_v))
                            {
                                unwanted_intersection = true;
                            }
                        }

                        for (size_t i = 0; i < ring2.size() - 1 && !unwanted_intersection; ++i)
                        {
                            line_xy_t line_v = line_xy_t{ring2[i], ring2[i + 1]}; // Create a segment for the edge
                            bool to_ignore1 = (bg::equals(ring2[i], point1) || bg::equals(ring2[i + 1], point1));
                            bool to_ignore2 = (bg::equals(ring2[i], point2) || bg::equals(ring2[i + 1], point2));
                            if (!to_ignore1 && !to_ignore2 && bg::intersects(line, line_v))
                            {
                                unwanted_intersection = true;
                            }
                        }

                        if (!unwanted_intersection)
                            bitangents.emplace_back(line);
                    }
                }
            }
        }
    }

    return bitangents;
}

std::vector<point_t> get_points_at_distance(std::vector<line_t> inflated_shell, point_t inflated_center, point_t clean_center, double distance)
{

    std::vector<point_t> points;

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
    if (!isnan(bg::get<0>(C)) && !isnan(bg::get<1>(C)))
        points.emplace_back(C);

    std::cout << "A:" << bg::get<0>(A) << " y" << bg::get<1>(A) << " |B:" << bg::get<0>(B) << " y" << bg::get<1>(B) << " |C:" << bg::get<0>(C) << " y" << bg::get<1>(C) << "\n";
    // Calculate the orthogonal vector (-dy, dx)
    double ortho_dx = -unit_dy; // Negative y component of the unit vector
    double ortho_dy = unit_dx;  // Positive x component of the unit vector

    // Scale the orthogonal vector to half the length of g
    double half_length = distance / 2.0;
    double scaled_ortho_dx = ortho_dx * half_length;
    double scaled_ortho_dy = ortho_dy * half_length;

    // Calculate the endpoints of g
    point_t g1(bg::get<0>(C) + scaled_ortho_dx, bg::get<1>(C) + scaled_ortho_dy);
    point_t g2(bg::get<0>(C) - scaled_ortho_dx, bg::get<1>(C) - scaled_ortho_dy);
    line_t line = line_t({g1, g2});
    for (auto shell_line : inflated_shell)
    {
        std::vector<point_t> intersecting_points;
        bg::intersection(shell_line, line, intersecting_points);
        if (intersecting_points.size() > 0)
        {
            points.insert(points.end(), intersecting_points.begin(), intersecting_points.end());
        }
    }
    // Output the coordinates of line g
    std::cout << "Point g1: (" << bg::get<0>(g1) << ", " << bg::get<1>(g1) << ")" << std::endl;
    std::cout << "Point g2: (" << bg::get<0>(g2) << ", " << bg::get<1>(g2) << ")" << std::endl;

    return points;
}

std::vector<point_t> get_cut_points(multi_polygon_t clean, multi_polygon_t inflated)
{
    std::vector<point_t> cut_points;
    for (int i = 1; i < clean.size(); i++)
    {
        auto clean_poly = clean[i];
        auto inflated_poly = inflated[i];
        auto clean_ring = clean_poly.outer();
        auto inflated_ring = inflated_poly.outer();
        std::vector<line_t> inflated_shell;

        for (int x = 0; x < inflated_ring.size() - 1; x++)
        {
            inflated_shell.emplace_back(line_t({inflated_ring[x], inflated_ring[x + 1]}));
        }
        inflated_shell.emplace_back(line_t({inflated_ring.back(), inflated_ring[0]}));

        std::sort(clean_ring.begin(), clean_ring.end(), [](const point_t &a, const point_t &b)
                  { return (bg::get<0>(a) + bg::get<1>(b)) < (bg::get<0>(b) + bg::get<1>(b)); });
        std::sort(inflated_ring.begin(), inflated_ring.end(), [](const point_t &a, const point_t &b)
                  { return (bg::get<0>(a) + bg::get<1>(b)) < (bg::get<0>(b) + bg::get<1>(b)); });

        for (int j = 0; j < clean_ring.size(); j++)
        {

            int pc = j;
            auto points = get_points_at_distance(inflated_shell, inflated_ring[pc], clean_ring[pc], SHELFINO_INFLATION);

            cut_points.insert(cut_points.end(), points.begin(), points.end());
        }
    }
    return cut_points;
}