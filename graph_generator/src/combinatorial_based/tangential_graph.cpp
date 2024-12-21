#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/algorithms/intersection.hpp>

namespace bg = boost::geometry;

int main() {
    // Define types
    using Point = bg::model::d2::point_xy<double>;
    using LineSegment = bg::model::segment<Point>;
    using Circle = bg::model::polygon<Point>;

    // Define a circle centered at (0, 0) with radius 1
    Circle circle;
    constexpr int num_points = 100; // Approximate circle with many points
    double radius = 1.0;
    for (int i = 0; i <= num_points; ++i) {
        double angle = 2.0 * M_PI * i / num_points;
        circle.outer().emplace_back(radius * cos(angle), radius * sin(angle));
    }
    bg::correct(circle);

    // Define a line segment
    LineSegment line(Point(-1.0, 1.0), Point(1.0, 1.0)); // Example horizontal line

    // Find intersections
    std::vector<Point> intersection_points;
    bg::intersection(circle, line, intersection_points);

    // Determine if line is tangential
    if (intersection_points.size() == 1) {
        std::cout << "The line is tangential to the circle." << std::endl;
    } else if (intersection_points.size() > 1) {
        std::cout << "The line intersects the circle at multiple points." << std::endl;
    } else {
        std::cout << "The line does not intersect the circle." << std::endl;
    }

    return 0;
}
