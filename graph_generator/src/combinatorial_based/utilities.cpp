/////////////////////// GRAPH_BUILDER

// Function to check tangency
#include "graph_generator/combinatorial_based/utilities.hpp"


std::vector<std::vector<double>> convert_points(std::vector<point_t> points){
    std::vector<std::vector<double>> new_points;
    for (auto point :points){
        std::vector<double> new_point;
        new_point.push_back(bg::get<0>(point));
        new_point.push_back(bg::get<1>(point));
        new_points.push_back(new_point);
    }
    std::reverse(new_points.begin(), new_points.end());
    new_points.erase(new_points.begin());
    new_points.erase(new_points.end());
    return new_points;
}