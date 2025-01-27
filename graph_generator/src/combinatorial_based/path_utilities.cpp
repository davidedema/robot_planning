
#include "graph_generator/combinatorial_based/path_utilities.hpp"

nav_msgs::msg::Path convertDubinsPathToNavPath(const std::vector<dubins_curve> &dubins_curves)
{
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = rclcpp::Clock().now();

    for (const auto &curve : dubins_curves)
    {
        auto add_arc_points_to_path = [&](const dubins_arc &arc)
        {
            double step = 0.05; // Step size for sampling points along the arc
            double theta = arc.th0;

            if (std::abs(arc.k) < 1e-6)
            {
                // Straight-line case
                for (double s = 0; s <= arc.L; s += step)
                {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header = path_msg.header;
                    pose.pose.position.x = arc.x0 + s * cos(theta);
                    pose.pose.position.y = arc.y0 + s * sin(theta);
                    pose.pose.position.z = 0.0;
                    tf2::Quaternion quat(0, 0, sin(theta / 2), cos(theta / 2));
                    pose.pose.orientation.x = quat.x();
                    pose.pose.orientation.y = quat.y();
                    pose.pose.orientation.z = quat.z();
                    pose.pose.orientation.w = quat.w();

                    path_msg.poses.push_back(pose);
                }
            }
            else
            {
                // Curved case
                double radius = 1.0 / arc.k;
                for (double s = 0; s <= arc.L; s += step)
                {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header = path_msg.header;
                    pose.pose.position.x = arc.x0 + radius * (sin(theta + arc.k * s) - sin(theta));
                    pose.pose.position.y = arc.y0 + radius * (cos(theta) - cos(theta + arc.k * s));
                    pose.pose.position.z = 0.0;
                    tf2::Quaternion quat(0, 0, sin(theta / 2), cos(theta / 2));
                    pose.pose.orientation.x = quat.x();
                    pose.pose.orientation.y = quat.y();
                    pose.pose.orientation.z = quat.z();
                    pose.pose.orientation.w = quat.w();

                    path_msg.poses.push_back(pose);
                }
            }
        };

        // Add points from all arcs in the Dubins curve
        add_arc_points_to_path(curve.a1);
        add_arc_points_to_path(curve.a2);
        add_arc_points_to_path(curve.a3);
    }

    return path_msg;
}

size_t checkIntersection(const nav_msgs::msg::Path &path1, const nav_msgs::msg::Path &path2)
{
    // Get the sizes of the paths
    size_t path1_size = path1.poses.size();
    size_t path2_size = path2.poses.size();

    // Get the smaller path size to prevent out-of-bounds access
    size_t min_size = std::min(path1_size, path2_size);

    // Loop through points in the two paths
    for (size_t i = 0; i < min_size; i++)
    {
        const auto &p1 = path1.poses.at(i).pose.position;
        const auto &p2 = path2.poses.at(i).pose.position;

        // Check if the distance between points is less than twice the robot radius
        double distance = std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
        if (distance < 2 * 0.4)
        {
            return i; // Collision detected at this time step
        }
    }

    // Return a special value to indicate no collision
    return -1; // No collision detected
}

double compute_score(const nav_msgs::msg::Path &path, size_t collision_point)
{
    if (collision_point >= path.poses.size())
    {
        throw std::out_of_range("Collision point is out of bounds.");
    }

    // Compute the distance along the path from the collision point to the goal
    double total_distance = 0.0;
    for (size_t i = collision_point; i < path.poses.size() - 1; ++i)
    {
        const auto &current_pose = path.poses[i].pose.position;
        const auto &next_pose = path.poses[i + 1].pose.position;

        double dx = next_pose.x - current_pose.x;
        double dy = next_pose.y - current_pose.y;
        total_distance += std::sqrt(dx * dx + dy * dy);
    }

    // Compute the score; higher score for shorter distances
    // Avoid division by zero if total_distance is very small
    return (total_distance > 1e-6) ? (1.0 / total_distance) : std::numeric_limits<double>::infinity();
}

std::vector<KDNode_t> reschedule_path(std::vector<KDNode_t> path, KDNode_t collision_point, double step_size)
{
    std::vector<KDNode_t> new_path = path;
    // if the path is point to point
    if (path.size() == 2)
    {
        // take the mid point between collision and start
        KDNode_t mid_point = {(path.at(0).at(0) + collision_point.at(0)) / 2, (path.at(0).at(1) + collision_point.at(1)) / 2};
        // find the direction orthogonal to the goal
        KDNode_t direction = {path.at(0).at(1) - collision_point.at(1), collision_point.at(0) - path.at(0).at(0)};
        // normalize the direction
        double norm = sqrt(pow(direction.at(0), 2) + pow(direction.at(1), 2));
        direction.at(0) /= norm;
        direction.at(1) /= norm;
        // shift wrt the direction the mid point of a step size
        // KDNode_t new_point = {mid_point.at(0) - step_size * direction.at(0), mid_point.at(1) - step_size * direction.at(1)};
        KDNode_t new_point = {path.at(0).at(0) - step_size * direction.at(0), path.at(0).at(1) - step_size * direction.at(1)};
        new_path.insert(new_path.begin() + 1, collision_point);
        new_path.insert(new_path.begin() + 1, new_point);
    }
    // if the path has more point
    else
    {
        // take mid point from start and first point in path
        KDNode_t mid_point = {(path.at(0).at(0) + path.at(1).at(0)) / 2, (path.at(0).at(1) + path.at(1).at(1)) / 2};
        // find the direction orthogonal to the goal
        KDNode_t direction = {path.at(0).at(1) - path.at(1).at(1), path.at(1).at(0) - path.at(0).at(0)};
        // normalize the direction
        double norm = sqrt(pow(direction.at(0), 2) + pow(direction.at(1), 2));
        direction.at(0) /= norm;
        direction.at(1) /= norm;
        // shift wrt the direction the mid point of a step size
        // KDNode_t new_point = {mid_point.at(0) + step_size * direction.at(0), mid_point.at(1) + step_size * direction.at(1)};
        KDNode_t new_point = {path.at(0).at(0) + step_size * direction.at(0), path.at(0).at(1) + step_size * direction.at(1)};
        // insert the new point as a first point
        new_path.insert(new_path.begin() + 1, collision_point);
        new_path.insert(new_path.begin() + 1, new_point);
    }

    return new_path;
}

void genearateMovementPath(nav_msgs::msg::Path &shelfino1_nav2, nav_msgs::msg::Path &shelfino2_nav2, std::vector<point_t> points_path1, std::vector<point_t> points_path2, pose_t pose_shellfino1, pose_t pose_shellfino2, pose_t pose_gate, multi_polygon_t map)
{

    auto converted_path1 = convert_points(points_path1);
    auto converted_path2 = convert_points(points_path2);


    std::reverse(converted_path1.begin(), converted_path1.end());
    converted_path1.erase(converted_path1.begin());
    converted_path1.erase(converted_path1.end());

    std::reverse(converted_path2.begin(), converted_path2.end());
    converted_path2.erase(converted_path2.begin());
    converted_path2.erase(converted_path2.end());


    Dubins d;

    auto dubins_path_1 = d.dubins_multi_point(pose_shellfino1.at(0), pose_shellfino1.at(1), pose_shellfino1.at(2), pose_gate.at(0), pose_gate.at(1), pose_gate.at(2), converted_path1, 2, map);
    auto dubins_path_2 = d.dubins_multi_point(pose_shellfino2.at(0), pose_shellfino2.at(1), pose_shellfino2.at(2), pose_gate.at(0), pose_gate.at(1), pose_gate.at(2), converted_path2, 2, map);

    shelfino1_nav2 = convertDubinsPathToNavPath(dubins_path_1);
    shelfino2_nav2 = convertDubinsPathToNavPath(dubins_path_2);
    // Collision checking-----------------------------------------------------------------------
    converted_path1.push_back({pose_gate.at(0), pose_gate.at(1)});
    converted_path1.insert(converted_path1.begin(), {pose_shellfino1.at(0), pose_shellfino1.at(1)});
    converted_path2.push_back({pose_gate.at(0), pose_gate.at(1)});
    converted_path2.insert(converted_path2.begin(), {pose_shellfino2.at(0), pose_shellfino2.at(1)});

    int collision_index = checkIntersection(shelfino1_nav2, shelfino2_nav2);
    if (collision_index != -1)
    {
        // get the collision point as a KDNode_t
        KDNode_t collision_point = {shelfino1_nav2.poses.at(collision_index).pose.position.x, shelfino1_nav2.poses.at(collision_index).pose.position.y};
        double score1 = compute_score(shelfino1_nav2, collision_index);
        double score2 = compute_score(shelfino2_nav2, collision_index);
        // print scores

        // who has the largest score has to deviate the path
        if (score1 > score2)
        {
            converted_path1 = reschedule_path(converted_path1, collision_point, 0.5);
            converted_path1.erase(converted_path1.begin());
            converted_path1.erase(converted_path1.end());
            dubins_path_1 = d.dubins_multi_point(pose_shellfino1.at(0), pose_shellfino1.at(1), pose_shellfino1.at(2), pose_gate.at(0), pose_gate.at(1), pose_gate.at(2), converted_path1, 2.0, map);
            shelfino1_nav2 = convertDubinsPathToNavPath(dubins_path_1);
            converted_path1.push_back({pose_gate.at(0), pose_gate.at(1)});
            converted_path1.insert(converted_path1.begin(), {pose_shellfino1.at(0), pose_shellfino1.at(1)});
        }
        else
        {
            // reschedule for shelfino 2
            converted_path2 = reschedule_path(converted_path2, collision_point, 0.5);
            converted_path2.erase(converted_path2.begin());
            converted_path2.erase(converted_path2.end());
            dubins_path_2 = d.dubins_multi_point(pose_shellfino2.at(0), pose_shellfino2.at(1), pose_shellfino2.at(2), pose_gate.at(0), pose_gate.at(1), pose_gate.at(2), converted_path2, 2.0, map);
            shelfino2_nav2 = convertDubinsPathToNavPath(dubins_path_2);
            converted_path2.push_back({pose_gate.at(0), pose_gate.at(1)});
            converted_path2.insert(converted_path2.begin(), {pose_shellfino2.at(0), pose_shellfino2.at(1)});
        }
    }
}