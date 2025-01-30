
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
// Function to calculate distance from a point to all obstacles
auto calculate_distance(const pose_t &shelfino_pos, const std::vector<polygon_t> &obstacles) -> double
{
    double min_distance = std::numeric_limits<double>::max();
    for (const auto &obstacle : obstacles)
    {
        for (const auto &pt : obstacle.outer())
        {
            double dist = boost::geometry::distance(point_t(shelfino_pos[0], shelfino_pos[1]), pt);
            if (dist < min_distance)
            {
                min_distance = dist;
            }
        }
    }
    return min_distance;
}

