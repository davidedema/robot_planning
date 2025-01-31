
#include "graph_generator/combinatorial_based/path_utilities.hpp"


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


std::vector<KDNode_t> reschedule_path(std::vector<KDNode_t> path, KDNode_t start_point, boost::geometry::model::multi_polygon<polygon_t> &map)
{
    std::vector<KDNode_t> new_path = path;
    // sample U [0, 1]
    bool valid = false;
    double x = 0.0;
    double y = 0.0;
    do
    {
        float U = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        // sample theta random [0, 2pi)
        float theta = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (2 * M_PI)));
        double rho = 0.5 * sqrt(U);

        // find new x and y
        x = start_point.at(0) + rho * cos(theta);
        y = start_point.at(1) + rho * sin(theta);

        // check if the new point is valid
        if (boost::geometry::within(point_t(x, y), map))
        {
            valid = true;
        }
    } while (!valid);

    std::cout << "New point: " << start_point.at(0) << ", " << start_point.at(1) << std::endl;

    // insert the new point as a first point
    new_path.insert(new_path.begin() + 1, {x, y});

    return new_path;
}

// Function to calculate distance from a point to all obstacles
double calculate_distance(const pose_t &shelfino_pos, const std::vector<polygon_t> &obstacles)
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

void generateMovementPath(nav_msgs::msg::Path &shelfino1_nav2, nav_msgs::msg::Path &shelfino2_nav2, plan_t converted_path1, plan_t converted_path2, pose_t pose_shellfino1, pose_t pose_shellfino2, pose_t pose_gate, multi_polygon_t map, std::vector<polygon_t> obstacles)
{
    Dubins d;
    std::vector<struct dubins_curve> shelfino1_d_path;
    std::vector<struct dubins_curve> shelfino2_d_path;


    shelfino1_d_path = d.dubins_multi_point(pose_shellfino1.at(0), pose_shellfino1.at(1), pose_shellfino1.at(2), pose_gate.at(0), pose_gate.at(1), pose_gate.at(2), converted_path1, 3, map);
    shelfino2_d_path = d.dubins_multi_point(pose_shellfino2.at(0), pose_shellfino2.at(1), pose_shellfino2.at(2), pose_gate.at(0), pose_gate.at(1), pose_gate.at(2), converted_path2, 3, map);

    converted_path1.push_back({pose_gate.at(0), pose_gate.at(1)});
    converted_path1.insert(converted_path1.begin(), {pose_shellfino1.at(0), pose_shellfino1.at(1)});
    converted_path2.push_back({pose_gate.at(0), pose_gate.at(1)});
    converted_path2.insert(converted_path2.begin(), {pose_shellfino2.at(0), pose_shellfino2.at(1)});

    shelfino1_nav2 = convertDubinsPathToNavPath(shelfino1_d_path);
    shelfino2_nav2 = convertDubinsPathToNavPath(shelfino2_d_path);

    // Check collisions between the two paths
    //RCLCPP_INFO(m->get_logger(), "\033[1;32m Collision checking \033[0m");

    bool solved_collision = false;

    // Calculate distances
    double shelfino1_distance = calculate_distance(pose_shellfino1, obstacles);
    double shelfino2_distance = calculate_distance(pose_shellfino2, obstacles);

    // Scoring logic
    double shelfino1_score = shelfino1_distance > 1.0 ? shelfino1_distance * 10 : 0; // Example scoring rule
    double shelfino2_score = shelfino2_distance > 1.0 ? shelfino2_distance * 10 : 0;

    while (!solved_collision)
    {
        int collision_index = checkIntersection(shelfino1_nav2, shelfino2_nav2);
        if (collision_index != -1)
        {
            //RCLCPP_INFO(m->get_logger(), "\033[1;33m Found a collision \033[0m");
            // get the collision point as a KDNode_t
            KDNode_t collision_point = {shelfino1_nav2.poses.at(collision_index).pose.position.x, shelfino1_nav2.poses.at(collision_index).pose.position.y};

            // who has the largest score has to deviate the path
            if (shelfino1_score < shelfino2_score)
            {
                converted_path1 = reschedule_path(converted_path1, pose_shellfino1, map);
                converted_path1.erase(converted_path1.begin());
                converted_path1.erase(converted_path1.end());
                try
                {
                    shelfino1_d_path = d.dubins_multi_point(pose_shellfino1.at(0), pose_shellfino1.at(1), pose_shellfino1.at(2), pose_gate.at(0), pose_gate.at(1), pose_gate.at(2), converted_path1, 3, map);
                }
                catch (const std::exception &e)
                {
                    //RCLCPP_ERROR(m->get_logger(), "Error in Dubins");
                    continue;
                }
                shelfino1_nav2 = convertDubinsPathToNavPath(shelfino1_d_path);
                converted_path1.push_back({pose_gate.at(0), pose_gate.at(1)});
                converted_path1.insert(converted_path1.begin(), {pose_shellfino1.at(0), pose_shellfino1.at(1)});
            }
            else
            {
                // reschedule for shelfino 2
                converted_path2 = reschedule_path(converted_path2, pose_shellfino2, map);
                converted_path2.erase(converted_path2.begin());
                converted_path2.erase(converted_path2.end());
                try
                {
                    shelfino2_d_path = d.dubins_multi_point(pose_shellfino2.at(0), pose_shellfino2.at(1), pose_shellfino2.at(2), pose_gate.at(0), pose_gate.at(1), pose_gate.at(2), converted_path1, 3, map);
                }
                catch (const std::exception &e)
                {
                    //RCLCPP_ERROR(m->get_logger(), "Error in Dubins");
                    continue;
                }
                shelfino2_nav2 = convertDubinsPathToNavPath(shelfino2_d_path);
                converted_path2.push_back({pose_gate.at(0), pose_gate.at(1)});
                converted_path2.insert(converted_path2.begin(), {pose_shellfino2.at(0), pose_shellfino2.at(1)});
            }
        }
        else
        {
            solved_collision = true;
            //RCLCPP_INFO(m->get_logger(), "\033[1;32m No collisions \033[0m");
        }
    }

    //RCLCPP_INFO(m->get_logger(), "\033[1;32m Sending paths to nav2 controller \033[0m");

    if (shelfino2_nav2.poses.size() == 0)
    {
        throw std::runtime_error("Empty path 2");
    }
    if (shelfino1_nav2.poses.size() == 0)
    {
        throw std::runtime_error("Empty path 1");
    }
}
