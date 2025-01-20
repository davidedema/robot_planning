#pragma once

#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "clipper.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "rclcpp/qos.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using namespace ClipperLib;
namespace bg = boost::geometry;

#define SHELFINO_INFLATION 0.5
#define CIRCLE_APPROXIMATION 36

typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_t;
typedef boost::geometry::model::polygon<point_t> polygon_t;
typedef boost::geometry::model::multi_polygon<polygon_t> multi_polygon_t;

typedef boost::geometry::model::d2::point_xy<double> point_xy_t;
typedef boost::geometry::model::polygon<point_xy_t> polygon_xy_t;


typedef std::vector<double> pose_t;
const double CLIPPER_SCALE_FACTOR = 1000.0;
static const rmw_qos_profile_t qos_profile_custom = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    10,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};

class MapConstruction : public rclcpp::Node
{
public:
    // constructor and distructor
    MapConstruction();
    ~MapConstruction();

    // callbacks for the subscribers
    void callback_borders(const geometry_msgs::msg::Polygon::SharedPtr msg);
    void callback_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg);
    void callback_gates(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void callback_pos1(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void callback_pos2(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void callback_pos3(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // flags for received msgs
    bool borders_r_;
    bool obstacles_r_;
    bool gates_r_;
    bool pos1_r_;
    bool pos2_r_;
    bool pos3_r_;

    // flag for inflated_map creation
    bool is_map_created;

    // setter for inflated_map
    void set_obstacles(const obstacles_msgs::msg::ObstacleArrayMsg &msg);
    void inflate_obstacles();
    void set_borders(const geometry_msgs::msg::Polygon &msg);
    void set_gate(const geometry_msgs::msg::PoseArray &msg);
    void set_pos1(const geometry_msgs::msg::PoseWithCovarianceStamped &msg);
    void set_pos2(const geometry_msgs::msg::PoseWithCovarianceStamped &msg);
    void set_pos3(const geometry_msgs::msg::PoseWithCovarianceStamped &msg);

    // getter for inflated_map
    std::vector<polygon_t> get_obstacles();
    polygon_t get_borders();
    pose_t get_gate();

    pose_t get_pose1();
    pose_t get_pose2();
    pose_t get_pose3();

    // get the inflated_map (polygon with holes replacing the obstacles)
    boost::geometry::model::multi_polygon<polygon_t> get_map();
    ClipperLib::Paths boostToClipper(const polygon_t &boost_polygon);
    polygon_t clipperToBoost(const ClipperLib::Paths &clipper_paths);

private:
    // inflated_map values
    std::vector<polygon_t> obstacle_arr;
    multi_polygon_t inflated_obstacles;

    polygon_t map_borders;
    multi_polygon_t inflated_borders;
    std::vector<pose_t> gates;
    multi_polygon_t clean_map;

    multi_polygon_t inflated_map;

    pose_t pos1;
    pose_t pos2;
    pose_t pos3;

    // subscriber
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subscription_borders_;
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subscription_obstacles_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_gates_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_position1_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_position2_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_position3_;
};