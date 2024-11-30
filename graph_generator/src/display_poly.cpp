#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

class PolygonPublisher : public rclcpp::Node
{
public:
  PolygonPublisher()
      : Node("polygon_publisher")
  {
    // Publisher for visualization markers
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    // Timer to publish markers periodically
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&PolygonPublisher::publishPolygons, this));
  }

private:
  void publishPolygons()
  {
    std::vector<std::vector<geometry_msgs::msg::Point>> polygons = createPolygons();

    int id = 0; // Marker ID counter
    for (const auto &polygon : polygons)
    {
      auto marker = createPolygonMarker(id++, polygon, "map", 1.0, 0.0, 0.0);
      marker_pub_->publish(marker);
    }
  }

  // Function to create a polygon marker
  visualization_msgs::msg::Marker createPolygonMarker(int id,
                                                      const std::vector<geometry_msgs::msg::Point> &points,
                                                      const std::string &frame_id,
                                                      float r, float g, float b)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "polygons";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.1; // Line width
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0; // Fully opaque

    // Add points to the marker
    for (const auto &point : points)
    {
      marker.points.push_back(point);
    }

    // Close the polygon by connecting the last point to the first
    if (!points.empty())
    {
      marker.points.push_back(points.front());
    }

    return marker;
  }

  // Function to create example polygons
  std::vector<std::vector<geometry_msgs::msg::Point>> createPolygons()
  {
    std::vector<std::vector<geometry_msgs::msg::Point>> polygons;

    // Triangle
    polygons.push_back(createPolygon({{-10.0, 10.0}, {-10.0, -10.0}, {10.0, -10.0}, {10.0, 10.0}}));

    // Square
    polygons.push_back(createPolygon({{4.19398, 1.5522}, {3.7294, 2.07499}, {3.21636, 1.61907}, {3.68094, 1.09628}}));

    // Pentagon
    polygons.push_back(createPolygon({{-3.30637, 1.08966}, {-2.64501, 1.0538}, {-2.59385, 1.99747}, {-3.25521, 2.03332}}));

    // Hexagon
    polygons.push_back(createPolygon({{-1.94244, -5.85587}, {-1.86404, -6.63454}, {-1.24442, -6.57214}, {-1.32282, -5.79348}}));

    // Octagon
    polygons.push_back(createPolygon({{-5.33034, -7.35524}, {-4.3944, -7.28044}, {-4.47063, -6.32658}, {-5.40657, -6.40138}}));

    // Custom shape
    polygons.push_back(createPolygon({{6.15544, 0.529636}, {6.93099, 0.876703}, {6.7194, 1.34951}, {5.94385, 1.00244}}));

    return polygons;
  }

  // Function to create a polygon from points
  std::vector<geometry_msgs::msg::Point> createPolygon(const std::vector<std::pair<double, double>> &vertices)
  {
    std::vector<geometry_msgs::msg::Point> points;
    for (const auto &vertex : vertices)
    {
      geometry_msgs::msg::Point point;
      point.x = vertex.first;
      point.y = vertex.second;
      point.z = 0.0; // Flat on the ground
      points.push_back(point);
    }
    return points;
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PolygonPublisher>());
  rclcpp::shutdown();
  return 0;
}
