#include "graph_generator/graph_node.hpp"
#include "graph_generator/sampling_based/utils/rrt.hpp"

using namespace std;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Monitor execution time
  auto m = std::make_shared<GraphGenerator>();

  RCLCPP_INFO(m->get_logger(), "Waiting for obstacles, borders and gates...");
  while (!m->obstacles_r_)
  {
    rclcpp::spin_some(m->get_node_base_interface());
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  // test obstacles
  // auto obs = m->get_obstacles();

  // for (const auto &obstacle : obs)
  // {
  //   for (auto it = boost::begin(boost::geometry::exterior_ring(obstacle)); it != boost::end(boost::geometry::exterior_ring(obstacle)); ++it)
  //   {
  //     double x = boost::geometry::get<0>(*it);
  //     double y = boost::geometry::get<1>(*it);
  //     cout << "X: " << x;
  //     cout << " Y: " << y << endl;
  //   }
  // }
  // cout << " --------------------------------------- " << endl;
  // // test map borders
  // auto borders = m->get_borders();
  // for (auto it = boost::begin(boost::geometry::exterior_ring(borders)); it != boost::end(boost::geometry::exterior_ring(borders)); ++it)
  // {
  //   double x = boost::geometry::get<0>(*it);
  //   double y = boost::geometry::get<1>(*it);
  //   cout << "X: " << x;
  //   cout << " Y: " << y << endl;
  // }
  // cout << " --------------------------------------- " << endl;
  // auto gate = m->get_gate();
  // for (const auto point : gate)
  // {
  //   cout << point << endl;
  // }
  // RCLCPP_INFO(m->get_logger(), "\033[1;32m Map information received!\033[0m");

  // cout << " --------------------------------------- " << endl;
  RCLCPP_INFO(m->get_logger(), "\033[1;32m Print map\033[0m");
  auto map = m->get_map();

  // cout << map.size() << endl;

  for (auto it = boost::begin(boost::geometry::exterior_ring(map)); it != boost::end(boost::geometry::exterior_ring(map)); ++it)
  {
    double x = boost::geometry::get<0>(*it);
    double y = boost::geometry::get<1>(*it);
    cout << "X: " << x;
    cout << " Y: " << y << endl;
  }
  for (const auto &poly : map.inners())
  {
    for (const auto &point : poly)
    {
      double x = boost::geometry::get<0>(point);
      double y = boost::geometry::get<1>(point);
      cout << "X: " << x;
      cout << " Y: " << y << endl;
    }
  }
  rclcpp::shutdown();
  cout << "Done!" << endl;
  return 0;
}