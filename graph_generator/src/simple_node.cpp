#include "graph_generator/graph_node.hpp"

using namespace std;


int main(int argc, char **argv){
  rclcpp::init(argc, argv);

  // Monitor execution time
  auto m = std::make_shared<GraphGenerator>();

  RCLCPP_INFO(m->get_logger(), "Waiting for obstacles, borders and gates...");
  while (rclcpp::ok()) {
    // RCLCPP_INFO(m->get_logger(), "Map received: %d, Border: %d, Gates: %d",
    // m->obstacles_received_, m->borders_received_, m->gates_received_);
    rclcpp::spin_some(m->get_node_base_interface());
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(m->get_logger(), "\033[1;32m Map information received!\033[0m");
  rclcpp::shutdown();
  cout << "Done!" << endl;
  return 0;
}