#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rde_lifecycle_cpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<RdeLifecycleNode>("rde_lifecycle_cpp");
  
  rclcpp::spin(node->get_node_base_interface());
  
  rclcpp::shutdown();
  return 0;
}
