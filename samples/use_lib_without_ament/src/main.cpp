#include <rclcpp/rclcpp.hpp>
#include <rde_lib_without_ament/example.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  print_example();

  rclcpp::shutdown();
  return 0;
}
