#include "rde_cpp/rde_publisher.hpp"

RdePublisher::RdePublisher()
: Node("rde_publisher"), count_(0)
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("rde", 10);
  timer_ = this->create_wall_timer(
    1000ms, std::bind(&RdePublisher::timer_callback, this));
}

void RdePublisher::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "hello rde " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RdePublisher>());
  rclcpp::shutdown();
  return 0;
}
