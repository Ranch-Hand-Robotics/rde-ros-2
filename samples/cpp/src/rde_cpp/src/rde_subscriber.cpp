#include "rde_cpp/rde_subscriber.hpp"

RdeSubscriber::RdeSubscriber()
: Node("rde_subscriber")
{
  subscription_ = this->create_subscription<std_msgs::msg::String>(
    "rde", 10, std::bind(&RdeSubscriber::topic_callback, this, std::placeholders::_1));
}

void RdeSubscriber::topic_callback(const std_msgs::msg::String::SharedPtr msg) const
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RdeSubscriber>());
  rclcpp::shutdown();
  return 0;
}
