#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class RdePublisher : public rclcpp::Node
{
public:
  RdePublisher()
  : Node("rde_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("rde", 10);
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&RdePublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "hello rde " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RdePublisher>());
  rclcpp::shutdown();
  return 0;
}
