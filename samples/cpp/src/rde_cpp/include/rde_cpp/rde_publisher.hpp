#ifndef RDE_CPP__RDE_PUBLISHER_HPP_
#define RDE_CPP__RDE_PUBLISHER_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class RdePublisher : public rclcpp::Node
{
public:
  RdePublisher();

private:
  void timer_callback();
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

#endif  // RDE_CPP__RDE_PUBLISHER_HPP_