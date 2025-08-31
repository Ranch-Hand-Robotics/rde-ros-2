#ifndef RDE_CPP__RDE_SUBSCRIBER_HPP_
#define RDE_CPP__RDE_SUBSCRIBER_HPP_

#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class RdeSubscriber : public rclcpp::Node
{
public:
  RdeSubscriber();

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

#endif  // RDE_CPP__RDE_SUBSCRIBER_HPP_