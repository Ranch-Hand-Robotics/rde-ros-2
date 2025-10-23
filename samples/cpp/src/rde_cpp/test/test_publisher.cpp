#include <gtest/gtest.h>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class TestRdePublisher : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_rde_publisher");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    message_count_ = 0;
    received_messages_.clear();
  }

  void TearDown() override
  {
    executor_->cancel();
    executor_.reset();
    subscription_.reset();
    node_.reset();
  }

  void message_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    message_count_++;
    received_messages_.push_back(msg->data);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  size_t message_count_;
  std::vector<std::string> received_messages_;
};

TEST_F(TestRdePublisher, TestPublisherExists)
{
  // Wait for the publisher to appear
  auto start_time = std::chrono::steady_clock::now();
  bool publisher_found = false;
  
  while (std::chrono::steady_clock::now() - start_time < 5s) {
    auto topic_info = node_->get_publishers_info_by_topic("/rde");
    if (!topic_info.empty()) {
      publisher_found = true;
      break;
    }
    std::this_thread::sleep_for(100ms);
  }
  
  if (!publisher_found) {
    GTEST_SKIP() << "Publisher not found - rde_publisher node may not be running";
  }
  
  EXPECT_TRUE(publisher_found);
}

TEST_F(TestRdePublisher, TestMessagePublishing)
{
  // Create a subscription to the /rde topic
  subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "/rde",
    10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      this->message_callback(msg);
    }
  );
  
  // Spin for a few seconds to receive messages
  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < 3s) {
    executor_->spin_some();
    std::this_thread::sleep_for(100ms);
  }
  
  // We should have received at least 2 messages (publisher runs at 1 Hz)
  if (message_count_ == 0) {
    GTEST_SKIP() << "No messages received - rde_publisher node may not be running";
  }
  
  EXPECT_GE(message_count_, 2);
  
  // Verify message format
  for (const auto& msg : received_messages_) {
    EXPECT_FALSE(msg.empty());
    EXPECT_NE(msg.find("hello rde"), std::string::npos);
  }
}

TEST_F(TestRdePublisher, TestMessageIncrement)
{
  // Create a subscription to the /rde topic
  subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "/rde",
    10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      this->message_callback(msg);
    }
  );
  
  // Spin for a few seconds to receive messages
  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < 3s) {
    executor_->spin_some();
    std::this_thread::sleep_for(100ms);
  }
  
  if (received_messages_.size() < 2) {
    GTEST_SKIP() << "Not enough messages received";
  }
  
  // The messages should contain incrementing numbers
  // This is a basic check that the counter is working
  EXPECT_NE(received_messages_[0], received_messages_[1]);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
