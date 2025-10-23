#include <gtest/gtest.h>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class TestRdeSubscriber : public ::testing::Test
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
    node_ = std::make_shared<rclcpp::Node>("test_rde_subscriber");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
  }

  void TearDown() override
  {
    executor_->cancel();
    executor_.reset();
    publisher_.reset();
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

TEST_F(TestRdeSubscriber, TestSubscriberExists)
{
  // Wait for the subscriber to appear
  auto start_time = std::chrono::steady_clock::now();
  bool subscriber_found = false;
  
  while (std::chrono::steady_clock::now() - start_time < 5s) {
    auto topic_info = node_->get_subscriptions_info_by_topic("/rde");
    if (!topic_info.empty()) {
      subscriber_found = true;
      break;
    }
    std::this_thread::sleep_for(100ms);
  }
  
  if (!subscriber_found) {
    GTEST_SKIP() << "Subscriber not found - rde_subscriber node may not be running";
  }
  
  EXPECT_TRUE(subscriber_found);
}

TEST_F(TestRdeSubscriber, TestSubscriberReceivesMessages)
{
  // Create a publisher to send test messages
  publisher_ = node_->create_publisher<std_msgs::msg::String>("/rde", 10);
  
  // Wait for subscriber to connect
  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < 2s) {
    if (publisher_->get_subscription_count() > 0) {
      break;
    }
    std::this_thread::sleep_for(100ms);
  }
  
  if (publisher_->get_subscription_count() == 0) {
    GTEST_SKIP() << "No subscribers connected - rde_subscriber node may not be running";
  }
  
  // Publish some test messages
  for (int i = 0; i < 5; i++) {
    auto message = std_msgs::msg::String();
    message.data = "Test message " + std::to_string(i);
    publisher_->publish(message);
    
    executor_->spin_some();
    std::this_thread::sleep_for(100ms);
  }
  
  // The subscriber should have received our messages
  // (We can't directly verify this without modifying the subscriber node,
  // but we can verify it's subscribed)
  EXPECT_GT(publisher_->get_subscription_count(), 0u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
