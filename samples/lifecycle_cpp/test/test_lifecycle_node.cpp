#include <gtest/gtest.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rde_lifecycle_cpp.hpp"

// Mock the main code for testing the RdeLifecycleNode
class RdeLifecycleNodeTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

// Test that lifecycle node can be instantiated
TEST_F(RdeLifecycleNodeTest, NodeInstantiation) {
  auto node = std::make_shared<RdeLifecycleNode>("test_node");
  EXPECT_NE(node, nullptr);
  EXPECT_EQ(node->get_name(), std::string("test_node"));
}

// Test that on_configure reBturns SUCCESS
TEST_F(RdeLifecycleNodeTest, OnConfigureReturnsSuccess) {
  auto node = std::make_shared<RdeLifecycleNode>("test_node");
  rclcpp_lifecycle::State state;
  auto result = node->on_configure(state);
  EXPECT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS
  );
}

// Test that on_activate returns SUCCESS
TEST_F(RdeLifecycleNodeTest, OnActivateReturnsSuccess) {
  auto node = std::make_shared<RdeLifecycleNode>("test_node");
  rclcpp_lifecycle::State state;
  auto result = node->on_activate(state);
  EXPECT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS
  );
}

// Test that on_deactivate returns SUCCESS
TEST_F(RdeLifecycleNodeTest, OnDeactivateReturnsSuccess) {
  auto node = std::make_shared<RdeLifecycleNode>("test_node");
  rclcpp_lifecycle::State state;
  auto result = node->on_deactivate(state);
  EXPECT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS
  );
}

// Test that on_cleanup returns SUCCESS
TEST_F(RdeLifecycleNodeTest, OnCleanupReturnsSuccess) {
  auto node = std::make_shared<RdeLifecycleNode>("test_node");
  rclcpp_lifecycle::State state;
  auto result = node->on_cleanup(state);
  EXPECT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS
  );
}

// Test that on_shutdown returns SUCCESS
TEST_F(RdeLifecycleNodeTest, OnShutdownReturnsSuccess) {
  auto node = std::make_shared<RdeLifecycleNode>("test_node");
  rclcpp_lifecycle::State state;
  auto result = node->on_shutdown(state);
  EXPECT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS
  );
}

// Test that on_error returns SUCCESS
TEST_F(RdeLifecycleNodeTest, OnErrorReturnsSuccess) {
  auto node = std::make_shared<RdeLifecycleNode>("test_node");
  rclcpp_lifecycle::State state;
  auto result = node->on_error(state);
  EXPECT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS
  );
}

// Test lifecycle transition: configure -> activate
TEST_F(RdeLifecycleNodeTest, LifecycleConfigureToActivate) {
  auto node = std::make_shared<RdeLifecycleNode>("test_node");
  rclcpp_lifecycle::State state;
  
  // First configure
  auto result = node->on_configure(state);
  EXPECT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS
  );
  
  // Then activate
  result = node->on_activate(state);
  EXPECT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS
  );
}

// Test lifecycle transition: activate -> deactivate
TEST_F(RdeLifecycleNodeTest, LifecycleActivateToDeactivate) {
  auto node = std::make_shared<RdeLifecycleNode>("test_node");
  rclcpp_lifecycle::State state;
  
  // Activate
  auto result = node->on_activate(state);
  EXPECT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS
  );
  
  // Then deactivate
  result = node->on_deactivate(state);
  EXPECT_EQ(
    result,
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS
  );
}
