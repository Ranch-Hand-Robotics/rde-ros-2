#include <gtest/gtest.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <rde_lib_without_ament/example.hpp>

// Test that the library can be included and used
class UseLibWithoutAmentTest : public ::testing::Test {
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

// Test that the external library function can be called
TEST_F(UseLibWithoutAmentTest, ExternalLibraryFunctionCallable) {
  EXPECT_NO_THROW(print_example());
}

// Test that ROS node can be created and uses the library
TEST_F(UseLibWithoutAmentTest, NodeCanBeCreated) {
  auto node = std::make_shared<rclcpp::Node>("test_node");
  EXPECT_NE(node, nullptr);
  EXPECT_EQ(node->get_name(), std::string("test_node"));
  
  // Library function should still be callable within ROS node context
  EXPECT_NO_THROW(print_example());
}

// Test multiple integrations
TEST_F(UseLibWithoutAmentTest, MultipleNodeIntegration) {
  auto node1 = std::make_shared<rclcpp::Node>("node_1");
  auto node2 = std::make_shared<rclcpp::Node>("node_2");
  
  EXPECT_NE(node1, nullptr);
  EXPECT_NE(node2, nullptr);
  
  // Library should work with multiple ROS nodes
  EXPECT_NO_THROW({
    print_example();
    print_example();
  });
}

// Test library integration resilience
TEST_F(UseLibWithoutAmentTest, LibraryIntegrationResilience) {
  auto node = std::make_shared<rclcpp::Node>("resilience_test");
  
  // Call library multiple times
  for (int i = 0; i < 5; ++i) {
    EXPECT_NO_THROW(print_example());
  }
}
