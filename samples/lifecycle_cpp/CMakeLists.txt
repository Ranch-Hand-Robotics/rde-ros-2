cmake_minimum_required(VERSION 3.8)
project(rde_lifecycle_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# Create the lifecycle node executable
add_executable(rde_lifecycle_cpp src/main.cpp)
ament_target_dependencies(rde_lifecycle_cpp
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
)

# Install the executable
install(TARGETS
  rde_lifecycle_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # Add GTest
  find_package(ament_cmake_gtest REQUIRED)
  
  # Test lifecycle node
  ament_add_gtest(test_lifecycle_node test/test_lifecycle_node.cpp)
  target_link_libraries(test_lifecycle_node
    ${rclcpp_LIBRARIES}
    ${rclcpp_lifecycle_LIBRARIES}
    ${lifecycle_msgs_LIBRARIES}
  )
  ament_target_dependencies(test_lifecycle_node
    rclcpp
    rclcpp_lifecycle
    lifecycle_msgs
  )
  
  # Install test launch files
  install(DIRECTORY
    test
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
