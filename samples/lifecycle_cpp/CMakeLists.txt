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

# Include directories for the header file
target_include_directories(rde_lifecycle_cpp
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install the executable
install(TARGETS
  rde_lifecycle_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY
  include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  find_package(GTest REQUIRED)
  
  add_executable(test_lifecycle_node test/test_lifecycle_node.cpp)
  ament_target_dependencies(test_lifecycle_node
    rclcpp
    rclcpp_lifecycle
    lifecycle_msgs
  )
  target_include_directories(test_lifecycle_node
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_lifecycle_node
    GTest::GTest
    GTest::Main
  )
  
  enable_testing()
  add_test(
    NAME test_lifecycle_node
    COMMAND test_lifecycle_node
  )
endif()

ament_package()
