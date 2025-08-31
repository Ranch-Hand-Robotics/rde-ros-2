cmake_minimum_required(VERSION 3.8)
project(rde_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add include directories
include_directories(include)

# Create the publisher executable
add_executable(rde_publisher src/rde_publisher.cpp)
ament_target_dependencies(rde_publisher rclcpp std_msgs)
target_include_directories(rde_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Create the subscriber executable
add_executable(rde_subscriber src/rde_subscriber.cpp)
ament_target_dependencies(rde_subscriber rclcpp std_msgs)
target_include_directories(rde_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Install the executables
install(TARGETS
  rde_publisher
  rde_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
