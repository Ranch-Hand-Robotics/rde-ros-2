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
  
  # Add GTest
  find_package(ament_cmake_gtest REQUIRED)
  
  # Test publisher
  ament_add_gtest(test_publisher test/test_publisher.cpp)
  target_link_libraries(test_publisher
    ${rclcpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
  )
  ament_target_dependencies(test_publisher
    rclcpp
    std_msgs
  )
  
  # Test subscriber
  ament_add_gtest(test_subscriber test/test_subscriber.cpp)
  target_link_libraries(test_subscriber
    ${rclcpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
  )
  ament_target_dependencies(test_subscriber
    rclcpp
    std_msgs
  )
  
  # Install test launch files
  install(DIRECTORY
    test
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
