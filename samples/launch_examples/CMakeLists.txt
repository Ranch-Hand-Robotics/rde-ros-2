cmake_minimum_required(VERSION 3.8)
project(rde_launch_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(test_launch_files test/test_launch_files.py)
  endif()
endif()

ament_package()
