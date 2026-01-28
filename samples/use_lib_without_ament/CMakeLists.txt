cmake_minimum_required(VERSION 3.5)
project(rde_use_lib_without_ament)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rde_lib_without_ament REQUIRED)

add_executable(${PROJECT_NAME}
  src/main.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rde_lib_without_ament
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  
  add_executable(test_library_integration test/test_library_integration.cpp)
  ament_target_dependencies(test_library_integration
    rclcpp
    rde_lib_without_ament
  )
  target_link_libraries(test_library_integration
    GTest::GTest
    GTest::Main
  )
  
  enable_testing()
  add_test(
    NAME test_library_integration
    COMMAND test_library_integration
  )
endif()

ament_package()
