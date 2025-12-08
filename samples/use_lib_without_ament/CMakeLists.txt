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

if(BUILD_TESTING)
endif()

ament_package()
