cmake_minimum_required(VERSION 3.8)
project(lifecycle_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(my_moving_robot_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp example_interfaces)

add_executable(number_publisher_lifecycle src/number_publisher_lifecycle.cpp)
ament_target_dependencies(number_publisher_lifecycle rclcpp example_interfaces rclcpp_lifecycle)

add_executable(moving_robot_lifecycle_server src/moving_robot_lifecycle_server.cpp)
ament_target_dependencies(moving_robot_lifecycle_server rclcpp my_moving_robot_interfaces rclcpp_action rclcpp_lifecycle)

install(TARGETS
  number_publisher
  number_publisher_lifecycle
  moving_robot_lifecycle_server
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
