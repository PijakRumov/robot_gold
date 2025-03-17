cmake_minimum_required(VERSION 3.8)
project(robot_gold)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(io_node src/io_node.cpp)
add_executable(motor_node src/motor_node.cpp)
add_executable(line_node src/line_node.cpp)
add_executable(line_follow src/line_follow.cpp)

ament_target_dependencies(io_node rclcpp std_msgs)
ament_target_dependencies(motor_node rclcpp std_msgs)
ament_target_dependencies(line_node rclcpp std_msgs)
ament_target_dependencies(line_follow rclcpp std_msgs)

target_include_directories(io_node PUBLIC include/robot_gold)
target_include_directories(line_node PUBLIC include/robot_gold)
target_include_directories(motor_node PUBLIC include/robot_gold)
target_include_directories(line_follow PUBLIC include/robot_gold)

install(TARGETS
  io_node
  motor_node
  line_node
  line_follow
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()