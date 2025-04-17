cmake_minimum_required(VERSION 3.8)
project(robot_gold)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SDL2 REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(io_node src/io_node.cpp)
add_executable(motor_node src/motor_node.cpp)
add_executable(line_node src/line_node.cpp)
add_executable(line_follow src/line_follow.cpp)
add_executable(lidar_node src/lidar_node.cpp)
add_executable(imu_node src/imu_node.cpp)
add_executable(pokus src/smth.cpp)

ament_target_dependencies(io_node rclcpp std_msgs)
ament_target_dependencies(motor_node rclcpp std_msgs)
ament_target_dependencies(line_node rclcpp std_msgs)
ament_target_dependencies(line_follow rclcpp std_msgs)
ament_target_dependencies(lidar_node rclcpp std_msgs sensor_msgs)
ament_target_dependencies(imu_node rclcpp std_msgs sensor_msgs)
ament_target_dependencies(pokus rclcpp std_msgs sensor_msgs)

target_include_directories(io_node PUBLIC include/robot_gold)
target_include_directories(line_node PUBLIC include/robot_gold)
target_include_directories(motor_node PUBLIC include/robot_gold)
target_include_directories(line_follow PUBLIC include/robot_gold)
target_include_directories(lidar_node PUBLIC include/robot_gold)
target_include_directories(imu_node PUBLIC include/robot_gold)
target_include_directories(pokus PUBLIC include/robot_gold)

install(TARGETS
  io_node
  motor_node
  line_node
  line_follow
  lidar_node
  imu_node
  pokus
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()