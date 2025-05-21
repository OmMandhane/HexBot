cmake_minimum_required(VERSION 3.5)
project(new_robot_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(simple_controller src/simple_controller.cpp)
ament_target_dependencies(simple_controller rclcpp geometry_msgs std_msgs sensor_msgs nav_msgs tf2 tf2_ros ${Eigen_LIBRARIES})

add_executable(keyboard_control src/keyboard_control.cpp)
ament_target_dependencies(keyboard_control rclcpp geometry_msgs std_msgs)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  simple_controller
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  keyboard_control
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/simple_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()