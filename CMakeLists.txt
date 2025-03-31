cmake_minimum_required(VERSION 3.8)
project(motors_trajectory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)
install(
    DIRECTORY include/ 
    DESTINATION include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/RobotiQGripperControl.srv"
    "srv/MotorParams.srv"
)

add_executable(motor_mover_node src/motor_mover_node.cpp src/motor.cpp)

ament_target_dependencies(motor_mover_node
    rclcpp
    sensor_msgs
    std_msgs
)

install(
    TARGETS motor_mover_node
    DESTINATION lib/${PROJECT_NAME}
)

add_executable(robotiq_85_gripper_node src/gripper_node.cpp src/gripper.cpp src/motor.cpp)

ament_target_dependencies(robotiq_85_gripper_node
    rclcpp
    sensor_msgs
    std_msgs
    std_srvs
)

install(
    TARGETS robotiq_85_gripper_node
    DESTINATION lib/${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(motor_mover_node "${cpp_typesupport_target}")
target_link_libraries(robotiq_85_gripper_node "${cpp_typesupport_target}")

ament_package()
