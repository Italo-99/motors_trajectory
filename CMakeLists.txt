cmake_minimum_required(VERSION 3.5)
project(gripper)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include srv
  LIBRARIES motor_planner motor_mover gripper
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

# LIBRARIES DECLARATIONS
# motor planner library declaration
add_library(motor_planner src/motor_planner.cpp)
target_link_libraries(motor_planner ${catkin_LIBRARIES})
# add_dependencies(motor_planner ${catkin_EXPORTED_TARGETS})

# motor mover library declaration
add_library(motor_mover src/motor_mover.cpp)
target_link_libraries(motor_mover ${catkin_LIBRARIES})
# add_dependencies(motor_mover ${catkin_EXPORTED_TARGETS})

# gripper library declaration
add_library(gripper src/gripper.cpp)
target_link_libraries(gripper ${catkin_LIBRARIES})
add_dependencies(gripper ${catkin_EXPORTED_TARGETS} motor_mover)

# EXECUTABLES DECLARATIONS
# motor_planner_node code executable
add_executable(motor_planner_node src/motor_planner.cpp)
target_link_libraries(motor_planner_node ${catkin_LIBRARIES})

# robotiq85_gripper_node code executable
add_executable(robotiq85_gripper src/robotiq85_gripper_node.cpp)
target_link_libraries(robotiq85_gripper ${catkin_LIBRARIES} gripper motor_mover)