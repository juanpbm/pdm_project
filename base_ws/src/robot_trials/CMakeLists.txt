cmake_minimum_required(VERSION 3.8)
project(robot_trials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.

#catkin_package(
#  CATKIN_DEPENDS message_runtime
#)

find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

#add_executable(base_planning_node src/base_planning_node.py)
#ament_target_dependencies(base_planning_node rclpy std_msgs)

#add_executable(mobile_base_node src/mobile_base_node.py)
#ament_target_dependencies(mobile_base_node rclpy std_msgs)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BasePlan.msg"
)


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

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  src/base_planning_node.py
  src/mobile_base_node.py
  DESTINATION lib/${PROJECT_NAME})

#install(TARGETS
#  base_planning_node
#  mobile_base_node
#  DESTINATION lib/${PROJECT_NAME})

ament_package()

