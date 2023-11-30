cmake_minimum_required(VERSION 3.8)
project(multi_robots)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(accel_to_cmd_vel src/accel_to_cmd_vel.cpp src/utilities.h)
ament_target_dependencies(accel_to_cmd_vel rclcpp nav_msgs geometry_msgs)

add_executable(com_to_pt_odom src/com_to_pt_odom.cpp src/utilities.h)
ament_target_dependencies(com_to_pt_odom rclcpp nav_msgs geometry_msgs)

add_executable(control_system_proportional src/control_system_proportional.cpp src/utilities.h)
ament_target_dependencies(control_system_proportional rclcpp nav_msgs geometry_msgs)

add_executable(create_target_positions src/create_target_positions.cpp src/utilities.h)
add_library(targetPositionGenerator SHARED src/targetPositionGenerator.cpp src/utilities.h)
target_link_libraries(create_target_positions targetPositionGenerator)

ament_target_dependencies(create_target_positions rclcpp nav_msgs geometry_msgs rosgraph_msgs)

install(TARGETS
  accel_to_cmd_vel
  create_target_positions
  com_to_pt_odom
  control_system_proportional
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

# Uncomment this for debug
#add_compile_options(-g)