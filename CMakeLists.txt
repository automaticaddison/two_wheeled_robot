cmake_minimum_required(VERSION 3.5)
project(two_wheeled_robot)

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

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Include cpp "include" directory
include_directories(include)

install(
  DIRECTORY config include launch maps media meshes models params rviz scripts src two_wheeled_robot urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/run_inspection.py
  scripts/lift_controller.py
  scripts/nav_to_pose.py
  scripts/nav_through_poses.py
  scripts/pick_and_deliver.py
  scripts/robot_navigator.py
  scripts/security_demo.py
  scripts/waypoint_follower.py
  scripts/battery_state/battery_state_pub.py
  scripts/navigate_to_charging_dock.py
  scripts/navigate_to_charging_dock_v2.py
  scripts/transforms/map_to_base_link_transform.py
  DESTINATION lib/${PROJECT_NAME}
)

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


ament_package()
