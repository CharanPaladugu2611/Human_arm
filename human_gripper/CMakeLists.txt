cmake_minimum_required(VERSION 3.8)
project(human_gripper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find dependencies
# set(CMAKE_PREFIX_PATH "/home/jason/proj2ws/install/human_gripper/share/human_gripper/cmake")
find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(tf2 REQUIRED)
find_package(ros2_control REQUIRED)
find_package(tf2_ros REQUIRED)


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

install(
  DIRECTORY
  urdf
  meshes
  launch
  worlds
  config
  DESTINATION share/human_gripper/
)

install(
  DIRECTORY include/human_gripper
  DESTINATION include
)

install(
  PROGRAMS
  launch/robot_description_publisher.py
  launch/gazebo.launch.py
  launch/spawn_robot_ros2.launch.py
  DESTINATION lib/human_gripper
)

ament_package()