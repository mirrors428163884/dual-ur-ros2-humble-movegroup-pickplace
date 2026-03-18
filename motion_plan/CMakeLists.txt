cmake_minimum_required(VERSION 3.8)
project(motion_plan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-variable)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(control_msgs REQUIRED)
# find_package(moveit_visual_tools REQUIRED)


#byd move group node
add_executable(motion_plan src/motion_plan.cpp)
target_include_directories(motion_plan PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  motion_plan
  "moveit_ros_planning_interface"
  "rclcpp"
)
install(TARGETS motion_plan
  DESTINATION lib/${PROJECT_NAME})



#dual move group node
add_executable(dual_move src/dual_move.cpp)
target_include_directories(dual_move PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  dual_move
  "moveit_ros_planning_interface"
  "rclcpp"
  "control_msgs"
)
install(TARGETS dual_move
  DESTINATION lib/${PROJECT_NAME})

#jointprinternode
add_executable(jointprinter src/jointprinter.cpp)
target_include_directories(jointprinter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  jointprinter
  "moveit_ros_planning_interface"
  "rclcpp"
)
install(TARGETS jointprinter
  DESTINATION lib/${PROJECT_NAME})

#


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

ament_package()
