cmake_minimum_required(VERSION 3.8)
project(motion_plan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-variable)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)  # 添加此行
find_package(moveit_servo REQUIRED)
# find_package(moveit_visual_tools REQUIRED)

# === 定义可执行文件 ===

# byd move group node
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

# dual move group node
add_executable(dual_move src/dual_move.cpp) # 注意：您的上下文里是 jointprinter.cpp，但注释是 dual_move
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

# jointprinter node
# 假设您的源文件是 src/jointprinter.cpp
add_executable(jointprinter src/jointprinter.cpp)
target_include_directories(jointprinter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  jointprinter
  "moveit_ros_planning_interface"
  "rclcpp"
  "control_msgs"
)
install(TARGETS jointprinter
  DESTINATION lib/${PROJECT_NAME})

# 添加新节点
add_executable(dual_arm_follower src/dual_arm_follower.cpp)
target_include_directories(dual_arm_follower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# 链接依赖
ament_target_dependencies(dual_arm_follower
  "rclcpp"
  "tf2_ros"
  "geometry_msgs"
  "moveit_servo"
  "moveit_ros_planning_interface"
  "moveit_ros_planning"
)

# 安装
install(TARGETS dual_arm_follower
  DESTINATION lib/${PROJECT_NAME}
)


# === 安装 include 目录 ===
# 这会将 include/ 目录下的所有头文件安装到 install/motion_plan/include/
install(DIRECTORY include/
  DESTINATION include
)

# === 安装 launch 目录 ===
# 这会将 launch/ 目录下的所有 .launch.py 或 .launch.xml 文件安装到 install/motion_plan/share/motion_plan/launch/
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# === 安装 config 目录 (可选但推荐) ===
# 如果您的 YAML 配置文件（如 pick_object.yaml）放在 config 目录下，也应一并安装
# install(DIRECTORY config/
#   DESTINATION share/${PROJECT_NAME}/config
# )

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