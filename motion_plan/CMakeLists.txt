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
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(eigen_stl_containers REQUIRED) 
find_package(linkattacher_msgs REQUIRED)

# === 定义可执行文件 ===

# 1. motion_plan node
add_executable(motion_plan src/motion_plan.cpp)
target_include_directories(motion_plan PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  # 【修复】手动添加 eigen_stl_containers 的正确包含路径
  ${eigen_stl_containers_INCLUDE_DIRS} 
)
# 如果上面变量为空，尝试硬编码路径作为备选 (通常不需要，但以防万一)
# get_target_property(EIGEN_STL_INCLUDE_DIR eigen_stl_containers::eigen_stl_containers INTERFACE_INCLUDE_DIRECTORIES)
# target_include_directories(motion_plan PRIVATE ${EIGEN_STL_INCLUDE_DIR}/..) 

ament_target_dependencies(
  motion_plan
  "moveit_ros_planning_interface"
  "rclcpp"
  "control_msgs"
  "moveit_visual_tools" 
  "eigen_stl_containers"
  "linkattacher_msgs"
)
install(TARGETS motion_plan DESTINATION lib/${PROJECT_NAME})

# 2. dual_move node
add_executable(dual_move src/dual_move.cpp) 
target_include_directories(dual_move PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${eigen_stl_containers_INCLUDE_DIRS}
)
ament_target_dependencies(
  dual_move
  "moveit_ros_planning_interface"
  "rclcpp"
  "control_msgs"
  "moveit_visual_tools"   # 确保加上，因为它也依赖 moveit_core
  "eigen_stl_containers"
)
install(TARGETS dual_move DESTINATION lib/${PROJECT_NAME})

# 3. jointprinter node
add_executable(jointprinter src/jointprinter.cpp)
target_include_directories(jointprinter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${eigen_stl_containers_INCLUDE_DIRS}
)
ament_target_dependencies(
  jointprinter
  "moveit_ros_planning_interface"
  "rclcpp"
  "control_msgs"
  "moveit_visual_tools" 
  "eigen_stl_containers"
)
install(TARGETS jointprinter DESTINATION lib/${PROJECT_NAME})

# 4. dual_arm_follower node
add_executable(dual_arm_follower src/dual_arm_follower.cpp)
target_include_directories(dual_arm_follower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${eigen_stl_containers_INCLUDE_DIRS}
)
ament_target_dependencies(dual_arm_follower
  "rclcpp"
  "tf2_ros"
  "geometry_msgs"
  "moveit_servo"
  "moveit_ros_planning_interface"
  "moveit_ros_planning"
  "eigen_stl_containers" # 确保加上
)
install(TARGETS dual_arm_follower DESTINATION lib/${PROJECT_NAME})

# === 安装目录 ===
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()