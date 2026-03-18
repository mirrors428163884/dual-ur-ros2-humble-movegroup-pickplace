cmake_minimum_required(VERSION 3.8)
project(robot_wrench_frame)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
# find_package(custom_action REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

ament_auto_find_build_dependencies()


# Add the action client executable
ament_auto_add_executable(mechanical_arm_control
src/mechanical_arm_test.cpp
src/mechanical_arm_control.cpp) # dependencies
# Install the executable
install(TARGETS mechanical_arm_control
DESTINATION lib/${PROJECT_NAME}
)

# Install launchfile
install(
DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}
)


include_directories(
  include
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_action
  geometry_msgs
  controller_manager_msgs
)



add_library(${PROJECT_NAME} SHARED  
  src/mechanical_arm_control.cpp
)


# 为库添加依赖项
ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
# 安装目标（库）到特定位置
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 导出依赖
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# 导出包含目录（可能是为了确保覆盖）
ament_export_include_directories(
  include
)

# 导出库（可能是为了确保覆盖）
ament_export_libraries(
  ${PROJECT_NAME}
)

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
set(ament_cmake_copyright_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()

ament_package()
