cmake_minimum_required(VERSION 3.8)
project(static_broadcaster_listener)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake_auto REQUIRED)

# ament_auto_find_build_dependencies()

include_directories(
  include
)

add_executable(publisher_listener_node
               src/publisher_listener_test.cpp
               src/static_frame_publisher.cpp
               src/transform_listener.cpp)
ament_target_dependencies(publisher_listener_node rclcpp tf2_ros geometry_msgs)



install(TARGETS
  publisher_listener_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  )

install(
  DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
  )

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  tf2_ros
  geometry_msgs  
)



add_library(${PROJECT_NAME} SHARED  
            src/static_frame_publisher.cpp
            src/transform_listener.cpp
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
