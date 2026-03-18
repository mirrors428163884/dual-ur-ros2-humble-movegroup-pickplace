cmake_minimum_required(VERSION 3.8)
project(robotiq_2f_gripper_hardware LANGUAGES CXX)

include(GNUInstallDirs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robotiq_2f_gripper_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(robotiq_driver REQUIRED)

include_directories(
  include
)

add_executable(gripper_node_test
  src/robotiq_2f_gripper_node.cpp
  src/gripper_node_test.cpp
)
ament_target_dependencies(gripper_node_test 
  rclcpp
  rclcpp_action
  hardware_interface 
  pluginlib 
  serial
  robotiq_2f_gripper_msgs
  std_msgs
  sensor_msgs 
  robotiq_driver
)


add_executable(test_use_action_client_node
  src/test_use_action_client.cpp
  src/gripper_client.cpp)
ament_target_dependencies(test_use_action_client_node
  rclcpp
  rclcpp_action
  hardware_interface 
  pluginlib 
  serial
  robotiq_2f_gripper_msgs
  std_msgs
  sensor_msgs 
  robotiq_driver  
)

install(TARGETS 
  gripper_node_test
  test_use_action_client_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY 
  include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY 
  launch 
  DESTINATION share/${PROJECT_NAME}
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  rclcpp_action
  hardware_interface 
  pluginlib 
  serial
  robotiq_2f_gripper_msgs
  std_msgs
  sensor_msgs 
  robotiq_driver
)

add_library(${PROJECT_NAME}
  SHARED
  include/robotiq_2f_gripper_hardware/gripper_client.hpp
  include/robotiq_2f_gripper_hardware/robotiq_2f_gripper_node.hpp
  src/gripper_client.cpp
  src/robotiq_2f_gripper_node.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_export_targets(
  export_${PROJECT_NAME} 
)

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}          # include
)

# Install our library.
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}      # lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}      # bin
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} # include
)

ament_package()
