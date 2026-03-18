#include "static_frame_publisher.hpp"
#include "transform_listener.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // 创建节点实例
  auto node_pb = std::make_shared<StaticFramePublisher>();
  // auto node_ls = std::make_shared<TransformListener>();

  // 创建一个多线程执行器
  rclcpp::executors::MultiThreadedExecutor executor;

  // 将节点添加到执行器
  executor.add_node(node_pb);
  // executor.add_node(node_ls);

  // 使用执行器来运行节点
  executor.spin();

  // 清理资源
  executor.remove_node(node_pb);
  // executor.remove_node(node_ls);

  rclcpp::shutdown();
  return 0;
}

