#include "MotionManager.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

using namespace rclcpp::executors;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<clear::MotionManager>();

  MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);

  try {
    node->init();
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(node->get_logger(), e.what() << '\n');
  }

  rclcpp::shutdown();
}