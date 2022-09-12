#include "app/commander_node.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(),5,false);
  auto commander_node = std::make_shared<CommanderNode>();
  executor.add_node(commander_node);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}