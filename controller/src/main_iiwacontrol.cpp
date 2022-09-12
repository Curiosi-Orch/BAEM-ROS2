#include "app/iiwa_node.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(),2,false);
  auto iiwa_node = std::make_shared<IIwaNode>();
  executor.add_node(iiwa_node);
  executor.spin();
  // rclcpp::spin(iiwa_node);

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
