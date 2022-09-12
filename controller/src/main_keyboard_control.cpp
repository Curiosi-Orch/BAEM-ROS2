#include "app/keyboard_control.h"


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto keyboard_control = std::make_shared<KeyboardControl>();
  rclcpp::spin(keyboard_control);
  
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}