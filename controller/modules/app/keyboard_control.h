#ifndef KEYBOARD_NODE_H_
#define KEYBOARD_NODE_H_

#include <stdio.h>
#include <stdlib.h>
#include <termio.h>
#include <unistd.h>

#include "std_msgs/msg/int16.hpp"
#include "rclcpp/rclcpp.hpp"

class KeyboardControl: public rclcpp::Node {
 public:
  KeyboardControl();
  ~KeyboardControl();

  void TaskCallback();

  char ScanKeyboard();
  void PrintUsage();

 private:
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_key_value_;

  std::thread* task_thread_;

  termios new_settings_;
  termios stored_settings_;
};

#endif  // KEYBOARD_NODE_H_