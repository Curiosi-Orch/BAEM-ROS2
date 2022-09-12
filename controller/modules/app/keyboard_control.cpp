#include "app/keyboard_control.h"

KeyboardControl::KeyboardControl() : Node("keyboard_node") {
  pub_key_value_ = this->create_publisher<std_msgs::msg::Int16>("key_value", 1); 
  PrintUsage();

  // setup threads
  task_thread_ = new std::thread(std::bind(&KeyboardControl::TaskCallback, this));
  task_thread_->detach();

  // modify settings for terminal input
  tcgetattr(STDIN_FILENO, &stored_settings_);
  memcpy(&new_settings_, &stored_settings_, sizeof(struct termios));
  new_settings_.c_lflag &= ~(ICANON | ECHO); // knock down keybuffer
  new_settings_.c_cc[VEOL] = 1;
  new_settings_.c_cc[VEOF] = 2;
  tcsetattr(STDIN_FILENO, TCSANOW, &new_settings_);
}

KeyboardControl::~KeyboardControl() {
  tcsetattr(STDIN_FILENO, TCSANOW, &stored_settings_);
}

void KeyboardControl::TaskCallback() {
  while (rclcpp::ok()) {
    char key_value = ScanKeyboard();
    std_msgs::msg::Int16 msg;
    msg.data = key_value;
    pub_key_value_->publish(msg);
    sleep(0.01);
  }
}

void KeyboardControl::PrintUsage() {
  std::cout << "Key board command controller for iiwa robot controling." << "\n\n"
            << "Hints: you can use <shift+key> to switch to capital letter as a shortcut." << "\n"
            << "Joint command: switch to JOG mode" << "\n"
            << "J0-\t" << "<q>\t\t" << "J0+\t" << "<Q>" << "\n"
            << "J1-\t" << "<w>\t\t" << "J1+\t" << "<W>" << "\n"
            << "J2-\t" << "<e>\t\t" << "J2+\t" << "<E>" << "\n"
            << "J3-\t" << "<r>\t\t" << "J3+\t" << "<R>" << "\n"
            << "J4-\t" << "<t>\t\t" << "J4+\t" << "<T>" << "\n"
            << "J5-\t" << "<y>\t\t" << "J5+\t" << "<Y>" << "\n"
            << "J6-\t" << "<u>\t\t" << "J6+\t" << "<U>" << "\n"
            << "\n"
            << "Pose command: switch to JOG mode" << "\n"
            << "X-\t" << "<z>\t\t" << "X+\t" << "<Z>" << "\n"
            << "Y-\t" << "<x>\t\t" << "Y+\t" << "<X>" << "\n"
            << "Z-\t" << "<c>\t\t" << "Z+\t" << "<C>" << "\n"
            << "R-\t" << "<v>\t\t" << "R+\t" << "<V>" << "\n"
            << "P-\t" << "<b>\t\t" << "P+\t" << "<B>" << "\n"
            << "Y-\t" << "<n>\t\t" << "Y+\t" << "<N>" << "\n" 
            << "\n"
            << "Mode command:" << "\n"
            << "Task\t" << "<0>or<1>\t" << "move to start of trajectory, if done, move alone trajectory." << "\n"
            << "Tele\t" << "<Enter>\t" << "alter to teleoperation mode." << "\n"
            << "Prediction options: [M=Mechanical delay\t T=Transmission delay\t C=Computational delay]" << "\n" 
            << "M+T+C\t" << "<*>\t\t" << "T+C\t" << "<+>" << "\n" 
            << "T\t" << "</>\t\t" << "Raw\t" << "<->\n"
            << "Passive\t" << "<Space>" << "\n"
            << std::endl;
}

char KeyboardControl::ScanKeyboard() {
  char in;
  if(::read(STDIN_FILENO, &in, 1) < 0) {
    perror("read():");
    exit(-1);
  }
  return in;
}
