#ifndef COMMANDER_NODE_H_
#define COMMANDER_NODE_H_

#include <mutex>

#include "messages/msg/joint_command.hpp"
#include "messages/msg/pose_command.hpp"
#include "messages/msg/joint_state.hpp"
#include "messages/msg/pose_state.hpp"
#include "messages/msg/double_array.hpp"
#include "messages/msg/multi_double_array_stamped.hpp"
#include "std_msgs/msg/int16.hpp"
#include "rclcpp/rclcpp.hpp"

#include "utils/robot_utilities.h"
#include "app/keyboard_control.h"
#include "utils/clock.h"
#include "omega/omega_control.h"
#include "utils/data_types.h"

class CommanderNode: public rclcpp::Node {
 public:
  CommanderNode();
  ~CommanderNode();

  void JointStateCallback(const messages::msg::JointState::SharedPtr msg);
  void PoseStateCallback(const messages::msg::PoseState::SharedPtr msg);
  void KeyValueCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void AdvanceCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void TrajectoryPredictionCallback(const messages::msg::MultiDoubleArrayStamped::SharedPtr msg);
  void TaskCallback();
  void TeleCallback();
  void TimeCallback();

  template<class T>
  void ModifyCommand(int index, double increment, T* command);
  void PublishJointCommand(Eigen::VectorXd joint_command);
  void PublishPoseCommand(Eigen::Vector3d position_command,
                          Eigen::Vector3d orientation_command);
  void PublishPoseRaw(Eigen::Vector3d position_raw,
                      Eigen::Vector3d orientation_raw);
  void PublishTrajectoryInput();
  void ProcessKey(int key_value);

  void DeclareRosParameters();
  bool LoadTrajectory(const std::string& file);
  bool CheckInTaskStartPosition(int trajectory_index);

  void SwitchMode(ControlMode mode);

 private:
  rclcpp::Subscription<messages::msg::JointState>::SharedPtr sub_joint_state_;
  rclcpp::Subscription<messages::msg::PoseState>::SharedPtr sub_pose_state_;
  rclcpp::Subscription<messages::msg::MultiDoubleArrayStamped>::SharedPtr sub_trajectory_prediction_list_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_key_value_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_advance_q_;

  rclcpp::Publisher<messages::msg::JointCommand>::SharedPtr pub_joint_command_;
  rclcpp::Publisher<messages::msg::PoseCommand>::SharedPtr pub_pose_command_;
  rclcpp::Publisher<messages::msg::PoseCommand>::SharedPtr pub_pose_raw_;
  rclcpp::Publisher<messages::msg::MultiDoubleArrayStamped>::SharedPtr pub_trajectory_prediction_list_;

  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_sub_joint_state_;
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_sub_pose_state_;
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_sub_key_value_;
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_sub_trajectory_prediction_list_;
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_sub_advance_q_;

  // slave robot states
  Eigen::VectorXd current_slave_joint_angles_; // (rad)
  Eigen::VectorXd current_slave_joint_velocities_; // (rad/s)
  Eigen::Vector3d current_slave_position_;     // xyz (mm)
  Eigen::Vector3d current_slave_orientation_;  // rpy (rad)

  Eigen::Vector3d current_master_position_;     // xyz (mm)
  Eigen::Vector3d current_master_orientation_;  // rpy (rad)
  Eigen::Vector3d current_prediction_position_T_;
  Eigen::Vector3d current_prediction_position_TC_;
  Eigen::Vector3d current_prediction_position_TCM_;
  bool current_master_button_;

  Clock clock_;
  double current_time_;
  double mechanical_delay_;
  double frequency_;
  double average_delay_computation_;

  ControlMode mode_;
  PredictionMode prediction_mode_;
  bool is_trajectory_loaded_;
  
  int current_advance_q_;
  int count_trajectory_prediction_;
  int input_stack_length_;
  int output_stack_length_;
  int prediction_filter_radius_;

  // jog step increment 
  double joint_increment_;  // (rad)
  double position_increment_; // (mm)
  double orientation_increment_; // (rad)

  std::vector<Position> trajectory_;
  int current_trajectory_;
  Position master_position_stack_;
  Position previous_command_stack_;
  std::thread* task_thread_;
  std::thread* tele_thread_;
  std::thread* time_thread_;

  std::mutex* mutex_;
  OmegaControl* omega_control_;
  
};

#endif  // COMMANDER_NODE_H_