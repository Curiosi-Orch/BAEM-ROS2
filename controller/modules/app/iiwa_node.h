#ifndef IIWA_NODE_H_
#define IIWA_NODE_H_

#include <thread>

#include "messages/msg/joint_command.hpp"
#include "messages/msg/pose_command.hpp"
#include "messages/msg/joint_state.hpp"
#include "messages/msg/pose_state.hpp"
#include "messages/msg/double_array.hpp"
#include "messages/msg/multi_double_array_stamped.hpp"
#include "std_msgs/msg/int16.hpp"

#include "iiwa/interface/iiwa_control.h"
#include "rclcpp/rclcpp.hpp"
#include "utils/clock.h"
#include "utils/data_types.h"

class IIwaNode: public rclcpp::Node {
 public:
  IIwaNode();
  ~IIwaNode();

  void JointCommandCallback(const messages::msg::JointCommand::SharedPtr msg);
  void PoseCommandCallback(const messages::msg::PoseCommand::SharedPtr msg);
  void KeyValueCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void TrajectoryPredictionCallback(const messages::msg::MultiDoubleArrayStamped::SharedPtr msg);
  void UpdateTimeCallback();
  void TaskCallback();

  void PublishJointState();
  void PublishPoseState();
  void PublishTrajectoryInput();
  void DeclareRosParameters();
  void SetupRobot();

 private:
  rclcpp::Subscription<messages::msg::JointCommand>::SharedPtr sub_joint_command_;
  rclcpp::Subscription<messages::msg::PoseCommand>::SharedPtr sub_pose_command_;
  rclcpp::Subscription<messages::msg::MultiDoubleArrayStamped>::SharedPtr sub_trajectory_prediction_list_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_key_value_;

  rclcpp::Publisher<messages::msg::JointState>::SharedPtr pub_joint_state_;
  rclcpp::Publisher<messages::msg::PoseState>::SharedPtr pub_pose_state_;
  rclcpp::Publisher<messages::msg::MultiDoubleArrayStamped>::SharedPtr pub_trajectory_prediction_list_;

  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_sub_joint_command_;
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_sub_pose_command_;
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_sub_key_value_;
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_sub_trajectory_prediction_list_;
  
  IIwaControl* iiwa_control_;
  std::thread* time_thread_;
  std::thread* task_thread_;
  std::mutex* mutex_;

  double frequency_;  // Hz
  double current_time_;
  bool is_iiwa_initialized_;
  Clock clock_;

  std::vector<Eigen::VectorXd> joint_angle_limits_;
  std::vector<Eigen::Vector3d> workspace_limits_;
  std::string joint_home_path_; 
  std::string DHtable_path_;
  
  PredictionMode prediction_mode_;
  Position received_command_stack_;
  int input_stack_length_;
  Eigen::Vector3d current_position_;
  Eigen::Matrix3d current_orientation_;
};

#endif  // IIWA_NODE_H_