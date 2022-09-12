#include <vector>

#include "app/iiwa_node.h"

#include "utils/robot_utilities.h"
#include "utils/utilities.h"
#include "ament_index_cpp/get_package_prefix.hpp"

IIwaNode::IIwaNode() : Node("iiwa_node"), 
                       iiwa_control_(0), 
                       is_iiwa_initialized_(false),
                       joint_angle_limits_(2),
                       workspace_limits_(2),
                       prediction_mode_(PredictionMode::NONE) {
  mutex_ = new std::mutex();
  clock_ = Clock();

  // read parameters
  DeclareRosParameters();

  std::string sub_joint_command_topic_name = 
      this->get_parameter("sub_joint_command_topic_name").as_string();
  std::string sub_pose_command_topic_name = 
      this->get_parameter("sub_pose_command_topic_name").as_string();
  std::string pub_joint_state_topic_name = 
      this->get_parameter("pub_joint_state_topic_name").as_string();
  std::string pub_pose_state_topic_name = 
      this->get_parameter("pub_pose_state_topic_name").as_string();
  std::string sub_trajectory_prediction_list_topic_name = 
      this->get_parameter("sub_trajectory_prediction_list_topic_name").as_string();
  std::string pub_trajectory_prediction_list_topic_name = 
      this->get_parameter("pub_trajectory_prediction_list_topic_name").as_string();
  
  frequency_ = this->get_parameter("frequency").as_double();
  input_stack_length_ = this->get_parameter("input_stack_length").as_int();

  // Setup ros node
  callback_group_sub_joint_command_ = 
      this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  callback_group_sub_pose_command_ = 
      this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  callback_group_sub_key_value_ = 
      this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  callback_group_sub_trajectory_prediction_list_ = 
      this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  auto sub_joint_command_opt = rclcpp::SubscriptionOptions();
  auto sub_pose_command_opt = rclcpp::SubscriptionOptions();
  auto sub_key_value_opt = rclcpp::SubscriptionOptions();
  auto sub_trajectory_prediction_list_opt = rclcpp::SubscriptionOptions();
  sub_joint_command_opt.callback_group = callback_group_sub_joint_command_;
  sub_pose_command_opt.callback_group = callback_group_sub_pose_command_;
  sub_key_value_opt.callback_group = callback_group_sub_key_value_;
  sub_trajectory_prediction_list_opt.callback_group = callback_group_sub_trajectory_prediction_list_;

  sub_joint_command_ = this->create_subscription<messages::msg::JointCommand>(
      sub_joint_command_topic_name, 1,
      std::bind(&IIwaNode::JointCommandCallback, this, std::placeholders::_1),
      sub_joint_command_opt); 
  sub_pose_command_ = this->create_subscription<messages::msg::PoseCommand>(
      sub_pose_command_topic_name, 1,
      std::bind(&IIwaNode::PoseCommandCallback, this, std::placeholders::_1),
      sub_pose_command_opt);
  sub_key_value_ = this->create_subscription<std_msgs::msg::Int16>(
      "key_value", 1,
      std::bind(&IIwaNode::KeyValueCallback, this, std::placeholders::_1),
      sub_key_value_opt); 
  sub_trajectory_prediction_list_ = this->create_subscription<messages::msg::MultiDoubleArrayStamped>(
      sub_trajectory_prediction_list_topic_name, 1,
      std::bind(&IIwaNode::TrajectoryPredictionCallback, this, std::placeholders::_1),
      sub_trajectory_prediction_list_opt);

  pub_joint_state_ = this->create_publisher<messages::msg::JointState>(
      pub_joint_state_topic_name, 1); 
  pub_pose_state_ = this->create_publisher<messages::msg::PoseState>(
      pub_pose_state_topic_name, 1); 
  pub_trajectory_prediction_list_ = this->create_publisher<messages::msg::MultiDoubleArrayStamped>(
      pub_trajectory_prediction_list_topic_name, 1);

  // setup robot
  SetupRobot();

  received_command_stack_ = Position(input_stack_length_);

  // setup threads
  time_thread_ = new std::thread(std::bind(&IIwaNode::UpdateTimeCallback, this));
  task_thread_ = new std::thread(std::bind(&IIwaNode::TaskCallback, this));
  time_thread_->detach();
  task_thread_->detach();
}

IIwaNode::~IIwaNode() {
  delete time_thread_;
  delete task_thread_;
  delete mutex_;
}

void IIwaNode::DeclareRosParameters() {
  this->declare_parameter("sub_joint_command_topic_name");
  this->declare_parameter("sub_pose_command_topic_name");
  this->declare_parameter("pub_joint_state_topic_name");
  this->declare_parameter("pub_pose_state_topic_name");

  this->declare_parameter("sub_trajectory_prediction_list_topic_name");
  this->declare_parameter("pub_trajectory_prediction_list_topic_name");

  this->declare_parameter("frequency");
  this->declare_parameter("DHtable_path");
  this->declare_parameter("joint_home_path");
  this->declare_parameter("input_stack_length");
  
  this->declare_parameter("ip_iiwa");
  this->declare_parameter("port_iiwa");
  this->declare_parameter("port_local");
  this->declare_parameter("timeout_initial");
  this->declare_parameter("timeout_communication");
  this->declare_parameter("id");
  this->declare_parameter("planner_sleep_time");
  this->declare_parameter("planner_cycle_time");
  this->declare_parameter("max_velocity");
  this->declare_parameter("max_acceleration");
  this->declare_parameter("max_joint_velocity");
  this->declare_parameter("max_joint_acceleration");
  this->declare_parameter("min_joint_limits");
  this->declare_parameter("max_joint_limits");
  this->declare_parameter("min_workspace_limits");
  this->declare_parameter("max_workspace_limits");
}

void IIwaNode::SetupRobot() {
  std::string prefix = ament_index_cpp::get_package_prefix("controller")+"/share/controller/configs/";
  std::string DHtable_path = prefix + this->get_parameter("DHtable_path").as_string();
  std::string joint_home_path_ = prefix + this->get_parameter("joint_home_path").as_string();
  KukaSunrisePlanned::InitialParameters params;

  params.hostname_ = this->get_parameter("ip_iiwa").as_string();
  params.iiwaPort_ = this->get_parameter("port_iiwa").as_int();
  params.localPort_ = this->get_parameter("port_local").as_int();
  params.inital_timeout_ = this->get_parameter("timeout_initial").as_double();
  params.comm_timeout_ = this->get_parameter("timeout_communication").as_double();
  params.Id_ = this->get_parameter("id").as_string();
  params.plannerSleepTime_ = this->get_parameter("planner_sleep_time").as_double();
  params.plannerCycleTime_ = this->get_parameter("planner_cycle_time").as_double();

  params.maxVelocity_ = Erl::Vector6d(this->get_parameter("max_velocity").as_double_array().data());
  params.maxAcceleration_ = Erl::Vector6d(this->get_parameter("max_acceleration").as_double_array().data());
  params.maxJointVelocity_ = robot::Deg2Rad<Eigen::Matrix<double, 7, 1>>(
      Eigen::Matrix<double,7,1>(this->get_parameter("max_joint_velocity").as_double_array().data()));
  params.maxJointAcceleration_ = robot::Deg2Rad<Eigen::Matrix<double, 7, 1>>(
      Eigen::Matrix<double,7,1>(this->get_parameter("max_joint_acceleration").as_double_array().data()));
  
  std::vector<double> min_joint_limits = this->get_parameter("min_joint_limits").as_double_array();
  std::vector<double> max_joint_limits = this->get_parameter("max_joint_limits").as_double_array();
  std::vector<double> min_workspace_limits = this->get_parameter("min_workspace_limits").as_double_array();
  std::vector<double> max_workspace_limits = this->get_parameter("max_workspace_limits").as_double_array();
  joint_angle_limits_[0] = robot::Deg2Rad<Eigen::VectorXd>(Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      min_joint_limits.data(), min_joint_limits.size()));
  joint_angle_limits_[1] = robot::Deg2Rad<Eigen::VectorXd>(Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      max_joint_limits.data(), max_joint_limits.size()));
  workspace_limits_[0] = Eigen::Vector3d(min_workspace_limits.data());
  workspace_limits_[1] = Eigen::Vector3d(max_workspace_limits.data());

  // setup robot
  iiwa_control_ = new IIwaControl(params);
  if (!iiwa_control_->is_connected()) {
    RCLCPP_ERROR(this->get_logger(),"ERROR! Fail to connect to robot.");
    rclcpp::shutdown();
  } else {
    iiwa_control_->SetJointAngleLimits(joint_angle_limits_);
    iiwa_control_->SetWorkspaceLimits(workspace_limits_);
    iiwa_control_->LoadHomeJointAngles(joint_home_path_);
    iiwa_control_->LoadDHTable(DHtable_path);
  }
}

void IIwaNode::UpdateTimeCallback() {
  while (rclcpp::ok()) {
    mutex_->lock();
    current_time_ = clock_.GetTime_ms();
    mutex_->unlock();
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}

void IIwaNode::TaskCallback() {
  int count = 0;
  while (rclcpp::ok()) {
    if (!iiwa_control_->is_connected()) {
      count++;
      RCLCPP_INFO(this->get_logger(),"Robot lost connection! Attemp %d/5",count);
      if (count >= 5) {
        RCLCPP_ERROR(this->get_logger(),"ERROR! Fail to connect to robot.");
        rclcpp::shutdown();
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } else {
      double base_time = current_time_;
      double end_time = base_time + 1000.0/frequency_;
      PublishJointState();
      PublishPoseState();
      int sleep_time;
      if (end_time > current_time_) {
        sleep_time = static_cast<int>(end_time-current_time_)*1000;
      } else {
        sleep_time = 0;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(sleep_time));
    }
  }
}

void IIwaNode::JointCommandCallback(
    const messages::msg::JointCommand::SharedPtr msg) {
  std::vector<double> joint_angles = msg->angles;
  Eigen::VectorXd angles = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      joint_angles.data(), joint_angles.size());

  // RCLCPP_INFO(this->get_logger(),"");
  std::cout << "Received joint angles: \n" << angles.transpose() << std::endl;
  iiwa_control_->SetJointAngles(angles);
}

void IIwaNode::PoseCommandCallback(
    const messages::msg::PoseCommand::SharedPtr msg) {
  double x = msg->pose.position.x;
  double y = msg->pose.position.y;
  double z = msg->pose.position.z;
  double qx = msg->pose.orientation.x;
  double qy = msg->pose.orientation.y;
  double qz = msg->pose.orientation.z;
  double qw = msg->pose.orientation.w;
  Eigen::Quaterniond q = Eigen::Quaterniond(qw,qx,qy,qz);

  Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4,4);
  T.topRightCorner(3,1) = Eigen::Vector3d(x,y,z);
  Eigen::Matrix3d R;
  robot::Quaternion2RotationMatrix(q, &R);
  T.topLeftCorner(3,3) = R;

  std::cout << "Received pose: \n" << T << std::endl;

  if (prediction_mode_ != PredictionMode::PASSIVE) {
    iiwa_control_->SetPose(T);
  } else {
    received_command_stack_.update(x,y,z);
    PublishTrajectoryInput();
  }
}

void IIwaNode::KeyValueCallback(std_msgs::msg::Int16::SharedPtr msg) {
  switch (msg->data) {
    case KeyValues::PRE_OFF: {
      prediction_mode_ = PredictionMode::NONE;
      break;
    }
    case KeyValues::PRE_T: {
      prediction_mode_ = PredictionMode::TRANS;
      break;
    }
    case KeyValues::PRE_TC: {
      prediction_mode_ = PredictionMode::TRANS_COMPU;
      break;
    }
    case KeyValues::PRE_TCM: {
      prediction_mode_ = PredictionMode::TRANS_COMPU_MECHA;
      break;
    }
    case KeyValues::PRE_PASSIVE: {
      prediction_mode_ = PredictionMode::PASSIVE;
      received_command_stack_ = Position(input_stack_length_,current_position_);
      break;
    }
    default: {
      prediction_mode_ = PredictionMode::NONE;
      break;
    }
  }
}

void IIwaNode::TrajectoryPredictionCallback(
    const messages::msg::MultiDoubleArrayStamped::SharedPtr msg) {
  int length = msg->data[0].data.size();
  double x = msg->data[0].data[length-1];
  double y = msg->data[1].data[length-1];
  double z = msg->data[2].data[length-1];

  Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4,4);
  T.topRightCorner(3,1) = Eigen::Vector3d(x,y,z);
  T.topLeftCorner(3,3) = current_orientation_;
  iiwa_control_->SetPose(T);
}


void IIwaNode::PublishJointState() {
  Eigen::VectorXd current_joint_angles = iiwa_control_->GetCurrentJointAngles();
  Eigen::VectorXd current_joint_velocities =
      iiwa_control_->GetCurrentJointVelocities();

  messages::msg::JointState msg;
  std::vector<double> joint_angles(
      &current_joint_angles[0],
      current_joint_angles.data() +
          current_joint_angles.cols() * current_joint_angles.rows());
  msg.angles = joint_angles;
  std::vector<double> joint_velocities(
      &current_joint_velocities[0],
      current_joint_velocities.data() +
          current_joint_velocities.cols() * current_joint_velocities.rows());
  msg.velocities = joint_velocities;

  pub_joint_state_->publish(msg);
}

void IIwaNode::PublishPoseState() {
  Eigen::MatrixXd current_pose = iiwa_control_->GetCurrentTransform();
  Eigen::Vector3d t_current = current_pose.col(3).head(3);
  Eigen::Matrix3d R_current = current_pose.topLeftCorner(3, 3);
  Eigen::Quaterniond q_current;
  robot::RotationMatrix2Quaternion(R_current, &q_current);
  Eigen::VectorXd current_velocity = iiwa_control_->GetCurrentVelocity();
  current_position_ = t_current;
  current_orientation_ = R_current;

  messages::msg::PoseState msg;
  msg.pose.position.x = t_current.x();
  msg.pose.position.y = t_current.y();
  msg.pose.position.z = t_current.z();
  msg.pose.orientation.x = q_current.x();
  msg.pose.orientation.y = q_current.y();
  msg.pose.orientation.z = q_current.z();
  msg.pose.orientation.w = q_current.w();

  msg.velocity.linear.x = current_velocity(0);
  msg.velocity.linear.y = current_velocity(1);
  msg.velocity.linear.z = current_velocity(2);
  msg.velocity.angular.x = current_velocity(3);
  msg.velocity.angular.y = current_velocity(4);
  msg.velocity.angular.z = current_velocity(5);

  pub_pose_state_->publish(msg);
}

void IIwaNode::PublishTrajectoryInput() {
  messages::msg::MultiDoubleArrayStamped msg;
  messages::msg::DoubleArray x_msg,y_msg,z_msg;
  for (int i = 0; i < received_command_stack_.x.size(); ++i) {
    x_msg.data.push_back(received_command_stack_.x[i]);
    y_msg.data.push_back(received_command_stack_.y[i]);
    z_msg.data.push_back(received_command_stack_.z[i]);
  }
  msg.data.push_back(x_msg);
  msg.data.push_back(y_msg);
  msg.data.push_back(z_msg);

  msg.timestamp = current_time_;
  pub_trajectory_prediction_list_->publish(msg);
}