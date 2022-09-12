#include "app/commander_node.h"
#include "ament_index_cpp/get_package_prefix.hpp"

CommanderNode::CommanderNode(): Node("commander_node"),
                                mode_(ControlMode::JOG),
                                count_trajectory_prediction_(0),
                                average_delay_computation_(0),
                                is_trajectory_loaded_(false),
                                prediction_mode_(PredictionMode::NONE),
                                current_advance_q_(0),
                                current_trajectory_(0) {
  current_slave_joint_angles_.setZero(7,1);
  current_slave_joint_velocities_.setZero(7,1);
  current_slave_position_.setZero(3,1);
  current_slave_orientation_.setZero(3,1);
  clock_ = Clock();

  mutex_ = new std::mutex();
  omega_control_ = new OmegaControl();

  // read parameters
  DeclareRosParameters();
  std::string prefix = ament_index_cpp::get_package_prefix("controller")+"/../../src/";

  std::string sub_joint_state_topic_name = 
      this->get_parameter("sub_joint_state_topic_name").as_string();
  std::string sub_pose_state_topic_name = 
      this->get_parameter("sub_pose_state_topic_name").as_string();
  std::string sub_trajectory_prediction_list_topic_name = 
      this->get_parameter("sub_trajectory_prediction_list_topic_name").as_string();
  std::string sub_advance_q_topic_name = 
      this->get_parameter("sub_advance_q_topic_name").as_string();
  std::string pub_joint_command_topic_name = 
      this->get_parameter("pub_joint_command_topic_name").as_string();
  std::string pub_pose_command_topic_name = 
      this->get_parameter("pub_pose_command_topic_name").as_string();
  std::string pub_pose_raw_topic_name = 
      this->get_parameter("pub_pose_raw_topic_name").as_string();    
  std::string pub_trajectory_prediction_list_topic_name = 
      this->get_parameter("pub_trajectory_prediction_list_topic_name").as_string();
  frequency_ = this->get_parameter("frequency").as_double();
  mechanical_delay_ = this->get_parameter("mechanical_delay").as_double();
  input_stack_length_ = this->get_parameter("input_stack_length").as_int();
  output_stack_length_ = this->get_parameter("output_stack_length").as_int();
  prediction_filter_radius_ = this->get_parameter("prediction_filter_radius").as_int();
  master_position_stack_ = Position(input_stack_length_);
  previous_command_stack_ = Position(output_stack_length_);
  
  joint_increment_ = 
      robot::Deg2Rad<double>(this->get_parameter("joint_increment").as_double());
  position_increment_ = 
      this->get_parameter("position_increment").as_double();
  orientation_increment_ = 
      robot::Deg2Rad<double>(this->get_parameter("orientation_increment").as_double());
  std::string trajectory_0_filepath = 
      prefix + this->get_parameter("trajectory_0_filepath").as_string();
  std::string trajectory_1_filepath = 
      prefix + this->get_parameter("trajectory_1_filepath").as_string();
  if (trajectory_0_filepath != "") LoadTrajectory(trajectory_0_filepath);
  if (trajectory_1_filepath != "") LoadTrajectory(trajectory_1_filepath);

  // Setup ros node
  callback_group_sub_joint_state_ = 
      this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  callback_group_sub_pose_state_ = 
      this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  callback_group_sub_key_value_ = 
      this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  callback_group_sub_trajectory_prediction_list_ =
      this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  callback_group_sub_advance_q_ = 
      this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  auto sub_joint_state_opt = rclcpp::SubscriptionOptions();
  auto sub_pose_state_opt = rclcpp::SubscriptionOptions();
  auto sub_key_value_opt = rclcpp::SubscriptionOptions();
  auto sub_trajectory_prediction_opt = rclcpp::SubscriptionOptions();
  auto sub_advance_q_opt = rclcpp::SubscriptionOptions();
  sub_joint_state_opt.callback_group = callback_group_sub_joint_state_;
  sub_pose_state_opt.callback_group = callback_group_sub_pose_state_;
  sub_key_value_opt.callback_group = callback_group_sub_key_value_;
  sub_trajectory_prediction_opt.callback_group = callback_group_sub_trajectory_prediction_list_;
  sub_advance_q_opt.callback_group = callback_group_sub_advance_q_;

  sub_joint_state_ = this->create_subscription<messages::msg::JointState>(
      sub_joint_state_topic_name, 1,
      std::bind(&CommanderNode::JointStateCallback, this, std::placeholders::_1),
      sub_joint_state_opt); 
  sub_pose_state_ = this->create_subscription<messages::msg::PoseState>(
      sub_pose_state_topic_name, 1,
      std::bind(&CommanderNode::PoseStateCallback, this, std::placeholders::_1),
      sub_pose_state_opt);
  sub_key_value_ = this->create_subscription<std_msgs::msg::Int16>(
      "key_value", 1,
      std::bind(&CommanderNode::KeyValueCallback, this, std::placeholders::_1),
      sub_key_value_opt);
  sub_trajectory_prediction_list_ = this->create_subscription<messages::msg::MultiDoubleArrayStamped>(
      sub_trajectory_prediction_list_topic_name, 1,
      std::bind(&CommanderNode::TrajectoryPredictionCallback, this, std::placeholders::_1),
      sub_trajectory_prediction_opt);
  sub_advance_q_ = this->create_subscription<std_msgs::msg::Int16>(
      sub_advance_q_topic_name, 1,
      std::bind(&CommanderNode::AdvanceCallback, this, std::placeholders::_1),
      sub_advance_q_opt);
  pub_joint_command_ = this->create_publisher<messages::msg::JointCommand>(
      pub_joint_command_topic_name, 1); 
  pub_pose_command_ = this->create_publisher<messages::msg::PoseCommand>(
      pub_pose_command_topic_name, 1); 
  pub_pose_raw_ = this->create_publisher<messages::msg::PoseCommand>(
      pub_pose_raw_topic_name, 1); 
  pub_trajectory_prediction_list_ = this->create_publisher<messages::msg::MultiDoubleArrayStamped>(
      pub_trajectory_prediction_list_topic_name, 1);

  // setup threads
  task_thread_ = new std::thread(std::bind(&CommanderNode::TaskCallback, this));
  tele_thread_ = new std::thread(std::bind(&CommanderNode::TeleCallback, this));
  time_thread_ = new std::thread(std::bind(&CommanderNode::TimeCallback, this));
  task_thread_->detach();
  tele_thread_->detach();
  time_thread_->detach();
}

CommanderNode::~CommanderNode() {
  delete task_thread_;
  delete time_thread_;
  delete tele_thread_;
}

// in TASK mode, need not to ask predictor to make prediction, but just
// send for an advance_q(without computational delay) as index in stored
// trajectory.
void CommanderNode::TaskCallback() {
  int task_step = 0;
  while (rclcpp::ok()) {
    if (mode_ == ControlMode::TASK) {
      double base_time = current_time_;
      double end_time = base_time + 1000.0/frequency_;
      int max_step = trajectory_[current_trajectory_].x.size();
      Eigen::Vector3d position_raw;
      Eigen::Vector3d position_command;
      position_raw[0] = trajectory_[current_trajectory_].x[task_step];
      position_raw[1] = trajectory_[current_trajectory_].y[task_step];
      position_raw[2] = trajectory_[current_trajectory_].z[task_step];
      PublishPoseRaw(position_raw, current_slave_orientation_);
      if (prediction_mode_ == PredictionMode::NONE || prediction_mode_ == PredictionMode::PASSIVE) {
        position_command = position_raw;
      } else if (prediction_mode_ == PredictionMode::TRANS ||
                 prediction_mode_ == PredictionMode::TRANS_COMPU) {
        int prediction_step = std::min(task_step+current_advance_q_, max_step-1);
        position_command[0] = trajectory_[current_trajectory_].x[prediction_step];
        position_command[1] = trajectory_[current_trajectory_].y[prediction_step];
        position_command[2] = trajectory_[current_trajectory_].z[prediction_step];
      } else if (prediction_mode_ == PredictionMode::TRANS_COMPU_MECHA) {
        int prediction_step = 
            std::min(task_step+current_advance_q_
                     +int(mechanical_delay_*frequency_/1000.), max_step-1);
        position_command[0] = trajectory_[current_trajectory_].x[prediction_step];
        position_command[1] = trajectory_[current_trajectory_].y[prediction_step];
        position_command[2] = trajectory_[current_trajectory_].z[prediction_step];
      }
      PublishPoseCommand(position_command, current_slave_orientation_);
      task_step++;
      if (task_step >= max_step) {
        SwitchMode(ControlMode::JOG);
        task_step = 0;
      }
      int sleep_time;
      if (end_time > current_time_) {
        sleep_time = static_cast<int>(end_time-current_time_)*1000;
      } else {
        sleep_time = 0;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(sleep_time));
    } else {
      task_step = 0;
    }
  }
}

void CommanderNode::TimeCallback() {
  while (rclcpp::ok()) {
    mutex_->lock();
    current_time_ = clock_.GetTime_ms();
    mutex_->unlock();
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}

void CommanderNode::TeleCallback() {
  Eigen::Vector3d start_position; // start position of slave robot
  Eigen::Vector3d base_position; // base position of master robot when button pressed
  bool start_teleoperation = false;
  while (rclcpp::ok()) {
    double base_time = current_time_;
    double end_time = base_time + 1000.0/frequency_;
    // update master state
    if (omega_control_->is_initialized()) {
      current_master_position_ = omega_control_->GetPosition();
      current_master_button_ = omega_control_->GetButton();
    }
    // send command
    if (mode_ == ControlMode::TELE && omega_control_->is_initialized()) {
      if (current_master_button_ && !start_teleoperation) {
        start_teleoperation = true;
        start_position = current_slave_position_;
        base_position = current_master_position_;
      } else if (!current_master_button_) {
        start_teleoperation = false;
        master_position_stack_ = Position(input_stack_length_);
      } else {
        master_position_stack_.update(current_master_position_[0]-base_position[0],
                                      current_master_position_[1]-base_position[1],
                                      current_master_position_[2]-base_position[2]);
        PublishTrajectoryInput();
        Eigen::Vector3d position_raw = start_position + current_master_position_ - base_position;
        Eigen::Vector3d position_command;
        PublishPoseRaw(position_raw, current_slave_orientation_);
        if (prediction_mode_ == PredictionMode::NONE || prediction_mode_ == PredictionMode::PASSIVE) {
          position_command = position_raw;
        } else if (prediction_mode_ == PredictionMode::TRANS) {
          position_command = start_position + current_prediction_position_T_;
        } else if (prediction_mode_ == PredictionMode::TRANS_COMPU) {
          position_command = start_position + current_prediction_position_TC_;
        } else if (prediction_mode_ == PredictionMode::TRANS_COMPU_MECHA) {
          position_command = start_position + current_prediction_position_TCM_;
        }
        PublishPoseCommand(position_command, current_slave_orientation_);
      }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int>(end_time-current_time_)*1000));
  }
}

void CommanderNode::AdvanceCallback(const std_msgs::msg::Int16::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(*mutex_); 
  current_advance_q_ = msg->data;
}

void CommanderNode::DeclareRosParameters() {
  this->declare_parameter("sub_joint_state_topic_name");
  this->declare_parameter("sub_pose_state_topic_name");
  this->declare_parameter("sub_advance_q_topic_name");
  this->declare_parameter("pub_joint_command_topic_name");
  this->declare_parameter("pub_pose_command_topic_name");
  this->declare_parameter("frequency");
  this->declare_parameter("sub_trajectory_prediction_list_topic_name");
  this->declare_parameter("pub_trajectory_prediction_list_topic_name");
  this->declare_parameter("pub_pose_raw_topic_name");

  this->declare_parameter("joint_increment");
  this->declare_parameter("orientation_increment");
  this->declare_parameter("position_increment");
  this->declare_parameter("trajectory_0_filepath");
  this->declare_parameter("trajectory_1_filepath");
  this->declare_parameter("input_stack_length");
  this->declare_parameter("mechanical_delay");
  this->declare_parameter("output_stack_length");
  this->declare_parameter("prediction_filter_radius");
}

bool CommanderNode::LoadTrajectory(const std::string& file_path) {
  std::vector<std::string> str;
  Position traj;
  if (!LoadStringFromFile(file_path, &str, "#")) {
    RCLCPP_ERROR(this->get_logger(),"Fail to read trajectory file: %s", file_path.c_str());
    return false;
  }
  for (int i = 1; i < str.size(); ++i) {
    std::vector<double> values = GetValuesFromStringSplit<double>(str[i], ",");
    traj.x.push_back(values[1]);
    traj.y.push_back(values[2]);
    traj.z.push_back(values[3]);
  }
  RCLCPP_INFO(this->get_logger(),"Load trajectory from file: %s", file_path.c_str());
  is_trajectory_loaded_ = true;
  trajectory_.push_back(traj);
  return true;
}

void CommanderNode::JointStateCallback(
    const messages::msg::JointState::SharedPtr msg) {
  std::vector<double> angles = msg->angles;
  std::vector<double> velocities = msg->velocities;

  std::lock_guard<std::mutex> lock(*mutex_);
  current_slave_joint_angles_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      angles.data(), angles.size());
  current_slave_joint_velocities_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      velocities.data(), velocities.size());
  // std::cout << "Received: Current joint angles: " << current_slave_joint_angles_.transpose() << std::endl;
}

void CommanderNode::PoseStateCallback(
    const messages::msg::PoseState::SharedPtr msg) {
  double x = msg->pose.position.x;
  double y = msg->pose.position.y;
  double z = msg->pose.position.z;
  double qx = msg->pose.orientation.x;
  double qy = msg->pose.orientation.y;
  double qz = msg->pose.orientation.z;
  double qw = msg->pose.orientation.w;

  std::lock_guard<std::mutex> lock(*mutex_);
  current_slave_position_ = Eigen::Vector3d(x, y, z);
  // std::cout << "Received: x y z :" << x << y << z <<std::endl;

  Eigen::Quaterniond q = Eigen::Quaterniond(qw, qx, qy, qz);
  robot::Quaternion2RPY(q, &current_slave_orientation_);
}

void CommanderNode::KeyValueCallback(const std_msgs::msg::Int16::SharedPtr msg) {
  int key_value = msg->data;
  ProcessKey(key_value);
}

void CommanderNode::TrajectoryPredictionCallback(
    const messages::msg::MultiDoubleArrayStamped::SharedPtr msg) {
  count_trajectory_prediction_ += 1;
  double time_prediction_start = msg->timestamp;
  double time_prediction_end = current_time_;
  double delay_computation = time_prediction_end - time_prediction_start;
  // RCLCPP_INFO(this->get_logger(),"computation delay: %f", delay_computation);

  average_delay_computation_ = (average_delay_computation_ * (count_trajectory_prediction_-1) 
                              + delay_computation)/count_trajectory_prediction_;
  
  int output_length = msg->data[0].data.size();
  int index_T = std::min(current_advance_q_,output_length-1);
  int index_TC = std::min(current_advance_q_
                 +static_cast<int>(average_delay_computation_*frequency_/1000.),output_length-1);
  int index_TCM = std::min(current_advance_q_
                 +static_cast<int>(average_delay_computation_*frequency_/1000.)
                 +static_cast<int>(mechanical_delay_*frequency_/1000.),output_length-1);

  // Position temp_stack = Position();
  // temp_stack.x = msg->data[0].data;
  // temp_stack.y = msg->data[1].data;
  // temp_stack.z = msg->data[2].data;

  // double sum_x,sum_y,sum_z = 0.0;
  // double sum_x_T,sum_y_T,sum_z_T = 0.0;
  // double sum_x_TC,sum_y_TC,sum_z_TC = 0.0;
  // double sum_x_TCM,sum_y_TCM,sum_z_TCM = 0.0;
  // int length_T, length_TC, length_TCM, length = 0;
  // previous_command_stack_.sum(output_stack_length_-prediction_filter_radius_,output_stack_length_,&sum_x,&sum_y,&sum_z,&length,false);
  // temp_stack.sum(index_T-prediction_filter_radius_,
  //                index_T+prediction_filter_radius_,
  //                &sum_x_T,&sum_y_T,&sum_z_T,&length_T,false);
  // temp_stack.sum(index_TC-prediction_filter_radius_,
  //                index_TC+prediction_filter_radius_,
  //                &sum_x_TC,&sum_y_TC,&sum_z_TC,&length_TC,false);
  // temp_stack.sum(index_TCM-prediction_filter_radius_,
  //                index_TCM+prediction_filter_radius_,
  //                &sum_x_TCM,&sum_y_TCM,&sum_z_TCM,&length_TCM,false);
  // current_prediction_position_T_ = Eigen::Vector3d(sum_x_T/length_T,
  //                                                  sum_y_T/length_T,
  //                                                  sum_z_T/length_T);
  // current_prediction_position_TC_ = Eigen::Vector3d(sum_x_TC/length_TC,
  //                                                   sum_y_TC/length_TC,
  //                                                   sum_z_TC/length_TC);
  // current_prediction_position_TCM_ = Eigen::Vector3d(sum_x_TCM/length_TCM,
  //                                                    sum_y_TCM/length_TCM,
  //                                                    sum_z_TCM/length_TCM);

  // current_prediction_position_T_ = Eigen::Vector3d((sum_x_T+sum_x)/(length_T+length),
  //                                                  (sum_y_T+sum_y)/(length_T+length),
  //                                                  (sum_z_T+sum_z)/(length_T+length));
  // current_prediction_position_TC_ = Eigen::Vector3d((sum_x_TC+sum_x)/(length_TC+length),
  //                                                  (sum_y_TC+sum_y)/(length_TC+length),
  //                                                  (sum_z_TC+sum_z)/(length_TC+length));
  // current_prediction_position_TCM_ = Eigen::Vector3d((sum_x_TCM+sum_x)/(length_TCM+length),
  //                                                  (sum_y_TCM+sum_y)/(length_TCM+length),
  //                                                  (sum_z_TCM+sum_z)/(length_TCM+length));

  current_prediction_position_T_ = Eigen::Vector3d(msg->data[0].data[index_T],
                                                   msg->data[1].data[index_T],
                                                   msg->data[2].data[index_T]);
  current_prediction_position_TC_ = Eigen::Vector3d(msg->data[0].data[index_TC],
                                                   msg->data[1].data[index_TC],
                                                   msg->data[2].data[index_TC]);                                                 
  current_prediction_position_TCM_ = Eigen::Vector3d(msg->data[0].data[index_TCM],
                                                   msg->data[1].data[index_TCM],
                                                   msg->data[2].data[index_TCM]);
}

template<class T>
void CommanderNode::ModifyCommand(int index, double increment,
                                 T* command) {
  (*command)[index] += increment;
}

void CommanderNode::PublishJointCommand(Eigen::VectorXd joint_command) {
  messages::msg::JointCommand msg;
  std::vector<double> joint_angles(
      &joint_command[0],
      joint_command.data() + joint_command.cols() * joint_command.rows());
  msg.angles = joint_angles;
  pub_joint_command_->publish(msg);
}

void CommanderNode::PublishPoseCommand(Eigen::Vector3d position_command,
                                      Eigen::Vector3d orientation_command) {
  previous_command_stack_.update(position_command.x(),
                                 position_command.y(),
                                 position_command.z());
  messages::msg::PoseCommand msg;
  Eigen::Quaterniond q;
  robot::RPY2Quaternion(orientation_command, &q);
  msg.pose.position.x = position_command.x();
  msg.pose.position.y = position_command.y();
  msg.pose.position.z = position_command.z();
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();
  pub_pose_command_->publish(msg);
}

void CommanderNode::PublishTrajectoryInput() {
  messages::msg::MultiDoubleArrayStamped msg;
  messages::msg::DoubleArray x_msg,y_msg,z_msg;
  for (int i = 0; i < master_position_stack_.x.size(); ++i) {
    x_msg.data.push_back(master_position_stack_.x[i]);
    y_msg.data.push_back(master_position_stack_.y[i]);
    z_msg.data.push_back(master_position_stack_.z[i]);
  }
  msg.data.push_back(x_msg);
  msg.data.push_back(y_msg);
  msg.data.push_back(z_msg);

  msg.timestamp = current_time_;
  pub_trajectory_prediction_list_->publish(msg);
}

void CommanderNode::PublishPoseRaw(Eigen::Vector3d position_raw,
                                   Eigen::Vector3d orientation_raw) {
  messages::msg::PoseCommand msg;
  Eigen::Quaterniond q;
  robot::RPY2Quaternion(orientation_raw, &q);
  msg.pose.position.x = position_raw.x();
  msg.pose.position.y = position_raw.y();
  msg.pose.position.z = position_raw.z();
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();
  pub_pose_raw_->publish(msg);
}

bool CommanderNode::CheckInTaskStartPosition(int trajectory_index) {
  if (((trajectory_[trajectory_index].x[0] - current_slave_position_[0])
      *(trajectory_[trajectory_index].x[0] - current_slave_position_[0])
      +(trajectory_[trajectory_index].y[0] - current_slave_position_[1])
      *(trajectory_[trajectory_index].y[0] - current_slave_position_[1])
      +(trajectory_[trajectory_index].z[0] - current_slave_position_[2])
      *(trajectory_[trajectory_index].z[0] - current_slave_position_[2])) < 3) {
    return true;
  }
  return false;
}

void CommanderNode::SwitchMode(ControlMode mode) {
  switch (mode) {
    case ControlMode::JOG: {
      if (mode_ != ControlMode::JOG) {
        mode_ = ControlMode::JOG;
        omega_control_->SetGripperForce(0);
        RCLCPP_INFO(this->get_logger(),"Switch to JOG mode.");
        if (prediction_mode_ != PredictionMode::NONE) {
          prediction_mode_ = PredictionMode::NONE;
          RCLCPP_INFO(this->get_logger(),"Prediction is off.");
        }
      }
      break;
    }
    case ControlMode::TASK: {
      if (mode_ != ControlMode::TASK) {
        mode_ = ControlMode::TASK;
        omega_control_->SetGripperForce(0);
        RCLCPP_INFO(this->get_logger(),"Switch to TASK mode.");
      }
      break;
    }
    case ControlMode::TELE: {
      if (mode_ != ControlMode::TELE) {
        mode_ = ControlMode::TELE;
        omega_control_->SetGripperForce(1);
        RCLCPP_INFO(this->get_logger(),"Switch to TELE mode.");
      }
      break;
    }
    default:
      break;
  }
}

void CommanderNode::ProcessKey(int key_value) {
  // the initialization should be placed here, since the scankey
  // is excuted first and wait until key pressed, but the member 
  // varibles takes a while for subscription
  Eigen::VectorXd joint_command = current_slave_joint_angles_;
  Eigen::Vector3d position_command = current_slave_position_;
  Eigen::Vector3d orientation_command = current_slave_orientation_;
  switch (key_value) {
    // joint command
    case KeyValues::J0_MINUS: {
      ModifyCommand(0, -joint_increment_, &joint_command);
      PublishJointCommand(joint_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    case KeyValues::J0_PLUS: {
      ModifyCommand(0, joint_increment_, &joint_command);
      PublishJointCommand(joint_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    case KeyValues::J1_MINUS: {
      ModifyCommand(1, -joint_increment_, &joint_command);
      PublishJointCommand(joint_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    case KeyValues::J1_PLUS: {
      ModifyCommand(1, joint_increment_, &joint_command);
      PublishJointCommand(joint_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    case KeyValues::J2_MINUS: {
      ModifyCommand(2, -joint_increment_, &joint_command);
      PublishJointCommand(joint_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    case KeyValues::J2_PLUS: {
      ModifyCommand(2, joint_increment_, &joint_command);
      PublishJointCommand(joint_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    case KeyValues::J3_MINUS: {
      ModifyCommand(3, -joint_increment_, &joint_command);
      PublishJointCommand(joint_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    case KeyValues::J3_PLUS: {
      ModifyCommand(3, joint_increment_, &joint_command);
      PublishJointCommand(joint_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    case KeyValues::J4_MINUS: {
      ModifyCommand(4, -joint_increment_, &joint_command);
      PublishJointCommand(joint_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    case KeyValues::J4_PLUS: {
      ModifyCommand(4, joint_increment_, &joint_command);
      PublishJointCommand(joint_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    case KeyValues::J5_MINUS: {
      ModifyCommand(5, -joint_increment_, &joint_command);
      PublishJointCommand(joint_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    case KeyValues::J5_PLUS: {
      ModifyCommand(5, joint_increment_, &joint_command);
      PublishJointCommand(joint_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    case KeyValues::J6_MINUS: {
      ModifyCommand(6, -joint_increment_, &joint_command);
      PublishJointCommand(joint_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    case KeyValues::J6_PLUS: {
      ModifyCommand(6, joint_increment_, &joint_command);
      PublishJointCommand(joint_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    // pose command
    case KeyValues::X_MINUS: {
      ModifyCommand(0, -position_increment_, &position_command);
      PublishPoseCommand(position_command, orientation_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    case KeyValues::X_PLUS: {
      ModifyCommand(0, position_increment_, &position_command);
      PublishPoseCommand(position_command, orientation_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    // pose command
    case KeyValues::Y_MINUS: {
      ModifyCommand(1, -position_increment_, &position_command);
      PublishPoseCommand(position_command, orientation_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    case KeyValues::Y_PLUS: {
      ModifyCommand(1, position_increment_, &position_command);
      PublishPoseCommand(position_command, orientation_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    // pose command
    case KeyValues::Z_MINUS: {
      ModifyCommand(2, -position_increment_, &position_command);
      PublishPoseCommand(position_command, orientation_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    case KeyValues::Z_PLUS: {
      ModifyCommand(2, position_increment_, &position_command);
      PublishPoseCommand(position_command, orientation_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    // pose command
    case KeyValues::ROLL_MINUS: {
      ModifyCommand(0, -orientation_increment_, &orientation_command);
      PublishPoseCommand(position_command, orientation_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    case KeyValues::ROLL_PLUS: {
      ModifyCommand(0, orientation_increment_, &orientation_command);
      PublishPoseCommand(position_command, orientation_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    // pose command
    case KeyValues::PITCH_MINUS: {
      ModifyCommand(1, -orientation_increment_, &orientation_command);
      PublishPoseCommand(position_command, orientation_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    case KeyValues::PITCH_PLUS: {
      ModifyCommand(1, orientation_increment_, &orientation_command);
      PublishPoseCommand(position_command, orientation_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    // pose command
    case KeyValues::YAW_MINUS: {
      ModifyCommand(2, -orientation_increment_, &orientation_command);
      PublishPoseCommand(position_command, orientation_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    case KeyValues::YAW_PLUS: {
      ModifyCommand(2, orientation_increment_, &orientation_command);
      PublishPoseCommand(position_command, orientation_command);
      SwitchMode(ControlMode::JOG);
      break;
    }
    case KeyValues::HUMAN: {
      SwitchMode(ControlMode::TELE);
      break;
    }
    case KeyValues::TRAJ_0: {
      if (!is_trajectory_loaded_) {
        RCLCPP_ERROR(this->get_logger(),"No trajectory loaded!");
      }
      current_trajectory_ = 0;
      if (mode_!=ControlMode::TASK && !CheckInTaskStartPosition(current_trajectory_)) {
        position_command[0] = trajectory_[current_trajectory_].x[0];
        position_command[1] = trajectory_[current_trajectory_].y[0];
        position_command[2] = trajectory_[current_trajectory_].z[0];
        PublishPoseCommand(position_command, orientation_command);
        RCLCPP_INFO(this->get_logger(),"Moving to trajectory 0 start point.");
        SwitchMode(ControlMode::JOG);
      } else {
        SwitchMode(ControlMode::TASK);
      }
      break;
    }
    case KeyValues::TRAJ_1: {
      if (!is_trajectory_loaded_) {
        RCLCPP_ERROR(this->get_logger(),"No trajectory loaded!");
      }
      current_trajectory_ = 1;
      if (mode_!=ControlMode::TASK && !CheckInTaskStartPosition(current_trajectory_)) {
        position_command[0] = trajectory_[current_trajectory_].x[0];
        position_command[1] = trajectory_[current_trajectory_].y[0];
        position_command[2] = trajectory_[current_trajectory_].z[0];
        PublishPoseCommand(position_command, orientation_command);
        RCLCPP_INFO(this->get_logger(),"Moving to trajectory 1 start point.");
        SwitchMode(ControlMode::JOG);
      } else {
        SwitchMode(ControlMode::TASK);
      }
      break;
    }
    case KeyValues::PRE_OFF: {
      if (prediction_mode_ != PredictionMode::NONE) {
        prediction_mode_ = PredictionMode::NONE;
        RCLCPP_INFO(this->get_logger(),"Prediction is off.");
      }
      break;
    }
    case KeyValues::PRE_T: {
      if (prediction_mode_ != PredictionMode::TRANS) {
        prediction_mode_ = PredictionMode::TRANS;
        RCLCPP_INFO(this->get_logger(),"Use Transmission delay compensation.");
      }
      break;
    }
    case KeyValues::PRE_TC: {
      if (prediction_mode_ != PredictionMode::TRANS_COMPU) {
        prediction_mode_ = PredictionMode::TRANS_COMPU;
        RCLCPP_INFO(this->get_logger(),"Use Transmission+Computation delay compensation.");
      }
      break;
    }
    case KeyValues::PRE_TCM: {
      if (prediction_mode_ != PredictionMode::TRANS_COMPU_MECHA) {
        prediction_mode_ = PredictionMode::TRANS_COMPU_MECHA;
        RCLCPP_INFO(this->get_logger(),"Use Transmission+Computation_Mechanical delay compensation.");
      }
      break;
    }
    case KeyValues::PRE_PASSIVE: {
      if (prediction_mode_ != PredictionMode::PASSIVE) {
        prediction_mode_ = PredictionMode::PASSIVE;
        RCLCPP_INFO(this->get_logger(),"Use Passive compensation.");
      }
      break;
    }
    default: {
      break;
    }
  }
}