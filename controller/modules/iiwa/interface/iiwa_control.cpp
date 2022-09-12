#include "iiwa/interface/iiwa_control.h"
#include "utils/robot_utilities.h"

IIwaControl::IIwaControl(KukaSunrisePlanned::InitialParameters params)
    : params_(params),
      is_connected_(false),
      q_init_(7),
      q_home_(7),
      joint_angle_limits_(2),
      workspace_limits_(2) {
  mutex_ = new std::mutex();
  is_connected_ = iiwa_.start(params_);
  Erl::sleep_ms(1000);
  T_init_ = Eigen::MatrixXd::Identity(4, 4);
  num_joints_ = joint_type_.size();
  DH_table_.resize(num_joints_, 4);
  joint_type_ = std::vector<char>(7, 'r');
  
  // joint_activation_matrix_ = Eigen::MatrixXd::Identity(7, 7);
  joint_angle_limits_[0] = robot::Deg2Rad<Eigen::VectorXd>(Eigen::VectorXd::Ones(num_joints_) * (-180));
  joint_angle_limits_[1] = robot::Deg2Rad<Eigen::VectorXd>(Eigen::VectorXd::Ones(num_joints_) * 180);
  joint_velocity_limits_ = robot::Deg2Rad<Eigen::VectorXd>(Eigen::VectorXd::Ones(num_joints_) * 1e6);

  if (is_connected_) {
    T_init_ = GetCurrentTransform();
    q_init_ = iiwa_.getJoints();
    std::cout << "=== IIWA CONNECTED! INITIAL STATUS ===" << std::endl;
    std::cout << "initial pose :\n" << T_init_ << std::endl;
    std::cout << "initial joint angles :\n" << q_init_.transpose() << "\n";
    update_state_thread_ = new std::thread(std::bind(&IIwaControl::UpdataStateCallback, this));
    update_state_thread_->detach();
  } else {
    std::cout << "=== NO KUKA CONNECTION ===" << std::endl;
  }
}

IIwaControl::~IIwaControl() {
  delete update_state_thread_;
}

void IIwaControl::UpdataStateCallback() {
  while (is_connected_) {
    mutex_->lock();
    current_joint_angles_ = iiwa_.getJoints();
    current_joint_velocities_ = iiwa_.getCurrentJointVelocity();
    current_transform_ = robot::Erl2Eigen(iiwa_.getMsrTransform());
    current_velocity_ = iiwa_.getCurrentVelocity();
    mutex_->unlock();

    std::this_thread::sleep_for(std::chrono::microseconds(1000));
  }
}

Eigen::VectorXd IIwaControl::GetCurrentJointAngles() {
  std::lock_guard<std::mutex> lock(*mutex_);
  return current_joint_angles_;
}

Eigen::MatrixXd IIwaControl::GetCurrentTransform() {
  std::lock_guard<std::mutex> lock(*mutex_);
  return current_transform_;
}

Eigen::VectorXd IIwaControl::GetCurrentVelocity() {
  std::lock_guard<std::mutex> lock(*mutex_);
  return current_velocity_;
}

Eigen::VectorXd IIwaControl::GetCurrentJointVelocities() {
  std::lock_guard<std::mutex> lock(*mutex_);
  return current_joint_velocities_;
}

bool IIwaControl::SetJointAngles(const Eigen::VectorXd& q) {
  iiwa_.setJoints(q);
  return true;
}

void IIwaControl::SetPose(const Eigen::MatrixXd& T_destination) {
  Erl::Transformd destination_transform = robot::Eigen2Erl(T_destination);

  // Eigen::VectorXd q_now = GetCurrentJointAngles();
  // Eigen::VectorXd axis_filter(6);
  // axis_filter << 1, 1, 1, 0, 0, 0;
  // Eigen::VectorXd q_out =
  //     robot::InverseKinematics(DH_table_, T_destination, q_now,
  //                              joint_angle_limits_, joint_type_, axis_filter);
  // iiwa_.setJoints(q_out);

  iiwa_.setTransform(destination_transform);
}

bool IIwaControl::LoadHomeJointAngles(const std::string& file_path) {
  Eigen::MatrixXd data;
  if (!robot::LoadMatrixFromFile(file_path, &data)) {
    std::cout << "Home joint angles file not exists: " << file_path
              << std::endl;
    return false;
  } else {
    q_home_ = data.row(0);
    std::cout << "Home joint angles loaded: " << q_home_.transpose()
              << std::endl;
  }
  return true;
}

bool IIwaControl::LoadDHTable(const std::string& file_path) {
  if (robot::ReadDHTable(file_path, &DH_table_)) {
    return true;
  }
  return false;
}

void IIwaControl::SetWorkspaceLimits(std::vector<Eigen::Vector3d> limits) {
  for (int i = 0; i < 2; i++) {
    workspace_limits_[i].resize(3);
    workspace_limits_[i] = limits[i];
  }
}

void IIwaControl::SetJointAngleLimits(std::vector<Eigen::VectorXd> limits) {
  for (int i = 0; i < 2; i++) {
    joint_angle_limits_[i].resize(num_joints_);
    joint_angle_limits_[i] = limits[i];
  }
}

void IIwaControl::SetJointVelocityLimits(Eigen::VectorXd qd_max) {
  joint_velocity_limits_ = qd_max;
}
