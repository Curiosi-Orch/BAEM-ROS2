#ifndef IIWA_CONTROL_H_
#define IIWA_CONTROL_H_

#include "iiwa/planner/kukasunriseplanned.h"
#include "iiwa/sunrise/kukasunrise.h"

class IIwaControl {
 public:
  IIwaControl(KukaSunrisePlanned::InitialParameters params);
  ~IIwaControl();

  Eigen::MatrixXd GetCurrentTransform();
  Eigen::VectorXd GetCurrentVelocity();
  Eigen::VectorXd GetCurrentJointAngles();
  Eigen::VectorXd GetCurrentJointVelocities();
  void SetPose(const Eigen::MatrixXd& T_destination);
  bool SetJointAngles(const Eigen::VectorXd& q);

  void SetWorkspaceLimits(std::vector<Eigen::Vector3d> limits);
  void SetJointAngleLimits(std::vector<Eigen::VectorXd> limits);
  void SetJointVelocityLimits(Eigen::VectorXd qd_max);
  bool LoadHomeJointAngles(const std::string& file_path);
  bool LoadDHTable(const std::string& file_path);

  void UpdataStateCallback();

  bool stop() { return iiwa_.stop(); }
  bool is_connected() const { return is_connected_; }
  Eigen::VectorXd q_init() const { return q_init_; }
  Eigen::MatrixXd T_init() const { return T_init_; }

 private:
  KukaSunrisePlanned::InitialParameters params_;
  KukaSunrisePlanned iiwa_;
  bool is_connected_;
  Eigen::VectorXd q_init_;
  Eigen::MatrixXd T_init_;

  Eigen::VectorXd q_home_;
  std::vector<char> joint_type_;
  Eigen::MatrixXd DH_table_;
  int num_joints_;
  std::vector<Eigen::Vector3d> workspace_limits_;
  std::vector<Eigen::VectorXd> joint_angle_limits_;
  Eigen::VectorXd joint_velocity_limits_;

  std::thread* update_state_thread_;
  std::mutex* mutex_;

  Eigen::MatrixXd current_transform_;
  Eigen::VectorXd current_velocity_;
  Eigen::VectorXd current_joint_angles_;
  Eigen::VectorXd current_joint_velocities_;

};

#endif  // IIWA_CONTROL_H_
