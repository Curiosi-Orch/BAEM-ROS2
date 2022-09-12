#ifndef OMEGA_CONTROL_H_
#define OMEGA_CONTROL_H_


#include "dhdc.h"
#include "drdc.h"
#include "utils/robot_utilities.h"

class OmegaControl {
 public:
  OmegaControl();
  ~OmegaControl();

  void UpdateStateCallback();

  Eigen::Vector3d GetPosition();
  bool GetButton();
  bool Initialize();
  bool is_initialized();
  bool SetGripperForce(double force);

 private:
  Eigen::Vector3d current_position_;
  Eigen::Vector3d current_velocity_;
  Eigen::Vector3d current_orientation_;
  Eigen::Vector3d current_force_;
  bool current_button_;
  bool is_initialized_;

  std::thread* state_thread_;
};

#endif  // OMEGA_CONTROL_H_