#include "omega/omega_control.h"

OmegaControl::OmegaControl() : is_initialized_(false) {
  Initialize();

  if (is_initialized_) {
    state_thread_ = new std::thread(std::bind(&OmegaControl::UpdateStateCallback,this));
    state_thread_->detach();
  } else {
    std::cout << "Fail to initialize master robot!" << std::endl;
  }
}

OmegaControl::~OmegaControl() {
  drdClose ();
}

void OmegaControl::UpdateStateCallback() {
  double px, py, pz;
  double fx, fy, fz;
  double vx, vy, vz;
  double rx, ry, rz, rb;
  double time_start = dhdGetTime ();
  bool done = false;
  double button_thresh = 0.1;
  // haptic loop
  while (!done) {
    // display refresh rate and position at 10kHz
    double time_now = dhdGetTime ();
    if ((time_now-time_start) > 0.0001) {
      // update timestamp
      time_start = time_now;

      // write down position
      if (dhdGetPosition(&px, &py, &pz) < 0) {
        std::cout << "error: cannot read position (" << dhdErrorGetLastStr() << ")" << std::endl;
        done = true;
      }
      if (dhdGetForce(&fx, &fy, &fz) < 0) {
        std::cout << "error: cannot read force (" << dhdErrorGetLastStr() << ")" << std::endl;
        done = true;
      }
      if (dhdGetLinearVelocity(&vx, &vy, &vz) < 0) {
        std::cout << "error: cannot read velocity (" << dhdErrorGetLastStr() << ")" << std::endl;
        done = true;
      }
      if (dhdGetOrientationDeg(&rx, &ry, &rz) < 0) {
        std::cout << "error: cannot read orientation (" << dhdErrorGetLastStr() << ")" << std::endl;
      }
      dhdGetGripperAngleDeg(&rb);
      if (rb <= button_thresh) {
        current_button_ = true;
      } else {
        current_button_ = false;
      }
      current_position_ = Eigen::Vector3d(px,py,pz);
      current_velocity_ = Eigen::Vector3d(vx,vy,vz);
      current_force_ = Eigen::Vector3d(fx,fy,fz);
      current_orientation_ = Eigen::Vector3d(rx,ry,rz);
      // std::cout << "pose: (" << px << "," << py << "," << pz 
      //           << " | " << rx << "," << ry << "," << rz << ")\t"
      //           << "velocity: (" << vx << "," << vy << "," << vz << ")\t"
      //           << "button: " << current_button_ << std::endl;
    }
  }
}

Eigen::Vector3d OmegaControl::GetPosition() {
  return current_position_*1000.;
}

bool OmegaControl::GetButton() {
  return current_button_;
}

bool OmegaControl::is_initialized() {
  return is_initialized_;
}

bool OmegaControl::SetGripperForce(double force) {
  if (dhdSetForceAndTorqueAndGripperForce (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, force) < DHD_NO_ERROR) {
    std::cout << "error: cannot set force (" << dhdErrorGetLastStr() << ")\n";
    return false;
  }
  return true;
}

bool OmegaControl::Initialize() {
  // required to change asynchronous operation mode
  // dhdEnableExpertMode ();
  
  // open the first available device
  if (drdOpen () < 0) {
    std::cout << "error: cannot open device (" << dhdErrorGetLastStr() << ")" << std::endl;
    dhdSleep (2.0);
    return false;
  }

  // print out device identifier
  if (!drdIsSupported()) {
    std::cout << "unsupported device" << std::endl;
    dhdSleep (2.0);
    drdClose ();
    return false;
  }
  std::cout << dhdGetSystemName() << " haptic device detected." << std::endl;

  // perform auto-initialization
  std::cout << "initializing..." << std::endl;
  if (drdAutoInit () < 0) {
    std::cout << "error: auto-initialization failed (" << dhdErrorGetLastStr() << ")\n";
    drdClose ();
    dhdSleep (2.0);
    return false;
  }

  // perform initialization check (optional)
  if (drdCheckInit () < 0) {
    std::cout << "error: device initialization check failed (" << dhdErrorGetLastStr() << ")\n";
    drdClose ();
    dhdSleep (2.0);
    return false;
  }

  // report success
  std::cout << "device successfully initialized." << std::endl;

  // stop regulation (and leave force enabled)
  drdStop (true);
  is_initialized_ = true;

  // apply zero force
  if (dhdSetForceAndTorqueAndGripperForce (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) < DHD_NO_ERROR) {
    std::cout << "error: cannot set force (" << dhdErrorGetLastStr() << ")\n";
    return false;
  }
  return true;
}