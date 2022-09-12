#ifndef CARTESIANVELOCITYPLANNER_H
#define CARTESIANVELOCITYPLANNER_H

#include <Erl.h>

#include <chrono>

class CartesianVelocityPlanner {
 public:
  CartesianVelocityPlanner()
      : p_(0, 0, 0),
        v_(0, 0, 0),
        a_(0, 0, 0),
        v_desired_(0, 0, 0),
        a_max_(10, 10, 10),
        a_max_brake_(a_max_),
        braking_(false),
        previous_(std::chrono::high_resolution_clock::now()),
        last_timestep_s_(0) {}

  void step() {
    std::lock_guard<std::mutex> lock(dataLock_);

    t_ = std::chrono::high_resolution_clock::now() - previous_;
    previous_ = std::chrono::high_resolution_clock::now();
    if (t_.count() == 0) return;
    double dt_s = t_.count() / 1000000000.0;
    last_timestep_s_ = dt_s;

    for (int i = 0; i < 3; i++) {
      if (v_[i] != v_desired_[i]) {
        if (braking_)
          a_[i] = a_max_brake_[i] * Erl::signum(v_desired_[i] - v_[i]);
        else
          a_[i] = a_max_[i] * Erl::signum(v_desired_[i] - v_[i]);
        ;
      } else
        a_[i] = 0.0;
    }

    v_ += a_ * dt_s;

    for (int i = 0; i < 3; i++) {
      if (a_[i] > 0)
        v_[i] = std::min(v_[i], v_desired_[i]);
      else if (a_[i] < 0)
        v_[i] = std::max(v_[i], v_desired_[i]);
    }
    p_ += v_ * dt_s;
  }

  inline void setPosition(const Erl::Vector3d& pos) {
    std::lock_guard<std::mutex> lock(dataLock_);
    p_ = pos;
  }
  inline void setVelocity(const Erl::Vector3d& vel) {
    std::lock_guard<std::mutex> lock(dataLock_);
    v_ = vel;
  }
  inline void setMaxAcceleration(const Erl::Vector3d& acc) {
    std::lock_guard<std::mutex> lock(dataLock_);
    a_max_ = acc;
  }
  inline void setMaxBrakeAcceleration(const Erl::Vector3d& acc) {
    std::lock_guard<std::mutex> lock(dataLock_);
    a_max_brake_ = acc;
  }

  inline void setDesiredVelocity(const Erl::Vector3d& vel) {
    std::lock_guard<std::mutex> lock(dataLock_);
    if (!braking_) v_desired_ = vel;
  }
  inline void setBraking(const bool& braking) {
    std::lock_guard<std::mutex> lock(dataLock_);
    braking_ = braking;
    v_desired_ = Erl::Vector3d::Zero();
  }

  inline Erl::Vector3d getPosition() {
    std::lock_guard<std::mutex> lock(dataLock_);
    return p_;
  }
  inline Erl::Vector3d getVelocity() {
    std::lock_guard<std::mutex> lock(dataLock_);
    return v_;
  }
  inline Erl::Vector3d getAcceleration() {
    std::lock_guard<std::mutex> lock(dataLock_);
    return a_;
  }
  inline Erl::Vector3d getDesiredVelocity() {
    std::lock_guard<std::mutex> lock(dataLock_);
    return v_desired_;
  }
  inline Erl::Vector3d getMaxAcceleration() {
    std::lock_guard<std::mutex> lock(dataLock_);
    return a_max_;
  }
  inline Erl::Vector3d getMaxBrakeAcceleration() {
    std::lock_guard<std::mutex> lock(dataLock_);
    return a_max_brake_;
  }
  inline bool isBraking() {
    std::lock_guard<std::mutex> lock(dataLock_);
    return braking_;
  }
  inline double getLastTimeStep_s() {
    std::lock_guard<std::mutex> lock(dataLock_);
    return last_timestep_s_;
  }

 protected:
  Erl::Vector3d p_;
  Erl::Vector3d v_;
  Erl::Vector3d a_;

  Erl::Vector3d v_desired_;
  Erl::Vector3d a_max_;
  Erl::Vector3d a_max_brake_;
  bool braking_;
  double last_timestep_s_;

  std::chrono::time_point<std::chrono::high_resolution_clock,
                          std::chrono::nanoseconds>
      previous_;
  mutable std::chrono::nanoseconds t_;
  mutable std::mutex dataLock_;
};

#endif
