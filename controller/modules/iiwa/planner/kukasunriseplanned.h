#ifndef IIWACOM_H
#define IIWACOM_H

#include "iiwa/planner/iiwaplanner.h"
#include "iiwa/sunrise/kukasunrise.h"

class KukaSunrisePlanned {
 public:
  KukaSunrisePlanned();
  ~KukaSunrisePlanned();

  KukaSunrisePlanned(const KukaSunrisePlanned& K) = delete;
  KukaSunrisePlanned& operator=(const KukaSunrisePlanned& K) = delete;

  struct InitialParameters {
    std::string Id_ = "";
    std::string hostname_ = "";
    int localPort_ = -1;
    int iiwaPort_ = -1;
    double inital_timeout_ = -1;
    double comm_timeout_ = -1;
    double plannerCycleTime_ = -1;
    double plannerSleepTime_ = -1;
    Erl::Vector6d maxVelocity_ = Erl::Vector6d::Zero();
    Erl::Vector6d maxAcceleration_ = Erl::Vector6d::Zero();
    Eigen::Matrix<double, 7, 1> maxJointVelocity_ =
        Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> maxJointAcceleration_ =
        Eigen::Matrix<double, 7, 1>::Zero();

    bool isValid() const {
      return Id_ != "" && hostname_ != "" && localPort_ != -1 &&
             iiwaPort_ != -1 && inital_timeout_ != -1 && comm_timeout_ != -1 &&
             plannerCycleTime_ != -1 && plannerSleepTime_ != -1 &&
             maxVelocity_ != Erl::Vector6d::Zero() &&
             maxAcceleration_ != Erl::Vector6d::Zero() &&
             maxJointVelocity_ != Eigen::Matrix<double, 7, 1>::Zero() &&
             maxJointAcceleration_ != Eigen::Matrix<double, 7, 1>::Zero();
    }
  };
  
  bool start(const InitialParameters& initParams, int8_t _Priority,
             int16_t _Affinity);
  bool start(const InitialParameters& initParams);
  bool stop();

  inline bool isRunning() const { return kuka_->isConnected(); }
  inline bool isPoseLoopBackEnabled() const {
    return kuka_->isPoseLoopBackEnabled();
  }

  inline Erl::Transformd getMsrTransform() const {
    return kuka_->getMsrTransform();
  }

  inline Erl::Transformd getDestTransform() const {
    return kuka_->getDestTransform();
  }

  inline Erl::Transformd getSetDestTransform() const {
    return kuka_->getSetDestTransform();
  }

  inline Eigen::Matrix<double, 7, 1> getJoints() const {
    return kuka_->getJoints();
  }

  inline Eigen::Matrix<double, 7, 1> getStiffness() const {
    return kuka_->getStiffness();
  }

  inline Eigen::Matrix<double, 7, 1> getDamping() const {
    return kuka_->getDamping();
  }

  inline bool isUsingImpedanceMode() const {
    return kuka_->isUsingImpedanceMode();
  }

  inline void setTransform(const Erl::Transformd& destination) {
    planner_->setDesiredTransform(destination);
  }

  inline void setJoints(const Eigen::Matrix<double, 7, 1>& joints) {
    planner_->setDesiredJointPosition(joints);
  }

  inline void setStiffness(const Eigen::Matrix<double, 7, 1>& stiffness) {
    return kuka_->setStiffness(stiffness);
  }

  inline void setStiffness(const Eigen::Matrix<double, 6, 1>& stiffness) {
    return kuka_->setStiffness(stiffness);
  }

  inline void setDamping(const Eigen::Matrix<double, 7, 1>& damping) {
    return kuka_->setDamping(damping);
  }

  inline void setDamping(const Eigen::Matrix<double, 6, 1>& damping) {
    return kuka_->setDamping(damping);
  }

  inline void setEnableOutput(const bool& enable) {
    return kuka_->setEnableOutput(enable);
  }

  inline void applyGravityCompensationSettings() {
    planner_->disengage();
    return kuka_->applyGravityCompensationSettings();
  }

  inline void disableGravityCompensationSettings(
      Erl::Vector6d stiffness = Erl::Vector6d(2000, 2000, 2000, 100, 100, 100),
      Erl::Vector6d damping = Erl::Vector6d(0.7, 0.7, 0.7, 0.7, 0.7, 0.7)) {
    if (!planner_->isEngaged()) planner_->engage();
    return kuka_->disableGravityCompensationSettings(stiffness, damping);
  }

  inline bool isOutputEnabled() const { return kuka_->isOutputEnabled(); }

  inline void setEnablePlanner(const bool& enable) {
    planner_->setEngaged(enable);
  }

  inline void setDesiredVelocity(const Erl::Vector6d& velocity) {
    planner_->setDesiredVelocity(velocity);
  }

  inline void setExplicitVelocity(const Erl::Vector6d& velocity) {
    planner_->setExplicitVelocity(velocity);
  }

  inline void setMaximumVelocity(const Erl::Vector6d& maximumVelocity) {
    planner_->setMaximumVelocity(maximumVelocity);
  }

  inline void setMaximumAcceleration(const Erl::Vector6d& maximumAcceleration) {
    planner_->setMaximumAcceleration(maximumAcceleration);
  }

  inline void setMaximumJointVelocity(
      const Eigen::Matrix<double, 7, 1>& maximumJointVelocity) {
    planner_->setMaximumJointVelocity(maximumJointVelocity);
  }

  inline void setMaximumJointAcceleration(
      const Eigen::Matrix<double, 7, 1>& maximumJointAcceleration) {
    planner_->setMaximumJointAcceleration(maximumJointAcceleration);
  }

  inline void setDefaultVelocity() {
    std::lock_guard<std::mutex> lock(initParamsLock_);
    planner_->setMaximumVelocity(initParams_.maxVelocity_);
  }

  inline void setDefaultAcceleration() {
    std::lock_guard<std::mutex> lock(initParamsLock_);
    planner_->setMaximumAcceleration(initParams_.maxAcceleration_);
  }

  inline void setDefaultJointVelocity() {
    std::lock_guard<std::mutex> lock(initParamsLock_);
    planner_->setMaximumJointVelocity(initParams_.maxJointVelocity_);
  }

  inline void setDefaultJointAcceleration() {
    std::lock_guard<std::mutex> lock(initParamsLock_);
    planner_->setMaximumJointAcceleration(initParams_.maxJointAcceleration_);
  }

  inline void waitForDestinationReached() const {
    planner_->waitForDestinationReached();
  }

  inline bool isDestinationReached() const {
    return planner_->isDestinationReached();
  }

  inline Erl::Vector6d getCurrentVelocity() const {
    return planner_->getCurrentVelocity();
  }

  inline Erl::Vector6d getCurrentAcceleration() const {
    return planner_->getCurrentAcceleration();
  }

  inline Erl::Vector6d getMaximumVelocity() const {
    return planner_->getMaximumVelocity();
  }

  inline Erl::Vector6d getMaximumAcceleration() const {
    return planner_->getMaximumAcceleration();
  }

  inline Eigen::Matrix<double, 7, 1> getCurrentJointVelocity() const {
    return planner_->getCurrentJointVelocity();
  }

  inline Eigen::Matrix<double, 7, 1> getCurrentJointAcceleration() const {
    return planner_->getCurrentJointAcceleration();
  }

  inline Eigen::Matrix<double, 7, 1> getMaximumJointVelocity() const {
    return planner_->getMaximumJointVelocity();
  }

  inline Eigen::Matrix<double, 7, 1> getMaximumJointAcceleration() const {
    return planner_->getMaximumJointAcceleration();
  }

  inline InitialParameters getInitialParameters() const {
    std::lock_guard<std::mutex> lock(initParamsLock_);
    return initParams_;
  }

  inline std::string getId() const {
    std::lock_guard<std::mutex> lock(initParamsLock_);
    return initParams_.Id_;
  }

  inline IIwaPlanner* _getPlanner() { return planner_; }

  inline KukaSunrise* _getKuka() { return kuka_; }

 protected:
  KukaSunrise* kuka_;
  IIwaPlanner* planner_;
  InitialParameters initParams_;
  mutable std::mutex initParamsLock_;
};

#endif
