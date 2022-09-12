#include "iiwa/planner/kukasunriseplanned.h"

KukaSunrisePlanned::KukaSunrisePlanned()
    : kuka_(new KukaSunrise()), planner_(new IIwaPlanner()) {}

KukaSunrisePlanned::~KukaSunrisePlanned() {
  stop();
  delete kuka_;
  kuka_ = nullptr;
  delete planner_;
  planner_ = nullptr;
}

bool KukaSunrisePlanned::start(const InitialParameters &initParams,
                               int8_t _Priority, int16_t _Affinity) {
  bool startOk = start(initParams);
  std::thread *thk = kuka_->getThread();
  Erl::RT::setAffinity(*thk, _Affinity);
  Erl::RT::setPolicy(*thk, Erl::RT::POLICY::FIFO, _Priority);

  return startOk;
}

bool KukaSunrisePlanned::start(const InitialParameters &initParams) {
  if (!kuka_->isConnected() && !initParams.isValid()) return false;

  std::lock_guard<std::mutex> lock(initParamsLock_);
  initParams_ = initParams;

  bool startOk = kuka_->connect(
      initParams_.hostname_, initParams_.localPort_, initParams_.iiwaPort_,
      initParams_.inital_timeout_, initParams_.comm_timeout_);
  planner_->start(initParams_.plannerCycleTime_, initParams_.plannerSleepTime_,
                  kuka_, initParams_.maxVelocity_, initParams_.maxAcceleration_,
                  initParams_.maxJointVelocity_,
                  initParams_.maxJointAcceleration_);
  return startOk;
}

bool KukaSunrisePlanned::stop() {
  if (!kuka_->isConnected()) return false;

  planner_->stop();
  return kuka_->close();
}
