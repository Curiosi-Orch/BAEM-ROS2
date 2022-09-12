#include "iiwaplanner.h"

void IIwaPlanner::start(KukaSunrise *kuka, const Erl::Vector6d &maximumVelocity,
                        const Erl::Vector6d &maximumAcceleration,
                        const Vector7d &maximumJointVelocity,
                        const Vector7d &maximumJointAcceleration) {
  if (!kuka->isConnected()) {
    std::cerr << "Cannot start kuka path planner if kuka is not connected"
              << std::endl;
    return;
  }
  kuka_ = kuka;
  Erl::Transformd initPose = kuka->getDestTransform();
  startJointPlanner(maximumJointVelocity, maximumJointAcceleration);
  TaskSpacePlanner::start(initPose, maximumVelocity, maximumAcceleration);
  engaged_ = true;
}

void IIwaPlanner::start(const double &cycle_Time_ms, const int &cycleSleep_ms,
                        KukaSunrise *kuka, const Erl::Vector6d &maximumVelocity,
                        const Erl::Vector6d &maximumAcceleration,
                        const Vector7d &maximumJointVelocity,
                        const Vector7d &maximumJointAcceleration) {
  if (!kuka->isConnected()) {
    std::cerr << "Cannot start kuka path planner if kuka is not connected"
              << std::endl;
    return;
  }
  kuka_ = kuka;
  Erl::Transformd initPose = kuka->getDestTransform();
  startJointPlanner(maximumJointVelocity, maximumJointAcceleration);
  TaskSpacePlanner::start(cycle_Time_ms, cycleSleep_ms, initPose,
                          maximumVelocity, maximumAcceleration);
  engaged_ = true;
}

void IIwaPlanner::setExplicitJointVelocity(const Vector7d &velocity) {
  if (controlMode_ < CONTROL_JOINT_POSITION) {
    setExplicitVelocity(Erl::Vector6d::Zero());
    waitForDestinationReached();

    std::lock_guard<std::mutex> lock1(loopLock_);
    std::lock_guard<std::mutex> lock2(currentLock_);
    V_currentJointPosition_ = kuka_->getJoints();
    V_currentJointVelocity_ = Vector7d::Zero();
    V_currentJointAcceleration_ = Vector7d::Zero();
  }

  std::lock_guard<std::mutex> lock(inputLock_);
  V_explicitJointVelocity_ = velocity;
  controlMode_ = CONTROL_JOINT_VELOCITY;
  checkReached_External_ = true;
}

void IIwaPlanner::setMaximumJointVelocity(
    const Vector7d &maximumJointVelocity) {
  std::lock_guard<std::mutex> lock(inputLock_);
  V_maximumJointVelocity_ = ERL_PI / 180.0 * maximumJointVelocity;
}

void IIwaPlanner::setMaximumJointAcceleration(
    const Vector7d &maximumJointAcceleration) {
  std::lock_guard<std::mutex> lock(inputLock_);
  V_maximumJointAcceleration_ = ERL_PI / 180.0 * maximumJointAcceleration;
}

IIwaPlanner::Vector7d IIwaPlanner::getCurrentJointVelocity() const {
  std::lock_guard<std::mutex> lock(currentLock_);
  return V_currentJointVelocity_;
}

IIwaPlanner::Vector7d IIwaPlanner::getCurrentJointAcceleration() const {
  std::lock_guard<std::mutex> lock(currentLock_);
  return V_currentJointAcceleration_;
}

IIwaPlanner::Vector7d IIwaPlanner::getMaximumJointVelocity() const {
  std::lock_guard<std::mutex> lock(inputLock_);
  return 180.0 / ERL_PI * V_maximumJointVelocity_;
}

IIwaPlanner::Vector7d IIwaPlanner::getMaximumJointAcceleration() const {
  std::lock_guard<std::mutex> lock(inputLock_);
  return 180.0 / ERL_PI * V_maximumJointAcceleration_;
}

void IIwaPlanner::setEngaged(const bool &engaged) {
  if (engaged) {
    if (controlMode_ <= CONTROL_CARTESIAN_VELOCITY)
      setDesiredTransformFromZero(kuka_->getSetDestTransform());
    else if (controlMode_ <= CONTROL_JOINT_VELOCITY)
      setDesiredJointsFromZero(kuka_->getJoints());
  }
  engaged_ = engaged;
}

void IIwaPlanner::setDesiredTransformFromZero(const Erl::Transformd &pose) {
  if (controlMode_ >= CONTROL_JOINT_POSITION) return;

  std::lock_guard<std::mutex> lock1(loopLock_);
  std::lock_guard<std::mutex> lock2(inputLock_);
  std::lock_guard<std::mutex> lock3(currentLock_);

  Erl::Transformd currPose = kuka_->getDestTransform();
  Qcurr_ = currPose.getQuaternion();
  Qcurr_.normalize();
  V_currentPosition_ =
      Erl::Vector6d(currPose.getTranslation(), Qcurr_.getRotationVector());

  V_desiredPosition_.setVector1(pose.getTranslation());
  Erl::Quaterniond Qorient = pose.getQuaternion();
  if (Qcurr_.dot(Qorient) > 0)
    V_desiredPosition_.setVector2(Qorient.getRotationVector());
  else
    V_desiredPosition_.setVector2((-Qorient).getRotationVector());

  V_currentVelocity_ = Erl::Vector6d::Zero();
  V_currentAcceleration_ = Erl::Vector6d::Zero();

  controlMode_ = CONTROL_CARTESIAN_POSITION;
  checkReached_External_ = true;
}

void IIwaPlanner::setExplicitVelocityFromZero(const Erl::Vector6d &velocity) {
  if (controlMode_ >= CONTROL_JOINT_POSITION) return;

  std::lock_guard<std::mutex> lock1(loopLock_);
  std::lock_guard<std::mutex> lock2(inputLock_);
  std::lock_guard<std::mutex> lock3(currentLock_);

  Erl::Transformd currPose = kuka_->getDestTransform();
  Qcurr_ = currPose.getQuaternion();
  Qcurr_.normalize();
  V_currentPosition_ =
      Erl::Vector6d(currPose.getTranslation(), Qcurr_.getRotationVector());

  V_explicitVelocity_.setVector1(velocity.getVector1());
  V_explicitVelocity_.setVector2(Erl::toRadian(velocity.getVector2()));
  V_currentVelocity_ = Erl::Vector6d::Zero();
  V_currentAcceleration_ = Erl::Vector6d::Zero();

  controlMode_ = CONTROL_CARTESIAN_VELOCITY;
  checkReached_External_ = true;
}

void IIwaPlanner::setDesiredJointsFromZero(const Vector7d &joints) {
  if (controlMode_ < CONTROL_JOINT_POSITION) return;

  std::lock_guard<std::mutex> lock1(loopLock_);
  std::lock_guard<std::mutex> lock2(currentLock_);
  V_currentJointPosition_ = kuka_->getJoints();
  V_desiredJointPosition_ = joints;
  V_currentJointVelocity_ = Vector7d::Zero();
  V_currentJointAcceleration_ = Vector7d::Zero();

  controlMode_ = CONTROL_JOINT_POSITION;
  checkReached_External_ = true;
}

void IIwaPlanner::setExplicitJointVelocityFromZero(const Vector7d &velocity) {
  if (controlMode_ < CONTROL_JOINT_POSITION) return;

  std::lock_guard<std::mutex> lock1(loopLock_);
  std::lock_guard<std::mutex> lock2(currentLock_);
  V_currentJointPosition_ = kuka_->getJoints();
  V_explicitJointVelocity_ = velocity;
  V_currentJointVelocity_ = Vector7d::Zero();
  V_currentJointAcceleration_ = Vector7d::Zero();

  controlMode_ = CONTROL_JOINT_VELOCITY;
  checkReached_External_ = true;
}

void IIwaPlanner::setDesiredJointPosition(const Vector7d &joints) {
  if (controlMode_ < CONTROL_JOINT_POSITION) {
    setExplicitVelocity(Erl::Vector6d::Zero());
    waitForDestinationReached();

    std::lock_guard<std::mutex> lock1(loopLock_);
    std::lock_guard<std::mutex> lock2(currentLock_);
    V_currentJointPosition_ = kuka_->getJoints();
    V_currentJointVelocity_ = Vector7d::Zero();
    V_currentJointAcceleration_ = Vector7d::Zero();
  }

  std::lock_guard<std::mutex> lock(inputLock_);
  V_desiredJointPosition_ = joints;
  controlMode_ = CONTROL_JOINT_POSITION;
  checkReached_External_ = true;
}

void IIwaPlanner::exitJointMode() {
  if (controlMode_ >= CONTROL_JOINT_POSITION) {
    setExplicitJointVelocity(Vector7d::Zero());
    waitForDestinationReached();

    std::lock_guard<std::mutex> lock1(loopLock_);
    std::lock_guard<std::mutex> lock2(inputLock_);
    std::lock_guard<std::mutex> lock3(currentLock_);

    Erl::Transformd currPose = kuka_->getDestTransform();
    Qcurr_ = currPose.getQuaternion();
    Qcurr_.normalize();
    V_currentPosition_ =
        Erl::Vector6d(currPose.getTranslation(), Qcurr_.getRotationVector());
    V_desiredPosition_ = V_currentPosition_;

    V_currentVelocity_ = Erl::Vector6d::Zero();
    V_currentAcceleration_ = Erl::Vector6d::Zero();

    controlMode_ = CONTROL_CARTESIAN_POSITION;
    checkReached_External_ = true;
  }
}

void IIwaPlanner::startJointPlanner(const Vector7d &maximumJointVelocity,
                                    const Vector7d &maximumJointAcceleration) {
  if (isRunning_) return;

  V_currentJointPosition_ = kuka_->getJoints();
  V_desiredJointPosition_ = kuka_->getJoints();

  V_currentJointVelocity_ = Vector7d::Zero();
  V_currentJointAcceleration_ = Vector7d::Zero();
  V_explicitJointVelocity_ = Vector7d::Zero();

  V_maximumJointVelocity_ = ERL_PI / 180.0 * maximumJointVelocity;
  V_maximumJointAcceleration_ = ERL_PI / 180.0 * maximumJointAcceleration;

  RML_joint_ = new ReflexxesAPI(7, cycleTime_ms_ / 1000.0);
  IP_joint_ = new RMLPositionInputParameters(7);
  OP_joint_ = new RMLPositionOutputParameters(7);
  IP_joint_vel_ = new RMLVelocityInputParameters(7);
  OP_joint_vel_ = new RMLVelocityOutputParameters(7);

  IP_joint_->CurrentPositionVector->VecData[0] = V_currentJointPosition_[0];
  IP_joint_->CurrentPositionVector->VecData[1] = V_currentJointPosition_[1];
  IP_joint_->CurrentPositionVector->VecData[2] = V_currentJointPosition_[2];
  IP_joint_->CurrentPositionVector->VecData[3] = V_currentJointPosition_[3];
  IP_joint_->CurrentPositionVector->VecData[4] = V_currentJointPosition_[4];
  IP_joint_->CurrentPositionVector->VecData[5] = V_currentJointPosition_[5];
  IP_joint_->CurrentPositionVector->VecData[6] = V_currentJointPosition_[6];

  IP_joint_->CurrentVelocityVector->VecData[0] = 0.0;
  IP_joint_->CurrentVelocityVector->VecData[1] = 0.0;
  IP_joint_->CurrentVelocityVector->VecData[2] = 0.0;
  IP_joint_->CurrentVelocityVector->VecData[3] = 0.0;
  IP_joint_->CurrentVelocityVector->VecData[4] = 0.0;
  IP_joint_->CurrentVelocityVector->VecData[5] = 0.0;
  IP_joint_->CurrentVelocityVector->VecData[6] = 0.0;

  IP_joint_->CurrentAccelerationVector->VecData[0] = 0.0;
  IP_joint_->CurrentAccelerationVector->VecData[1] = 0.0;
  IP_joint_->CurrentAccelerationVector->VecData[2] = 0.0;
  IP_joint_->CurrentAccelerationVector->VecData[3] = 0.0;
  IP_joint_->CurrentAccelerationVector->VecData[4] = 0.0;
  IP_joint_->CurrentAccelerationVector->VecData[5] = 0.0;
  IP_joint_->CurrentAccelerationVector->VecData[6] = 0.0;

  IP_joint_->MaxVelocityVector->VecData[0] = V_maximumJointVelocity_[0];
  IP_joint_->MaxVelocityVector->VecData[1] = V_maximumJointVelocity_[1];
  IP_joint_->MaxVelocityVector->VecData[2] = V_maximumJointVelocity_[2];
  IP_joint_->MaxVelocityVector->VecData[3] = V_maximumJointVelocity_[3];
  IP_joint_->MaxVelocityVector->VecData[4] = V_maximumJointVelocity_[4];
  IP_joint_->MaxVelocityVector->VecData[5] = V_maximumJointVelocity_[5];
  IP_joint_->MaxVelocityVector->VecData[6] = V_maximumJointVelocity_[6];

  IP_joint_->MaxAccelerationVector->VecData[0] = V_maximumJointAcceleration_[0];
  IP_joint_->MaxAccelerationVector->VecData[1] = V_maximumJointAcceleration_[1];
  IP_joint_->MaxAccelerationVector->VecData[2] = V_maximumJointAcceleration_[2];
  IP_joint_->MaxAccelerationVector->VecData[3] = V_maximumJointAcceleration_[3];
  IP_joint_->MaxAccelerationVector->VecData[4] = V_maximumJointAcceleration_[4];
  IP_joint_->MaxAccelerationVector->VecData[5] = V_maximumJointAcceleration_[5];
  IP_joint_->MaxAccelerationVector->VecData[6] = V_maximumJointAcceleration_[6];

  IP_joint_->TargetPositionVector->VecData[0] = V_desiredJointPosition_[0];
  IP_joint_->TargetPositionVector->VecData[1] = V_desiredJointPosition_[1];
  IP_joint_->TargetPositionVector->VecData[2] = V_desiredJointPosition_[2];
  IP_joint_->TargetPositionVector->VecData[3] = V_desiredJointPosition_[3];
  IP_joint_->TargetPositionVector->VecData[4] = V_desiredJointPosition_[4];
  IP_joint_->TargetPositionVector->VecData[5] = V_desiredJointPosition_[5];
  IP_joint_->TargetPositionVector->VecData[6] = V_desiredJointPosition_[6];

  IP_joint_->TargetVelocityVector->VecData[0] = 0.0;
  IP_joint_->TargetVelocityVector->VecData[1] = 0.0;
  IP_joint_->TargetVelocityVector->VecData[2] = 0.0;
  IP_joint_->TargetVelocityVector->VecData[3] = 0.0;
  IP_joint_->TargetVelocityVector->VecData[4] = 0.0;
  IP_joint_->TargetVelocityVector->VecData[5] = 0.0;
  IP_joint_->TargetVelocityVector->VecData[6] = 0.0;

  IP_joint_->SelectionVector->VecData[0] = true;
  IP_joint_->SelectionVector->VecData[1] = true;
  IP_joint_->SelectionVector->VecData[2] = true;
  IP_joint_->SelectionVector->VecData[3] = true;
  IP_joint_->SelectionVector->VecData[4] = true;
  IP_joint_->SelectionVector->VecData[5] = true;
  IP_joint_->SelectionVector->VecData[6] = true;

  *IP_joint_vel_->CurrentPositionVector = *IP_joint_->CurrentPositionVector;
  *IP_joint_vel_->CurrentVelocityVector = *IP_joint_->CurrentVelocityVector;
  *IP_joint_vel_->CurrentAccelerationVector =
      *IP_joint_->CurrentAccelerationVector;
  *IP_joint_vel_->MaxAccelerationVector = *IP_joint_->MaxAccelerationVector;
  *IP_joint_vel_->SelectionVector = *IP_joint_->SelectionVector;

  IP_joint_vel_->TargetVelocityVector->VecData[0] = V_explicitJointVelocity_[0];
  IP_joint_vel_->TargetVelocityVector->VecData[1] = V_explicitJointVelocity_[1];
  IP_joint_vel_->TargetVelocityVector->VecData[2] = V_explicitJointVelocity_[2];
  IP_joint_vel_->TargetVelocityVector->VecData[3] = V_explicitJointVelocity_[3];
  IP_joint_vel_->TargetVelocityVector->VecData[4] = V_explicitJointVelocity_[4];
  IP_joint_vel_->TargetVelocityVector->VecData[5] = V_explicitJointVelocity_[5];
  IP_joint_vel_->TargetVelocityVector->VecData[6] = V_explicitJointVelocity_[6];
}

void IIwaPlanner::updateExternalCommands_locked() {
  if (kuka_->isPoseLoopBackEnabled()) {
    std::lock_guard<std::mutex> lock(currentLock_);
    V_desiredPosition_ = V_currentPosition_;
    V_desiredJointPosition_ = V_currentJointPosition_;
  }

  IP_joint_->MaxVelocityVector->VecData[0] = V_maximumJointVelocity_[0];
  IP_joint_->MaxVelocityVector->VecData[1] = V_maximumJointVelocity_[1];
  IP_joint_->MaxVelocityVector->VecData[2] = V_maximumJointVelocity_[2];
  IP_joint_->MaxVelocityVector->VecData[3] = V_maximumJointVelocity_[3];
  IP_joint_->MaxVelocityVector->VecData[4] = V_maximumJointVelocity_[4];
  IP_joint_->MaxVelocityVector->VecData[5] = V_maximumJointVelocity_[5];
  IP_joint_->MaxVelocityVector->VecData[6] = V_maximumJointVelocity_[6];

  IP_joint_->MaxAccelerationVector->VecData[0] = V_maximumJointAcceleration_[0];
  IP_joint_->MaxAccelerationVector->VecData[1] = V_maximumJointAcceleration_[1];
  IP_joint_->MaxAccelerationVector->VecData[2] = V_maximumJointAcceleration_[2];
  IP_joint_->MaxAccelerationVector->VecData[3] = V_maximumJointAcceleration_[3];
  IP_joint_->MaxAccelerationVector->VecData[4] = V_maximumJointAcceleration_[4];
  IP_joint_->MaxAccelerationVector->VecData[5] = V_maximumJointAcceleration_[5];
  IP_joint_->MaxAccelerationVector->VecData[6] = V_maximumJointAcceleration_[6];

  IP_joint_->TargetPositionVector->VecData[0] = V_desiredJointPosition_[0];
  IP_joint_->TargetPositionVector->VecData[1] = V_desiredJointPosition_[1];
  IP_joint_->TargetPositionVector->VecData[2] = V_desiredJointPosition_[2];
  IP_joint_->TargetPositionVector->VecData[3] = V_desiredJointPosition_[3];
  IP_joint_->TargetPositionVector->VecData[4] = V_desiredJointPosition_[4];
  IP_joint_->TargetPositionVector->VecData[5] = V_desiredJointPosition_[5];
  IP_joint_->TargetPositionVector->VecData[6] = V_desiredJointPosition_[6];

  IP_joint_vel_->TargetVelocityVector->VecData[0] = V_explicitJointVelocity_[0];
  IP_joint_vel_->TargetVelocityVector->VecData[1] = V_explicitJointVelocity_[1];
  IP_joint_vel_->TargetVelocityVector->VecData[2] = V_explicitJointVelocity_[2];
  IP_joint_vel_->TargetVelocityVector->VecData[3] = V_explicitJointVelocity_[3];
  IP_joint_vel_->TargetVelocityVector->VecData[4] = V_explicitJointVelocity_[4];
  IP_joint_vel_->TargetVelocityVector->VecData[5] = V_explicitJointVelocity_[5];
  IP_joint_vel_->TargetVelocityVector->VecData[6] = V_explicitJointVelocity_[6];
}

void IIwaPlanner::updateSelfInputs() {
  {
    std::lock_guard<std::mutex> lock(currentLock_);

    updateSelfInputs_locked();

    IP_pos_->CurrentPositionVector->VecData[0] = V_currentPosition_[0];
    IP_pos_->CurrentPositionVector->VecData[1] = V_currentPosition_[1];
    IP_pos_->CurrentPositionVector->VecData[2] = V_currentPosition_[2];
    IP_pos_->CurrentPositionVector->VecData[3] = V_currentPosition_[3];
    IP_pos_->CurrentPositionVector->VecData[4] = V_currentPosition_[4];
    IP_pos_->CurrentPositionVector->VecData[5] = V_currentPosition_[5];

    IP_joint_->CurrentPositionVector->VecData[0] = V_currentJointPosition_[0];
    IP_joint_->CurrentPositionVector->VecData[1] = V_currentJointPosition_[1];
    IP_joint_->CurrentPositionVector->VecData[2] = V_currentJointPosition_[2];
    IP_joint_->CurrentPositionVector->VecData[3] = V_currentJointPosition_[3];
    IP_joint_->CurrentPositionVector->VecData[4] = V_currentJointPosition_[4];
    IP_joint_->CurrentPositionVector->VecData[5] = V_currentJointPosition_[5];
    IP_joint_->CurrentPositionVector->VecData[6] = V_currentJointPosition_[6];
  }

  *IP_pos_->CurrentVelocityVector = *OP_pos_->NewVelocityVector;
  *IP_pos_->CurrentAccelerationVector = *OP_pos_->NewAccelerationVector;
  *IP_joint_->CurrentVelocityVector = *OP_joint_->NewVelocityVector;
  *IP_joint_->CurrentAccelerationVector = *OP_joint_->NewAccelerationVector;

  *IP_joint_vel_->CurrentPositionVector = *IP_joint_->CurrentPositionVector;
  *IP_joint_vel_->CurrentVelocityVector = *IP_joint_->CurrentVelocityVector;
  *IP_joint_vel_->CurrentAccelerationVector =
      *IP_joint_->CurrentAccelerationVector;
  *IP_joint_vel_->MaxAccelerationVector = *IP_joint_->MaxAccelerationVector;
}

void IIwaPlanner::updateSelfInputs_locked() {
  /// Kuka destination represents the "ideal" position, disregarding errors,
  /// travel time, and impedance mode. Get the set destination as opposed to the
  /// received one to avoid the substential round trip delay.

  //    Erl::Transformd Tk = kuka_->getSetDestTransform();
  //    V_currentPosition_.setVector1(Tk.getTranslation());
  //    Erl::Quaterniond Qk = Tk.getQuaternion();
  //    if (Qcurr_.dot(Qk) > 0)
  //        V_currentPosition_.setVector2(Qk.getRotationVector());
  //    else
  //        V_currentPosition_.setVector2((-Qk).getRotationVector());

  //    V_currentJointPosition_ = kuka_->getSetJoints();
}

void IIwaPlanner::updateOutputs(const int &controlMode) {
  std::lock_guard<std::mutex> lock(currentLock_);

  if (controlMode <= CONTROL_CARTESIAN_VELOCITY) {
    if (controlMode == CONTROL_CARTESIAN_VELOCITY) {
      *OP_pos_->NewPositionVector = *OP_vel_->NewPositionVector;
      *OP_pos_->NewVelocityVector = *OP_vel_->NewVelocityVector;
      *OP_pos_->NewAccelerationVector = *OP_vel_->NewAccelerationVector;
    }

    V_currentPosition_[0] = (*OP_pos_->NewPositionVector)[0];
    V_currentPosition_[1] = (*OP_pos_->NewPositionVector)[1];
    V_currentPosition_[2] = (*OP_pos_->NewPositionVector)[2];
    V_currentPosition_[3] = (*OP_pos_->NewPositionVector)[3];
    V_currentPosition_[4] = (*OP_pos_->NewPositionVector)[4];
    V_currentPosition_[5] = (*OP_pos_->NewPositionVector)[5];

    V_currentVelocity_[0] = (*OP_pos_->NewVelocityVector)[0];
    V_currentVelocity_[1] = (*OP_pos_->NewVelocityVector)[1];
    V_currentVelocity_[2] = (*OP_pos_->NewVelocityVector)[2];
    V_currentVelocity_[3] = (*OP_pos_->NewVelocityVector)[3];
    V_currentVelocity_[4] = (*OP_pos_->NewVelocityVector)[4];
    V_currentVelocity_[5] = (*OP_pos_->NewVelocityVector)[5];

    V_currentAcceleration_[0] = (*OP_pos_->NewAccelerationVector)[0];
    V_currentAcceleration_[1] = (*OP_pos_->NewAccelerationVector)[1];
    V_currentAcceleration_[2] = (*OP_pos_->NewAccelerationVector)[2];
    V_currentAcceleration_[3] = (*OP_pos_->NewAccelerationVector)[3];
    V_currentAcceleration_[4] = (*OP_pos_->NewAccelerationVector)[4];
    V_currentAcceleration_[5] = (*OP_pos_->NewAccelerationVector)[5];

    Qcurr_ =
        Erl::Quaterniond::fromRotationVector(V_currentPosition_.getVector2());

    if (engaged_)
      kuka_->setTransform(
          Erl::Transformd(Qcurr_, V_currentPosition_.getVector1()));
  } else if (controlMode >= CONTROL_JOINT_POSITION) {
    if (controlMode == CONTROL_JOINT_VELOCITY) {
      *OP_joint_->NewPositionVector = *OP_joint_vel_->NewPositionVector;
      *OP_joint_->NewVelocityVector = *OP_joint_vel_->NewVelocityVector;
      *OP_joint_->NewAccelerationVector = *OP_joint_vel_->NewAccelerationVector;
    }

    V_currentJointPosition_[0] = (*OP_joint_->NewPositionVector)[0];
    V_currentJointPosition_[1] = (*OP_joint_->NewPositionVector)[1];
    V_currentJointPosition_[2] = (*OP_joint_->NewPositionVector)[2];
    V_currentJointPosition_[3] = (*OP_joint_->NewPositionVector)[3];
    V_currentJointPosition_[4] = (*OP_joint_->NewPositionVector)[4];
    V_currentJointPosition_[5] = (*OP_joint_->NewPositionVector)[5];
    V_currentJointPosition_[6] = (*OP_joint_->NewPositionVector)[6];

    V_currentJointVelocity_[0] = (*OP_joint_->NewVelocityVector)[0];
    V_currentJointVelocity_[1] = (*OP_joint_->NewVelocityVector)[1];
    V_currentJointVelocity_[2] = (*OP_joint_->NewVelocityVector)[2];
    V_currentJointVelocity_[3] = (*OP_joint_->NewVelocityVector)[3];
    V_currentJointVelocity_[4] = (*OP_joint_->NewVelocityVector)[4];
    V_currentJointVelocity_[5] = (*OP_joint_->NewVelocityVector)[5];
    V_currentJointVelocity_[6] = (*OP_joint_->NewVelocityVector)[6];

    V_currentJointAcceleration_[0] = (*OP_joint_->NewAccelerationVector)[0];
    V_currentJointAcceleration_[1] = (*OP_joint_->NewAccelerationVector)[1];
    V_currentJointAcceleration_[2] = (*OP_joint_->NewAccelerationVector)[2];
    V_currentJointAcceleration_[3] = (*OP_joint_->NewAccelerationVector)[3];
    V_currentJointAcceleration_[4] = (*OP_joint_->NewAccelerationVector)[4];
    V_currentJointAcceleration_[5] = (*OP_joint_->NewAccelerationVector)[5];
    V_currentJointAcceleration_[6] = (*OP_joint_->NewAccelerationVector)[6];

    if (engaged_) kuka_->setJoints(V_currentJointPosition_);
  }

  checkReached_External_ = checkReached_Internal_;
}

void IIwaPlanner::controlLoop() {
  isRunning_ = true;
  spinner_.reset();

  while (isRunning_) {
    int controlMode = controlMode_;
    updateInputs();

    if (controlMode == CONTROL_CARTESIAN_VELOCITY)
      RML_result_ = RML_->RMLVelocity(*IP_vel_, OP_vel_, Flags_vel_);
    else if (controlMode == CONTROL_CARTESIAN_POSITION)
      RML_result_ = RML_->RMLPosition(*IP_pos_, OP_pos_, Flags_pos_);
    else if (controlMode == CONTROL_JOINT_POSITION)
      RML_result_ =
          RML_joint_->RMLPosition(*IP_joint_, OP_joint_, Flags_joint_);
    else if (controlMode == CONTROL_JOINT_VELOCITY)
      RML_result_ = RML_joint_->RMLVelocity(*IP_joint_vel_, OP_joint_vel_,
                                            Flags_joint_vel_);

    if (RML_result_ < 0) {
      std::cerr << "Reflexxes cartesian planning error: " << RML_result_
                << std::endl;
      isRunning_ = false;
      break;
    }
    if (RML_result_ == ReflexxesAPI::RML_FINAL_STATE_REACHED &&
        checkReached_Internal_)
      checkReached_Internal_ = false;

    updateOutputs(controlMode);

    if (cycleSleep_ms_ > 0) Erl::sleep_ms(cycleSleep_ms_);
    spinner_.spinUpWithInfo();
  }

  if (RML_result_ < 0) cleanup();
}
