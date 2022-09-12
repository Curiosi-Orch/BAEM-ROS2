#include "iiwa/planner/taskspaceplanner.h"

TaskSpacePlanner::TaskSpacePlanner(const double &cycle_Time_ms,
                                   const int &cycleSleep_ms)
    : numDofs_(6),
      cycleTime_ms_(cycle_Time_ms),
      cycleSleep_ms_(cycleSleep_ms),
      spinner_(cycle_Time_ms),
      V_currentPosition_(Erl::Vector6d::Zero()),
      V_currentVelocity_(Erl::Vector6d::Zero()),
      V_currentAcceleration_(Erl::Vector6d::Zero()),
      V_desiredPosition_(Erl::Vector6d::Zero()),
      V_desiredVelocity_(Erl::Vector6d::Zero()),
      V_explicitVelocity_(Erl::Vector6d::Zero()),
      V_maximumVelocity_(Erl::Vector6d::Zero()),
      V_maximumAcceleration_(Erl::Vector6d::Zero()),
      V_manualPosition_(Erl::Vector6d::Zero()),
      manualUpdate_Position_(false),
      manualUpdate_Orientation_(false),
      isRunning_(false),
      RML_(nullptr),
      IP_pos_(nullptr),
      OP_pos_(nullptr),
      IP_vel_(nullptr),
      OP_vel_(nullptr),
      controlMode_(CONTROL_CARTESIAN_POSITION),
      checkReached_Internal_(false),
      checkReached_External_(false) {}

TaskSpacePlanner::TaskSpacePlanner() : TaskSpacePlanner(5.0, 3.0) {}

TaskSpacePlanner::~TaskSpacePlanner() { stop(); }

void TaskSpacePlanner::start(const Erl::Transformd &initialTransform,
                             const Erl::Vector6d &maximumVelocity,
                             const Erl::Vector6d &maximumAcceleration) {
  std::lock_guard<std::mutex> lock(initLock_);

  if (isRunning_) return;

  Erl::Vector3d initialPosition = initialTransform.getTranslation();
  Erl::Quaterniond initialOrientation = initialTransform.getQuaternion();

  Qcurr_ = initialOrientation;
  Qcurr_.normalize();
  V_currentPosition_ =
      Erl::Vector6d(initialPosition, Qcurr_.getRotationVector());
  V_desiredPosition_ =
      Erl::Vector6d(initialPosition, Qcurr_.getRotationVector());
  V_desiredVelocity_ = Erl::Vector6d::Zero();
  V_explicitVelocity_ = Erl::Vector6d::Zero();
  V_manualPosition_ =
      Erl::Vector6d(initialPosition, Qcurr_.getRotationVector());
  manualUpdate_Position_ = false;
  manualUpdate_Orientation_ = false;
  checkReached_Internal_ = false;
  checkReached_External_ = false;
  controlMode_ = CONTROL_CARTESIAN_POSITION;

  V_currentVelocity_ = Erl::Vector6d::Zero();
  V_currentAcceleration_ = Erl::Vector6d::Zero();

  V_maximumVelocity_[0] = maximumVelocity[0];
  V_maximumVelocity_[1] = maximumVelocity[1];
  V_maximumVelocity_[2] = maximumVelocity[2];
  V_maximumVelocity_[3] = Erl::toRadian(maximumVelocity[3]);
  V_maximumVelocity_[4] = Erl::toRadian(maximumVelocity[4]);
  V_maximumVelocity_[5] = Erl::toRadian(maximumVelocity[5]);

  V_maximumAcceleration_[0] = maximumAcceleration[0];
  V_maximumAcceleration_[1] = maximumAcceleration[1];
  V_maximumAcceleration_[2] = maximumAcceleration[2];
  V_maximumAcceleration_[3] = Erl::toRadian(maximumAcceleration[3]);
  V_maximumAcceleration_[4] = Erl::toRadian(maximumAcceleration[4]);
  V_maximumAcceleration_[5] = Erl::toRadian(maximumAcceleration[5]);

  RML_ = new ReflexxesAPI(numDofs_, cycleTime_ms_ / 1000.0);
  IP_pos_ = new RMLPositionInputParameters(numDofs_);
  OP_pos_ = new RMLPositionOutputParameters(numDofs_);
  IP_vel_ = new RMLVelocityInputParameters(numDofs_);
  OP_vel_ = new RMLVelocityOutputParameters(numDofs_);

  IP_pos_->CurrentPositionVector->VecData[0] = V_currentPosition_[0];
  IP_pos_->CurrentPositionVector->VecData[1] = V_currentPosition_[1];
  IP_pos_->CurrentPositionVector->VecData[2] = V_currentPosition_[2];
  IP_pos_->CurrentPositionVector->VecData[3] = V_currentPosition_[3];
  IP_pos_->CurrentPositionVector->VecData[4] = V_currentPosition_[4];
  IP_pos_->CurrentPositionVector->VecData[5] = V_currentPosition_[5];

  IP_pos_->CurrentVelocityVector->VecData[0] = 0.0;
  IP_pos_->CurrentVelocityVector->VecData[1] = 0.0;
  IP_pos_->CurrentVelocityVector->VecData[2] = 0.0;
  IP_pos_->CurrentVelocityVector->VecData[3] = 0.0;
  IP_pos_->CurrentVelocityVector->VecData[4] = 0.0;
  IP_pos_->CurrentVelocityVector->VecData[5] = 0.0;

  IP_pos_->CurrentAccelerationVector->VecData[0] = 0.0;
  IP_pos_->CurrentAccelerationVector->VecData[1] = 0.0;
  IP_pos_->CurrentAccelerationVector->VecData[2] = 0.0;
  IP_pos_->CurrentAccelerationVector->VecData[3] = 0.0;
  IP_pos_->CurrentAccelerationVector->VecData[4] = 0.0;
  IP_pos_->CurrentAccelerationVector->VecData[5] = 0.0;

  IP_pos_->MaxVelocityVector->VecData[0] = V_maximumVelocity_[0];
  IP_pos_->MaxVelocityVector->VecData[1] = V_maximumVelocity_[1];
  IP_pos_->MaxVelocityVector->VecData[2] = V_maximumVelocity_[2];
  IP_pos_->MaxVelocityVector->VecData[3] = V_maximumVelocity_[3];
  IP_pos_->MaxVelocityVector->VecData[4] = V_maximumVelocity_[4];
  IP_pos_->MaxVelocityVector->VecData[5] = V_maximumVelocity_[5];

  IP_pos_->MaxAccelerationVector->VecData[0] = V_maximumAcceleration_[0];
  IP_pos_->MaxAccelerationVector->VecData[1] = V_maximumAcceleration_[1];
  IP_pos_->MaxAccelerationVector->VecData[2] = V_maximumAcceleration_[2];
  IP_pos_->MaxAccelerationVector->VecData[3] = V_maximumAcceleration_[3];
  IP_pos_->MaxAccelerationVector->VecData[4] = V_maximumAcceleration_[4];
  IP_pos_->MaxAccelerationVector->VecData[5] = V_maximumAcceleration_[5];

  IP_pos_->TargetPositionVector->VecData[0] = V_desiredPosition_[0];
  IP_pos_->TargetPositionVector->VecData[1] = V_desiredPosition_[1];
  IP_pos_->TargetPositionVector->VecData[2] = V_desiredPosition_[2];
  IP_pos_->TargetPositionVector->VecData[3] = V_desiredPosition_[3];
  IP_pos_->TargetPositionVector->VecData[4] = V_desiredPosition_[4];
  IP_pos_->TargetPositionVector->VecData[5] = V_desiredPosition_[5];

  IP_pos_->TargetVelocityVector->VecData[0] = V_desiredVelocity_[0];
  IP_pos_->TargetVelocityVector->VecData[1] = V_desiredVelocity_[1];
  IP_pos_->TargetVelocityVector->VecData[2] = V_desiredVelocity_[2];
  IP_pos_->TargetVelocityVector->VecData[3] = V_desiredVelocity_[3];
  IP_pos_->TargetVelocityVector->VecData[4] = V_desiredVelocity_[4];
  IP_pos_->TargetVelocityVector->VecData[5] = V_desiredVelocity_[5];

  IP_pos_->SelectionVector->VecData[0] = true;
  IP_pos_->SelectionVector->VecData[1] = true;
  IP_pos_->SelectionVector->VecData[2] = true;
  IP_pos_->SelectionVector->VecData[3] = true;
  IP_pos_->SelectionVector->VecData[4] = true;
  IP_pos_->SelectionVector->VecData[5] = true;

  *IP_vel_->CurrentPositionVector = *IP_pos_->CurrentPositionVector;
  *IP_vel_->CurrentVelocityVector = *IP_pos_->CurrentVelocityVector;
  *IP_vel_->CurrentAccelerationVector = *IP_pos_->CurrentAccelerationVector;
  *IP_vel_->MaxAccelerationVector = *IP_pos_->MaxAccelerationVector;
  *IP_vel_->SelectionVector = *IP_pos_->SelectionVector;

  IP_vel_->TargetVelocityVector->VecData[0] = V_explicitVelocity_[0];
  IP_vel_->TargetVelocityVector->VecData[1] = V_explicitVelocity_[1];
  IP_vel_->TargetVelocityVector->VecData[2] = V_explicitVelocity_[2];
  IP_vel_->TargetVelocityVector->VecData[3] = V_explicitVelocity_[3];
  IP_vel_->TargetVelocityVector->VecData[4] = V_explicitVelocity_[4];
  IP_vel_->TargetVelocityVector->VecData[5] = V_explicitVelocity_[5];

  controlThread_ = std::thread(&TaskSpacePlanner::controlLoop, this);
  while (!isRunning_) Erl::sleep_ms(1);
}
void TaskSpacePlanner::start(const double &cycle_Time_ms,
                             const int &cycleSleep_ms,
                             const Erl::Transformd &initialPose,
                             const Erl::Vector6d &maximumVelocity,
                             const Erl::Vector6d &maximumAcceleration) {
  cycleTime_ms_ = cycle_Time_ms;
  cycleSleep_ms_ = cycleSleep_ms;
  start(initialPose, maximumVelocity, maximumAcceleration);
}

int TaskSpacePlanner::stop() {
  std::lock_guard<std::mutex> lock(initLock_);

  if (isRunning_) {
    isRunning_ = false;
    controlThread_.join();
    cleanup();
  }

  return spinner_.getMissedSpins();
}

Erl::Vector3d TaskSpacePlanner::getDesiredPosition() const {
  std::lock_guard<std::mutex> lock(inputLock_);
  return V_desiredPosition_.getVector1();
}

Erl::Quaterniond TaskSpacePlanner::getDesiredOrientation() const {
  std::lock_guard<std::mutex> lock(inputLock_);
  return Erl::Quaterniond::fromRotationVector(V_desiredPosition_.getVector2());
}

Erl::Transformd TaskSpacePlanner::getDesiredTransform() const {
  std::lock_guard<std::mutex> lock(inputLock_);
  Erl::Vector3d T = V_desiredPosition_.getVector1();
  Erl::Quaterniond Q =
      Erl::Quaterniond::fromRotationVector(V_desiredPosition_.getVector2());
  return Erl::Transformd(Q, T);
}

Erl::Vector6d TaskSpacePlanner::getDesiredVelocity() const {
  std::lock_guard<std::mutex> lock(inputLock_);
  Erl::Vector6d ret(V_desiredVelocity_.getVector1(),
                    Erl::toDegree(V_desiredVelocity_.getVector2()));
  return ret;
}

Erl::Vector6d TaskSpacePlanner::getExplicitVelocity() const {
  std::lock_guard<std::mutex> lock(inputLock_);
  Erl::Vector6d ret(V_explicitVelocity_.getVector1(),
                    Erl::toDegree(V_explicitVelocity_.getVector2()));
  return ret;
}

Erl::Vector3d TaskSpacePlanner::getCurrentPosition() const {
  std::lock_guard<std::mutex> lock(currentLock_);
  return V_currentPosition_.getVector1();
}

Erl::Quaterniond TaskSpacePlanner::getCurrentOrientation() const {
  std::lock_guard<std::mutex> lock(currentLock_);
  return Erl::Quaterniond::fromRotationVector(V_currentPosition_.getVector2());
}

Erl::Transformd TaskSpacePlanner::getCurrentTransform() const {
  std::lock_guard<std::mutex> lock(currentLock_);
  Erl::Vector3d T = V_currentPosition_.getVector1();
  Erl::Quaterniond Q =
      Erl::Quaterniond::fromRotationVector(V_currentPosition_.getVector2());
  return Erl::Transformd(Q, T);
}

Erl::Vector6d TaskSpacePlanner::getCurrentVelocity() const {
  std::lock_guard<std::mutex> lock(currentLock_);
  Erl::Vector6d ret(V_currentVelocity_.getVector1(),
                    Erl::toDegree(V_currentVelocity_.getVector2()));
  return ret;
}

Erl::Vector6d TaskSpacePlanner::getCurrentAcceleration() const {
  std::lock_guard<std::mutex> lock(currentLock_);
  Erl::Vector6d ret(V_currentAcceleration_.getVector1(),
                    Erl::toDegree(V_currentAcceleration_.getVector2()));
  return ret;
}

Erl::Vector6d TaskSpacePlanner::getMaximumVelocity() const {
  Erl::Vector6d ret;
  std::lock_guard<std::mutex> lock(inputLock_);
  ret[0] = V_maximumVelocity_[0];
  ret[1] = V_maximumVelocity_[1];
  ret[2] = V_maximumVelocity_[2];
  ret[3] = Erl::toDegree(V_maximumVelocity_[3]);
  ret[4] = Erl::toDegree(V_maximumVelocity_[4]);
  ret[5] = Erl::toDegree(V_maximumVelocity_[5]);
  return ret;
}

Erl::Vector6d TaskSpacePlanner::getMaximumAcceleration() const {
  Erl::Vector6d ret;
  std::lock_guard<std::mutex> lock(inputLock_);
  ret[0] = V_maximumAcceleration_[0];
  ret[1] = V_maximumAcceleration_[1];
  ret[2] = V_maximumAcceleration_[2];
  ret[3] = Erl::toDegree(V_maximumAcceleration_[3]);
  ret[4] = Erl::toDegree(V_maximumAcceleration_[4]);
  ret[5] = Erl::toDegree(V_maximumAcceleration_[5]);
  return ret;
}

void TaskSpacePlanner::setDesiredPosition(const Erl::Vector3d &position) {
  if (controlMode_ >= CONTROL_JOINT_POSITION) exitJointMode();

  {
    std::lock_guard<std::mutex> lock(inputLock_);
    V_desiredPosition_.setVector1(position);
    checkReached_External_ = true;
  }
  controlMode_ = CONTROL_CARTESIAN_POSITION;
}

void TaskSpacePlanner::setDesiredOrientation(
    const Erl::Quaterniond &orientation) {
  if (controlMode_ >= CONTROL_JOINT_POSITION) exitJointMode();

  currentLock_.lock();
  Erl::Quaterniond Qct = Qcurr_;
  currentLock_.unlock();

  {
    std::lock_guard<std::mutex> lock(inputLock_);
    if (Qct.dot(orientation) > 0)
      V_desiredPosition_.setVector2(orientation.getRotationVector());
    else
      V_desiredPosition_.setVector2((-orientation).getRotationVector());
    checkReached_External_ = true;
  }
  controlMode_ = CONTROL_CARTESIAN_POSITION;
}

void TaskSpacePlanner::setDesiredTransform(const Erl::Transformd &pose) {
  if (controlMode_ >= CONTROL_JOINT_POSITION) exitJointMode();

  currentLock_.lock();
  Erl::Quaterniond Qct = Qcurr_;
  currentLock_.unlock();

  {
    std::lock_guard<std::mutex> lock(inputLock_);
    V_desiredPosition_.setVector1(pose.getTranslation());
    Erl::Quaterniond Qorient = pose.getQuaternion();
    if (Qct.dot(Qorient) > 0)
      V_desiredPosition_.setVector2(Qorient.getRotationVector());
    else
      V_desiredPosition_.setVector2((-Qorient).getRotationVector());
    checkReached_External_ = true;
  }
  controlMode_ = CONTROL_CARTESIAN_POSITION;
}

void TaskSpacePlanner::setExplicitVelocity(const Erl::Vector6d &velocity) {
  if (controlMode_ >= CONTROL_JOINT_POSITION) exitJointMode();

  {
    std::lock_guard<std::mutex> lock(inputLock_);
    V_explicitVelocity_.setVector1(velocity.getVector1());
    V_explicitVelocity_.setVector2(Erl::toRadian(velocity.getVector2()));
    checkReached_External_ = true;
  }
  controlMode_ = CONTROL_CARTESIAN_VELOCITY;
}

void TaskSpacePlanner::setDesiredVelocity(const Erl::Vector6d &velocity) {
  std::lock_guard<std::mutex> lock(inputLock_);
  V_desiredVelocity_.setVector1(velocity.getVector1());
  V_desiredVelocity_.setVector2(Erl::toRadian(velocity.getVector2()));
  checkReached_External_ = true;
}

void TaskSpacePlanner::setMaximumVelocity(
    const Erl::Vector6d &maximumVelocity) {
  std::lock_guard<std::mutex> lock(inputLock_);

  V_maximumVelocity_[0] = maximumVelocity[0];
  V_maximumVelocity_[1] = maximumVelocity[1];
  V_maximumVelocity_[2] = maximumVelocity[2];
  V_maximumVelocity_[3] = Erl::toRadian(maximumVelocity[3]);
  V_maximumVelocity_[4] = Erl::toRadian(maximumVelocity[4]);
  V_maximumVelocity_[5] = Erl::toRadian(maximumVelocity[5]);
}

void TaskSpacePlanner::setMaximumAcceleration(
    const Erl::Vector6d &maximumAcceleration) {
  std::lock_guard<std::mutex> lock(inputLock_);

  V_maximumAcceleration_[0] = maximumAcceleration[0];
  V_maximumAcceleration_[1] = maximumAcceleration[1];
  V_maximumAcceleration_[2] = maximumAcceleration[2];
  V_maximumAcceleration_[3] = Erl::toRadian(maximumAcceleration[3]);
  V_maximumAcceleration_[4] = Erl::toRadian(maximumAcceleration[4]);
  V_maximumAcceleration_[5] = Erl::toRadian(maximumAcceleration[5]);
}

void TaskSpacePlanner::setCurrentPosition(const Erl::Vector3d &position) {
  std::lock_guard<std::mutex> lock(currentLock_);
  V_manualPosition_.setVector1(position);
  manualUpdate_Position_ = true;
}

void TaskSpacePlanner::setCurrentOrientation(
    const Erl::Quaterniond &orientation) {
  std::lock_guard<std::mutex> lock(currentLock_);
  if (Qcurr_.dot(orientation) > 0)
    V_manualPosition_.setVector2(orientation.getRotationVector());
  else
    V_manualPosition_.setVector2((-orientation).getRotationVector());
  manualUpdate_Orientation_ = true;
}

void TaskSpacePlanner::setCurrentTransform(const Erl::Transformd &pose) {
  std::lock_guard<std::mutex> lock(currentLock_);
  V_manualPosition_.setVector1(pose.getTranslation());
  Erl::Quaterniond Qorient = pose.getQuaternion();
  if (Qcurr_.dot(Qorient) > 0)
    V_manualPosition_.setVector2(Qorient.getRotationVector());
  else
    V_manualPosition_.setVector2((-Qorient).getRotationVector());
  manualUpdate_Position_ = true;
  manualUpdate_Orientation_ = true;
}

void TaskSpacePlanner::waitForDestinationReached() const {
  while (checkReached_External_) Erl::sleep_ms(5);
}

bool TaskSpacePlanner::isDestinationReached() const {
  return !checkReached_External_;
}

void TaskSpacePlanner::cleanup() {
  delete RML_;
  RML_ = nullptr;
  delete IP_pos_;
  IP_pos_ = nullptr;
  delete OP_pos_;
  OP_pos_ = nullptr;
  delete IP_vel_;
  IP_vel_ = nullptr;
  delete OP_vel_;
  OP_vel_ = nullptr;
}

void TaskSpacePlanner::updateExternalCommands() {
  std::lock_guard<std::mutex> lock(inputLock_);

  updateExternalCommands_locked();

  IP_pos_->MaxVelocityVector->VecData[0] = V_maximumVelocity_[0];
  IP_pos_->MaxVelocityVector->VecData[1] = V_maximumVelocity_[1];
  IP_pos_->MaxVelocityVector->VecData[2] = V_maximumVelocity_[2];
  IP_pos_->MaxVelocityVector->VecData[3] = V_maximumVelocity_[3];
  IP_pos_->MaxVelocityVector->VecData[4] = V_maximumVelocity_[4];
  IP_pos_->MaxVelocityVector->VecData[5] = V_maximumVelocity_[5];

  IP_pos_->MaxAccelerationVector->VecData[0] = V_maximumAcceleration_[0];
  IP_pos_->MaxAccelerationVector->VecData[1] = V_maximumAcceleration_[1];
  IP_pos_->MaxAccelerationVector->VecData[2] = V_maximumAcceleration_[2];
  IP_pos_->MaxAccelerationVector->VecData[3] = V_maximumAcceleration_[3];
  IP_pos_->MaxAccelerationVector->VecData[4] = V_maximumAcceleration_[4];
  IP_pos_->MaxAccelerationVector->VecData[5] = V_maximumAcceleration_[5];

  IP_pos_->TargetPositionVector->VecData[0] = V_desiredPosition_[0];
  IP_pos_->TargetPositionVector->VecData[1] = V_desiredPosition_[1];
  IP_pos_->TargetPositionVector->VecData[2] = V_desiredPosition_[2];
  IP_pos_->TargetPositionVector->VecData[3] = V_desiredPosition_[3];
  IP_pos_->TargetPositionVector->VecData[4] = V_desiredPosition_[4];
  IP_pos_->TargetPositionVector->VecData[5] = V_desiredPosition_[5];

  IP_pos_->TargetVelocityVector->VecData[0] = V_desiredVelocity_[0];
  IP_pos_->TargetVelocityVector->VecData[1] = V_desiredVelocity_[1];
  IP_pos_->TargetVelocityVector->VecData[2] = V_desiredVelocity_[2];
  IP_pos_->TargetVelocityVector->VecData[3] = V_desiredVelocity_[3];
  IP_pos_->TargetVelocityVector->VecData[4] = V_desiredVelocity_[4];
  IP_pos_->TargetVelocityVector->VecData[5] = V_desiredVelocity_[5];

  IP_vel_->TargetVelocityVector->VecData[0] = V_explicitVelocity_[0];
  IP_vel_->TargetVelocityVector->VecData[1] = V_explicitVelocity_[1];
  IP_vel_->TargetVelocityVector->VecData[2] = V_explicitVelocity_[2];
  IP_vel_->TargetVelocityVector->VecData[3] = V_explicitVelocity_[3];
  IP_vel_->TargetVelocityVector->VecData[4] = V_explicitVelocity_[4];
  IP_vel_->TargetVelocityVector->VecData[5] = V_explicitVelocity_[5];

  checkReached_Internal_ = checkReached_External_;
}

void TaskSpacePlanner::updateInputs() {
  updateExternalCommands();
  updateSelfInputs();

  *IP_vel_->CurrentPositionVector = *IP_pos_->CurrentPositionVector;
  *IP_vel_->CurrentVelocityVector = *IP_pos_->CurrentVelocityVector;
  *IP_vel_->CurrentAccelerationVector = *IP_pos_->CurrentAccelerationVector;
  *IP_vel_->MaxAccelerationVector = *IP_pos_->MaxAccelerationVector;
}

void TaskSpacePlanner::updateSelfInputs_locked() {
  if (manualUpdate_Position_) {
    V_currentPosition_.setVector1(V_manualPosition_.getVector1());
    manualUpdate_Position_ = false;
  }
  if (manualUpdate_Orientation_) {
    V_currentPosition_.setVector2(V_manualPosition_.getVector2());
    manualUpdate_Orientation_ = false;
  }
}

void TaskSpacePlanner::updateSelfInputs() {
  {
    std::lock_guard<std::mutex> lock(currentLock_);

    updateSelfInputs_locked();

    IP_pos_->CurrentPositionVector->VecData[0] = V_currentPosition_[0];
    IP_pos_->CurrentPositionVector->VecData[1] = V_currentPosition_[1];
    IP_pos_->CurrentPositionVector->VecData[2] = V_currentPosition_[2];
    IP_pos_->CurrentPositionVector->VecData[3] = V_currentPosition_[3];
    IP_pos_->CurrentPositionVector->VecData[4] = V_currentPosition_[4];
    IP_pos_->CurrentPositionVector->VecData[5] = V_currentPosition_[5];
  }

  *IP_pos_->CurrentVelocityVector = *OP_pos_->NewVelocityVector;
  *IP_pos_->CurrentAccelerationVector = *OP_pos_->NewAccelerationVector;
}

void TaskSpacePlanner::updateOutputs(const int &controlMode) {
  std::lock_guard<std::mutex> lock(currentLock_);

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
  checkReached_External_ = checkReached_Internal_;
}

void TaskSpacePlanner::controlLoop() {
  isRunning_ = true;
  spinner_.reset();

  while (isRunning_) {
    {
      std::lock_guard<std::mutex> lock(loopLock_);

      int controlMode = controlMode_;
      updateInputs();

      if (controlMode == CONTROL_CARTESIAN_VELOCITY)
        RML_result_ = RML_->RMLVelocity(*IP_vel_, OP_vel_, Flags_vel_);
      else if (controlMode == CONTROL_CARTESIAN_POSITION)
        RML_result_ = RML_->RMLPosition(*IP_pos_, OP_pos_, Flags_pos_);

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
    }

    if (cycleSleep_ms_ > 0) Erl::sleep_ms(cycleSleep_ms_);
    spinner_.spinUpWithInfo();
  }

  if (RML_result_ < 0) cleanup();
}
