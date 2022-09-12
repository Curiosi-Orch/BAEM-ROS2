#ifndef TASKSPACEPLANNER_H
#define TASKSPACEPLANNER_H

#include <Erl.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <ReflexxesAPI.h>

#include <thread>

class TaskSpacePlanner {
 public:
  enum {
    CONTROL_CARTESIAN_POSITION = 0,
    CONTROL_CARTESIAN_VELOCITY = 1,
    CONTROL_JOINT_POSITION = 2,
    CONTROL_JOINT_VELOCITY = 3
  };

  TaskSpacePlanner(const double &cycle_Time_ms, const int &cycleSleep_ms);
  TaskSpacePlanner();
  ~TaskSpacePlanner();

  void start(const Erl::Transformd &initialPose,
             const Erl::Vector6d &maximumVelocity,
             const Erl::Vector6d &maximumAcceleration);
  void start(const double &cycle_Time_ms, const int &cycleSleep_ms,
             const Erl::Transformd &initialPose,
             const Erl::Vector6d &maximumVelocity,
             const Erl::Vector6d &maximumAcceleration);
  int stop();

  Erl::Vector3d getDesiredPosition() const;
  Erl::Quaterniond getDesiredOrientation() const;
  Erl::Transformd getDesiredTransform() const;
  Erl::Vector6d getDesiredVelocity() const;
  Erl::Vector6d getExplicitVelocity() const;

  Erl::Vector3d getCurrentPosition() const;
  Erl::Quaterniond getCurrentOrientation() const;
  Erl::Transformd getCurrentTransform() const;

  Erl::Vector6d getCurrentVelocity() const;
  Erl::Vector6d getCurrentAcceleration() const;
  Erl::Vector6d getMaximumVelocity() const;
  Erl::Vector6d getMaximumAcceleration() const;

  inline double getCycleTime_ms() const { return cycleTime_ms_; }
  inline double getCycleSleep_ms() const { return cycleSleep_ms_; }

  bool isRunning() const { return isRunning_; }
  bool isUsingExplicitVelocity() const {
    return controlMode_ == CONTROL_CARTESIAN_VELOCITY;
  }
  int getControlMode() const { return controlMode_; }

  void setDesiredPosition(const Erl::Vector3d &position);
  void setDesiredOrientation(const Erl::Quaterniond &orientation);
  void setDesiredTransform(const Erl::Transformd &pose);
  void setDesiredVelocity(const Erl::Vector6d &velocity);
  void setExplicitVelocity(const Erl::Vector6d &velocity);
  void setMaximumVelocity(const Erl::Vector6d &maximumVelocity);
  void setMaximumAcceleration(const Erl::Vector6d &maximumAcceleration);

  virtual void setCurrentPosition(const Erl::Vector3d &position);
  virtual void setCurrentOrientation(const Erl::Quaterniond &orientation);
  virtual void setCurrentTransform(const Erl::Transformd &pose);

  void waitForDestinationReached() const;
  bool isDestinationReached() const;

  std::thread *getThread() { return &controlThread_; }

 protected:
  void updateInputs();
  virtual void updateOutputs(const int &controlMode);
  virtual void updateSelfInputs();
  void updateExternalCommands();

  inline virtual void updateExternalCommands_locked() {}
  virtual void updateSelfInputs_locked();
  virtual void cleanup();
  virtual void controlLoop();
  virtual void exitJointMode() {}

  const int numDofs_;
  double cycleTime_ms_;
  int cycleSleep_ms_;
  Erl::Spinner_ms spinner_;

  Erl::Vector6d V_currentPosition_;
  Erl::Vector6d V_currentVelocity_;
  Erl::Vector6d V_currentAcceleration_;

  Erl::Vector6d V_desiredPosition_;
  Erl::Vector6d V_desiredVelocity_;
  Erl::Vector6d V_explicitVelocity_;

  Erl::Vector6d V_maximumVelocity_;
  Erl::Vector6d V_maximumAcceleration_;

  Erl::Quaterniond Qcurr_;

  Erl::Vector6d V_manualPosition_;
  bool manualUpdate_Position_;
  bool manualUpdate_Orientation_;

  /// Locking order is highest to lowest.
  mutable std::mutex initLock_;
  mutable std::mutex loopLock_;
  mutable std::mutex inputLock_;
  mutable std::mutex currentLock_;

  std::atomic<bool> isRunning_;
  std::atomic<bool> needUpdate_;
  std::thread controlThread_;

  ReflexxesAPI *RML_;
  RMLPositionInputParameters *IP_pos_;
  RMLPositionOutputParameters *OP_pos_;
  RMLPositionFlags Flags_pos_;
  RMLVelocityInputParameters *IP_vel_;
  RMLVelocityOutputParameters *OP_vel_;
  RMLVelocityFlags Flags_vel_;
  int RML_result_;
  std::atomic<int> controlMode_;

  bool checkReached_Internal_;
  mutable std::atomic<bool> checkReached_External_;
};

#endif
