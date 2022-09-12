#ifndef IIWAPLANNER_H
#define IIWAPLANNER_H

#include "iiwa/planner/taskspaceplanner.h"
#include "iiwa/sunrise/kukasunrise.h"

class IIwaPlanner : public TaskSpacePlanner {
 public:
  typedef Eigen::Matrix<double, 7, 1> Vector7d;

  IIwaPlanner(const double &cycle_Time_ms, const int &cycleSleep_ms)
      : TaskSpacePlanner(cycle_Time_ms, cycleSleep_ms),
        kuka_(nullptr),
        engaged_(false),
        RML_joint_(nullptr) {}

  IIwaPlanner()
      : TaskSpacePlanner(),
        kuka_(nullptr),
        engaged_(false),
        RML_joint_(nullptr) {}

  virtual ~IIwaPlanner() {}

  void start(KukaSunrise *kuka, const Erl::Vector6d &maximumVelocity,
             const Erl::Vector6d &maximumAcceleration,
             const Vector7d &maximumJointVelocity,
             const Vector7d &maximumJointAcceleration);
  void start(const double &cycle_Time_ms, const int &cycleSleep_ms,
             KukaSunrise *kuka, const Erl::Vector6d &maximumVelocity,
             const Erl::Vector6d &maximumAcceleration,
             const Vector7d &maximumJointVelocity,
             const Vector7d &maximumJointAcceleration);

  void setDesiredJointPosition(const Vector7d &joints);
  void setExplicitJointVelocity(const Vector7d &velocity);
  void setMaximumJointVelocity(const Vector7d &maximumJointVelocity);
  void setMaximumJointAcceleration(const Vector7d &maximumJointAcceleration);

  Vector7d getCurrentJointVelocity() const;
  Vector7d getCurrentJointAcceleration() const;
  Vector7d getMaximumJointVelocity() const;
  Vector7d getMaximumJointAcceleration() const;
  void setEngaged(const bool &engaged);

  inline void engage() { setEngaged(true); }
  inline void disengage() { setEngaged(false); }
  inline bool isEngaged() const { return engaged_; }

 protected:
  void startJointPlanner(const Vector7d &maximumJointVelocity,
                         const Vector7d &maximumJointAcceleration);

  void setDesiredTransformFromZero(const Erl::Transformd &pose);
  void setExplicitVelocityFromZero(const Erl::Vector6d &velocity);
  void setDesiredJointsFromZero(const Vector7d &joints);
  void setExplicitJointVelocityFromZero(const Vector7d &velocity);
  virtual void exitJointMode() override;

  void cleanup() {
    engaged_ = false;
    kuka_ = nullptr;
    delete RML_;
    RML_ = nullptr;
    delete RML_joint_;
    RML_joint_ = nullptr;
    delete IP_pos_;
    IP_pos_ = nullptr;
    delete OP_pos_;
    OP_pos_ = nullptr;
    delete IP_vel_;
    IP_vel_ = nullptr;
    delete OP_vel_;
    OP_vel_ = nullptr;
    delete IP_joint_;
    IP_joint_ = nullptr;
    delete OP_joint_;
    OP_joint_ = nullptr;
    delete IP_joint_vel_;
    IP_joint_vel_ = nullptr;
    delete OP_joint_vel_;
    OP_joint_vel_ = nullptr;
  }

  template <typename T>
  void setCurrentPosition(const Erl::Vector3d &position) {
    static_assert(__internal_fail<T>::value,
                  "Do not manually set present values on IIwaPlanner. The kuka "
                  "updates the planner directly.");
  }
  template <typename T>
  void setCurrentOrientation(const Erl::Quaterniond &orientation) {
    static_assert(__internal_fail<T>::value,
                  "Do not manually set present values on IIwaPlanner. The kuka "
                  "updates the planner directly.");
  }
  template <typename T>
  void setCurrentTransform(const Erl::Transformd &pose) {
    static_assert(__internal_fail<T>::value,
                  "Do not manually set present values on IIwaPlanner. The kuka "
                  "updates the planner directly.");
  }

  virtual void updateExternalCommands_locked();
  virtual void updateSelfInputs();
  virtual void updateSelfInputs_locked();
  virtual void controlLoop();
  virtual void updateOutputs(const int &controlMode);

  IIwaPlanner(const IIwaPlanner &K) = delete;
  IIwaPlanner &operator=(const IIwaPlanner &K) = delete;

  KukaSunrise *kuka_;
  std::atomic<bool> engaged_;

  Vector7d V_currentJointPosition_;
  Vector7d V_currentJointVelocity_;
  Vector7d V_currentJointAcceleration_;

  Vector7d V_desiredJointPosition_;
  Vector7d V_explicitJointVelocity_;
  Vector7d V_maximumJointVelocity_;
  Vector7d V_maximumJointAcceleration_;

  ReflexxesAPI *RML_joint_;
  RMLPositionInputParameters *IP_joint_;
  RMLPositionOutputParameters *OP_joint_;
  RMLPositionFlags Flags_joint_;

  RMLVelocityInputParameters *IP_joint_vel_;
  RMLVelocityOutputParameters *OP_joint_vel_;
  RMLVelocityFlags Flags_joint_vel_;

  template <typename T>
  struct __internal_fail : std::false_type {};
};

#endif
