#ifndef ROBOT_UTILITIES_H_
#define ROBOT_UTILITIES_H_

#include <Erl.h>

#include "utils/utilities.h"

namespace robot {

void AppendData2File(std::string file_path, Eigen::VectorXd data);
bool LoadMatrixFromFile(std::string file_path, Eigen::MatrixXd* data);

void RotationMatrix2RPY(Eigen::Matrix3d R, Eigen::Vector3d* rpy);
void RotationMatrix2AngleAxis(Eigen::Matrix3d R, double* angle,
                              Eigen::Vector3d* axis);
void RPY2Quaternion(Eigen::Vector3d rpy, Eigen::Quaterniond* q);
void Quaternion2RPY(Eigen::Quaterniond q, Eigen::Vector3d* rpy);

void RotationMatrix2Quaternion(Eigen::Matrix3d R, Eigen::Quaterniond* q);
void Quaternion2RotationMatrix(Eigen::Quaterniond q, Eigen::Matrix3d* R);

Eigen::VectorXd GetTwistBetweenPoses(Eigen::Matrix4d pose_current,
                                     Eigen::Matrix4d pose_target,
                                     double dt = 1);
/**
 * @brief: function for computing pseudoinverse
 * @param J = generic matrix
 */
// Eigen::MatrixXd GetPseudoInverse(Eigen::MatrixXd J, double damping = 1e-03);
Eigen::MatrixXd GetPseudoInverse(
    const Eigen::MatrixXd& A,
    double epsilon = std::numeric_limits<double>::epsilon());

/**
 * @param params: d,theta,a,alpha
 */
Eigen::Matrix4d GenerateDHMatrix(Eigen::VectorXd params);

Eigen::MatrixXd Erl2Eigen(Erl::Transformd T_erl);
Erl::Transformd Eigen2Erl(Eigen::MatrixXd T_eigen);

template <class T>
T Deg2Rad(T deg) {
  return deg * M_PI / 180.;
}

template <class T>
T Rad2Deg(T rad) {
  return rad * 180. / M_PI;
}

Eigen::Matrix4d _ForwardKinematics(Eigen::MatrixXd DH_table, Eigen::VectorXd q,
                                   std::vector<char> joint_type,
                                   std::string model_type = "DH");

Eigen::Matrix4d ForwardKinematics(Eigen::MatrixXd DH_table, Eigen::VectorXd q,
                                  std::vector<char> joint_type,
                                  std::string model_type = "DH");

Eigen::Matrix4d ForwardKinematics(Eigen::MatrixXd DH_table, Eigen::VectorXd q,
                                  std::string model_type = "DH");

Eigen::Matrix4d RotationTransformation(char axis, double radian);

Eigen::Matrix4d TranslateTransformation(Eigen::Vector3d tr);

bool ClampJointLimits(std::vector<Eigen::VectorXd> joint_angle_limits,
                      Eigen::VectorXd* q);

// Jacobian of end effector
Eigen::MatrixXd GetJacobn(Eigen::MatrixXd DH_table, Eigen::VectorXd q);

// Jacobian of base link
Eigen::MatrixXd GetJacob0(Eigen::MatrixXd DH_table, Eigen::VectorXd q,
                          std::vector<char> joint_type);

Eigen::VectorXd InverseKinematics(
    Eigen::MatrixXd DH_table, Eigen::MatrixXd T, Eigen::VectorXd q_ref,
    std::vector<Eigen::VectorXd> joint_angle_limits,
    std::vector<char> joint_type, Eigen::VectorXd axis_filter,
    std::string model_type = "DH");

Eigen::VectorXd InverseKinematics(Eigen::MatrixXd DH_table, Eigen::MatrixXd T,
                                  Eigen::VectorXd q_ref,
                                  std::string model_type = "DH");

Eigen::VectorXd _InverseKinematics(
    Eigen::MatrixXd DH_table, Eigen::MatrixXd T, Eigen::VectorXd q_ref,
    std::vector<Eigen::VectorXd> joint_angle_limits,
    std::vector<char> joint_type, Eigen::VectorXd axis_filter,
    std::string model_type = "DH");

// /**
//  * @brief IK solver at velocity level, gievn desired twist
//  * @param twist in R^6x1 = linear and angular velocity vector
//  * @param axis_filter [x,y,z,wx,wy,wz], axis filter, i.e., [1,1,1,1,1,1] ->
//  * control all; [1,1,0,0,0,0] -> Control X,Y, others will change by
//  * theirselves
//  * @param ee_frame sets if pose expressed in ee frame or in base frame
//  */
// void SolveIK_vel(Eigen::VectorXd twist, Eigen::VectorXd q_now,
//                  Eigen::VectorXd axis_filter, Eigen::VectorXd* q_out,
//                  Eigen::VectorXd* qd_out, double damping = 1e-03,
//                  double dt = 1e-03, bool ee_frame = false);

// /**
//  * @brief IK solver at velocity level, gievn desired twist, recursivel. If
//  * joints reach limit and task cannot be fulfilled just sends 0 speed
//  * @param twist in R^6x1 = linear and angular velocity vector
//  * @param axis_filter [x,y,z,wx,wy,wz], axis filter, i.e., [1,1,1,1,1,1] ->
//  * control all; [1,1,0,0,0,0] -> Control X,Y, others will change by
//  * theirselves
//  * @param ee_frame sets if pose expressed in ee frame or in base frame
//  */
// void SolveIK_vel_recurs(Eigen::VectorXd twist, Eigen::VectorXd q_now,
//                         Eigen::VectorXd axis_filter, Eigen::VectorXd* q_out,
//                         Eigen::VectorXd* qd_out, double damping = 1e-03,
//                         double dt = 1e-03, bool ee_frame = false);

// /**
//  * @brief Solves IK at velocity level and scales down joint velocities if
//  * above limit
//  * @param twist in R^6x1 = linear and angular velocity vector
//  * @param axis_filter [x,y,z,wx,wy,wz], axis filter, i.e., [1,1,1,1,1,1] ->
//  * control all; [1,1,0,0,0,0] -> Control X,Y, others will change by
//  * theirselves
//  * @param ee_frame sets if pose expressed in ee frame or in base frame
//  */
// void SolveIK_vel_scaled(Eigen::VectorXd twist, Eigen::VectorXd q_now,
//                         Eigen::VectorXd axis_filter, Eigen::VectorXd* q_out,
//                         Eigen::VectorXd* qd_out,
//                         Eigen::VectorXd* twist_scaled, double damping =
//                         1e-03, double dt = 1e-03, bool ee_frame = false);

bool ReadDHTable(const std::string& file_path, Eigen::MatrixXd* DH_table);

bool CheckInBoundingBox(Eigen::MatrixXd DH_table, Eigen::VectorXd q,
                        std::vector<Eigen::Vector3d> workspace_limits,
                        std::vector<char> joint_type,
                        std::string model_type = "DH");

}  // namespace robot

#endif  // ROBOT_UTILITIES_H_