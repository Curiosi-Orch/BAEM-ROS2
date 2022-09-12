#include "utils/robot_utilities.h"

namespace robot {

void AppendData2File(std::string file_path, Eigen::VectorXd data) {
  std::string str;
  for (int i = 0; i < data.rows(); i++) {
    str += std::to_string(data[i]);
    str += "\t";
  }
  AppendString2File(file_path, str);
}

bool LoadMatrixFromFile(std::string file_path, Eigen::MatrixXd* data) {
  std::vector<std::string> str;
  if (!LoadStringFromFile(file_path, &str)) {
    return false;
  }
  for (int i = 0; i < str.size(); ++i) {
    std::vector<double> values = GetValuesFromStringSplit<double>(str[i], " ");
    Eigen::VectorXd input = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        values.data(), values.size());
    data->conservativeResize(i + 1, values.size());
    data->row(i) = input;
  }
  return true;
}

void RotationMatrix2RPY(Eigen::Matrix3d R, Eigen::Vector3d* rpy) {
  Eigen::Vector3d _rpy;
  double sy = std::sqrt(std::pow(R(0, 0), 2.) + std::pow(R(1, 0), 2.));
  double singular = 1e-06;
  if (sy > singular) {
    _rpy(0) = std::atan2(R(2, 1), R(2, 2));
    _rpy(1) = std::atan2(-R(2, 0), sy);
    _rpy(2) = std::atan2(R(1, 0), R(0, 0));
  } else {
    _rpy(0) = std::atan2(-R(1, 2), R(1, 1));
    _rpy(1) = std::atan2(-R(2, 0), sy);
    _rpy(2) = 0.;
  }
  (*rpy) = _rpy;
}

Eigen::MatrixXd Erl2Eigen(Erl::Transformd T_erl) {
  Eigen::MatrixXd T(4, 4);
  T.setZero();
  T(3, 3) = 1;
  T.topLeftCorner(3, 3) = T_erl.getRotation();
  T.col(3).head(3) = T_erl.getTranslation();
  return T;
}

Erl::Transformd Eigen2Erl(Eigen::MatrixXd T_eigen) {
  Erl::Transformd T;
  Eigen::Matrix3d R = T_eigen.topLeftCorner(3, 3);
  Eigen::Vector3d t = T_eigen.col(3).head(3);
  T.setRotation(R);
  T.setTranslation(t);
  return T;
}

Eigen::Matrix4d GenerateDHMatrix(Eigen::VectorXd params) {
  Eigen::Matrix4d T;
  T.setZero();
  T(0, 0) = std::cos(params(1));
  T(1, 0) = std::sin(params(1));
  T(0, 1) = -std::sin(params(1)) * std::cos(params(3));
  T(1, 1) = std::cos(params(1)) * std::cos(params(3));
  T(2, 1) = std::sin(params(3));
  T(0, 2) = std::sin(params(1)) * std::sin(params(3));
  T(1, 2) = -std::cos(params(1)) * std::sin(params(3));
  T(2, 2) = std::cos(params(3));
  T(0, 3) = params(2) * std::cos(params(1));
  T(1, 3) = params(2) * std::sin(params(1));
  T(2, 3) = params(0);
  T(3, 3) = 1;
  return T;
}

void RotationMatrix2AngleAxis(Eigen::Matrix3d R, double* angle,
                              Eigen::Vector3d* axis) {
  double val = (R(0, 0) + R(1, 1) + R(2, 2) - 1.) / 2.;
  val = std::min(val, 1.);
  val = std::max(val, -1.);
  double eps = 1e-03;
  double _theta;
  Eigen::Vector3d r;
  _theta = std::acos(val);

  if (std::abs(std::sin(_theta)) > eps) {
    r << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
    r = 1. / (2. * std::sin(_theta)) * r;
  } else {  // theta = 0, pi
    r.setZero();
    if (std::abs(std::cos(_theta) - 1.) <= eps) {  // theta = 0
      r(2) = 1.;
    } else {
      r(0) = std::sqrt(std::abs((R(0, 0) + 1) / 2.));
      r(1) = std::sqrt(std::abs((R(1, 1) + 1) / 2.));
      r(2) = std::sqrt(std::abs((R(2, 2) + 1) / 2.));

      if (R(0, 1) < 0) {
        r(0) = -r(0);
      }
      if (R(2, 1) < 0) {
        r(1) = -r(1);
      }
      if (r(0) * r(1) * R(0, 1) < 0) {
        r(0) = -r(0);
      }
    }
  }
  *angle = _theta;
  *axis = r;
}

Eigen::VectorXd GetTwistBetweenPoses(Eigen::Matrix4d pose_current,
                                     Eigen::Matrix4d pose_target, double dt) {
  Eigen::VectorXd twist(6);

  Eigen::Vector3d t_current = pose_current.col(3).head(3);
  Eigen::Vector3d t_target = pose_target.col(3).head(3);
  Eigen::Matrix3d R_current = pose_current.topLeftCorner(3, 3);
  Eigen::Matrix3d R_target = pose_target.topLeftCorner(3, 3);

  // difference between current and target
  Eigen::Matrix3d R_error = R_current.transpose() * R_target;
  Eigen::Vector3d t_error = t_target - t_current;

  Eigen::Vector3d r;
  double theta;
  RotationMatrix2AngleAxis(R_error, &theta, &r);
  r = R_current * r;

  twist.head(3) = t_error / dt;
  twist.tail(3) = theta / dt * r;
  return twist;
}

// Eigen::MatrixXd GetPseudoInverse(const Eigen::MatrixXd& A, double epsilon) {
//   Eigen::MatrixXd I = Eigen::MatrixXd::Identity(A.rows(), A.rows());
//   Eigen::MatrixXd H = A * A.transpose() + epsilon * I;
//   Eigen::MatrixXd A_pinv = A.transpose() * H.inverse();
//   return A_pinv;
// }

Eigen::MatrixXd GetPseudoInverse(const Eigen::MatrixXd& A, double epsilon) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  double tolerance = epsilon * std::max(A.cols(), A.rows()) *
                     svd.singularValues().array().abs()(0);
  return svd.matrixV() *
         (svd.singularValues().array().abs() > tolerance)
             .select(svd.singularValues().array().inverse(), 0)
             .matrix()
             .asDiagonal() *
         svd.matrixU().adjoint();
}

void RotationMatrix2Quaternion(Eigen::Matrix3d R, Eigen::Quaterniond* q) {
  *q = Eigen::Quaterniond(R);
}

void Quaternion2RotationMatrix(Eigen::Quaterniond q, Eigen::Matrix3d* R) {
  *R = q.normalized().toRotationMatrix();
}

void RPY2Quaternion(Eigen::Vector3d rpy, Eigen::Quaterniond* q) {
  Eigen::AngleAxisd rollAngle(
      Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(
      Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(
      Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()));

  Eigen::Quaterniond quaternion;
  *q = rollAngle * pitchAngle * yawAngle;
}

void Quaternion2RPY(Eigen::Quaterniond q, Eigen::Vector3d* rpy) {
  *rpy = q.normalized().toRotationMatrix().eulerAngles(0, 1, 2);
}

Eigen::Matrix4d _ForwardKinematics(Eigen::MatrixXd DH_table,
                                   Eigen::VectorXd q, 
                                   std::vector<char> joint_type, 
                                   std::string model_type) {
  Eigen::Matrix4d T = Eigen::MatrixXd::Identity(4, 4);
  for (int i = 0; i < q.rows(); i++) {
    Eigen::VectorXd params = DH_table.row(i);
    if (joint_type[i] == 'p') {
      params(0) += q(i);
    }
    if (joint_type[i] == 'r') {
      params(1) += q(i);
    }
    Eigen::MatrixXd Ti = GenerateDHMatrix(params);
    T *= Ti;
  }
  return T;
}

Eigen::Matrix4d ForwardKinematics(Eigen::MatrixXd DH_table,
                                  Eigen::VectorXd q, 
                                  std::vector<char> joint_type, 
                                  std::string model_type) {
  return _ForwardKinematics(DH_table, q, joint_type, model_type);
}

Eigen::Matrix4d ForwardKinematics(Eigen::MatrixXd DH_table,
                                  Eigen::VectorXd q, 
                                  std::string model_type) {
  int num_DOF = DH_table.rows();
  std::vector<char> joint_type(num_DOF, 'r');
  return _ForwardKinematics(DH_table, q, joint_type, model_type);
}

Eigen::Matrix4d RotationTransformation(char axis, double radian) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity(4, 4);
  double ct = cos(radian);
  double st = sin(radian);

  switch (axis) {
    case 'x':
    case 'X':
      T << 1, 0, 0, 0, 0, ct, -st, 0, 0, st, ct, 0, 0, 0, 0, 1;
      break;
    case 'y':
    case 'Y':
      T << ct, 0, st, 0, 0, 1, 0, 0, -st, 0, ct, 0, 0, 0, 0, 1;
      break;
    case 'z':
    case 'Z':
      T << ct, -st, 0, 0, st, ct, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
      break;
  }
  return T;
}

Eigen::Matrix4d TranslateTransformation(Eigen::Vector3d tr) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity(4, 4);
  T.topRightCorner(3, 1) = tr;
  return T;
}

bool ClampJointLimits(std::vector<Eigen::VectorXd> joint_angle_limits,
                      Eigen::VectorXd* q) {
  bool is_in_limit = true;
  for (int i = 0; i < q->rows(); i++) {
    if ((*q)(i) <= joint_angle_limits[0](i) ||
        (*q)(i) >= joint_angle_limits[1](i)) {
      std::cout << " Joint " << i << " out of limits\n";
      is_in_limit = false;
    }
  }
  // Clamp joint values
  *q = q->cwiseMax(joint_angle_limits[0]);
  *q = q->cwiseMin(joint_angle_limits[1]);
  return is_in_limit;
}

Eigen::MatrixXd GetJacobn(Eigen::MatrixXd DH_table, Eigen::VectorXd q) {
  int num_DOF = DH_table.rows();
  Eigen::MatrixXd Jacobn = Eigen::MatrixXd::Identity(6, num_DOF);
  Eigen::Matrix4d T_i = Eigen::Matrix4d::Identity(4, 4);
  for (int i = DH_table.rows() - 1; i >= 0; --i) {
    Eigen::Vector3d t;
    t << DH_table(i, 2), 0, DH_table(i, 0);
    DH_table(i, 1) = DH_table(i, 1) + q(i);
    T_i = RotationTransformation('z', DH_table(i, 1)) *
          TranslateTransformation(t) *
          RotationTransformation('x', DH_table(i, 3)) * T_i;

    Eigen::Vector3d n, o, a, p;
    a << T_i(0, 0), T_i(1, 0), T_i(2, 0);
    o << T_i(0, 1), T_i(1, 1), T_i(2, 1);
    n << T_i(0, 2), T_i(1, 2), T_i(2, 2);
    p << T_i(0, 3), T_i(1, 3), T_i(2, 3);

    Jacobn(0, i) = -a(0) * p(1) + a(1) * p(0);
    Jacobn(1, i) = -o(0) * p(1) + o(1) * p(0);
    Jacobn(2, i) = -n(0) * p(1) + n(1) * p(0);
    Jacobn(3, i) = a(2);
    Jacobn(4, i) = o(2);
    Jacobn(5, i) = n(2);
  }

  return Jacobn;
}

Eigen::MatrixXd GetJacob0(Eigen::MatrixXd DH_table, Eigen::VectorXd q,
                          std::vector<char> joint_type) {
  Eigen::MatrixXd Jacob0 = GetJacobn(DH_table, q);
  Eigen::MatrixXd T = ForwardKinematics(DH_table, q, joint_type);

  Eigen::MatrixXd Transform(6, 6);
  Transform.setZero();
  Transform.topLeftCorner(3, 3) = T.topLeftCorner(3, 3);
  Transform.bottomRightCorner(3, 3) = T.topLeftCorner(3, 3);
  Jacob0 = Transform * Jacob0;

  return Jacob0;
}

Eigen::VectorXd _InverseKinematics(
    Eigen::MatrixXd DH_table, Eigen::MatrixXd T, Eigen::VectorXd q_ref,
    std::vector<Eigen::VectorXd> joint_angle_limits,
    std::vector<char> joint_type, Eigen::VectorXd axis_filter,
    std::string model_type) {
  Eigen::VectorXd q = q_ref;

  double error = 999.9;
  double min_update = 1e-6;
  int epoch = 0;
  int max_epoch = 1000;
  Eigen::MatrixXd temp_T;
  double dt = 1;
  Eigen::MatrixXd W_task = axis_filter.asDiagonal();

  while (error > min_update) {
    temp_T = ForwardKinematics(DH_table, q, joint_type, model_type);
    Eigen::VectorXd diff = GetTwistBetweenPoses(temp_T, T);
    Eigen::MatrixXd Jacob0 = GetJacob0(DH_table, q, joint_type);
    Eigen::VectorXd dq = GetPseudoInverse(W_task * Jacob0) * (W_task * diff);

    q += dq * dt;
    // atan2 for each joint to be within [-pi,pi]
    for (int i = 0; i < q.rows(); i++) {
      q(i) = std::atan2(std::sin(q(i)), std::cos(q(i)));
    }
    // get the length of dq
    error = dq.norm();

    epoch++;
    if (epoch > max_epoch) {
      std::cout << "Inverse kinematics unsolvable!" << std::endl;
      break;
    }
  }

  ClampJointLimits(joint_angle_limits, &q);
  return q;
}

Eigen::VectorXd InverseKinematics(
    Eigen::MatrixXd DH_table, Eigen::MatrixXd T, Eigen::VectorXd q_ref,
    std::vector<Eigen::VectorXd> joint_angle_limits,
    std::vector<char> joint_type, Eigen::VectorXd axis_filter,
    std::string model_type) {
  return _InverseKinematics(DH_table, T, q_ref, joint_angle_limits, joint_type,
                            axis_filter, model_type);
}

Eigen::VectorXd InverseKinematics(Eigen::MatrixXd DH_table, Eigen::MatrixXd T,
                                  Eigen::VectorXd q_ref,
                                  std::string model_type) {
  int num_DOF = DH_table.rows();
  std::vector<Eigen::VectorXd> joint_angle_limits;
  std::vector<char> joint_type(num_DOF, 'r');
  Eigen::VectorXd axis_filter;

  Eigen::VectorXd temp_limit;
  temp_limit.setOnes(num_DOF);
  temp_limit = temp_limit * 999.9;
  joint_angle_limits.push_back(-temp_limit);
  joint_angle_limits.push_back(temp_limit);

  axis_filter.setOnes(6);

  return _InverseKinematics(DH_table, T, q_ref, joint_angle_limits, joint_type,
                            axis_filter, model_type);
}

bool ReadDHTable(const std::string& file_path, Eigen::MatrixXd* DH_table) {
  Eigen::MatrixXd DH;
  std::vector<std::string> str;
  if (!LoadStringFromFile(file_path, &str)) {
    std::cout << "Open file < " << file_path << " > failed!" << std::endl;
    return false;
  } else {
    std::cout << "Read DH table from < " << file_path << " >" << std::endl;
  }
  DH.resize(str.size(),4);
  for (int i = 0; i < str.size(); ++i) {
    std::vector<double> values = GetValuesFromStringSplit<double>(str[i], " ");
    DH.row(i) = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        values.data(), values.size());
  }
  *DH_table = DH;
}


bool CheckInBoundingBox(Eigen::MatrixXd DH_table, Eigen::VectorXd q,
                        std::vector<Eigen::Vector3d> workspace_limits,
                        std::vector<char> joint_type,
                        std::string model_type) {
  Eigen::Matrix4d T;
  for (int i = 0; i < q.rows(); i++) {
    Eigen::VectorXd params = DH_table.row(i);
    if (joint_type[i] == 'p') {
      params(0) += q(i);
    }
    if (joint_type[i] == 'r') {
      params(1) += q(i);
    }
    Eigen::MatrixXd Ti = robot::GenerateDHMatrix(params);
    T *= Ti;
    Eigen::Vector3d Pi = T.col(3).head(3);
    for (int j = 0; j < 3; j++) {
      if (Pi(j) >= workspace_limits[1](j) || Pi(j) <= workspace_limits[0](j)) {
        std::cout << "Warning! Joint " << i << " is out of workspace!"
                  << std::endl;
        return false;
      }
    }
  }
  Eigen::Matrix4d pose = ForwardKinematics(DH_table, q, joint_type, model_type);
  Eigen::Vector3d P = pose.col(3).head(3);
  for (int j = 0; j < 3; j++) {
    if (P(j) >= workspace_limits[1](j) || P(j) <= workspace_limits[0](j)) {
      std::cout << "Warning! End effector is out of workspace!" << std::endl;
      return false;
    }
  }
  return true;
}

}  // namespace robot
