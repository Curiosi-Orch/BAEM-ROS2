#ifndef DATA_TYPES_H_
#define DATE_TYPES_H_

#include <vector>
#include <Eigen/Core>

enum ControlMode {
  TELE = 0,
  JOG = 1,
  TASK = 2
};

enum PredictionMode {
  NONE = 0,
  TRANS = 1,
  TRANS_COMPU = 2,
  TRANS_COMPU_MECHA = 3,
  PASSIVE = 4
};

enum KeyValues {
  HUMAN = 10, // ENTER
  TRAJ_0 = '0', TRAJ_1 = '1',
  J0_PLUS = 'Q',J1_PLUS = 'W',J2_PLUS = 'E',J3_PLUS = 'R',J4_PLUS = 'T',J5_PLUS = 'Y',J6_PLUS = 'U',
  J0_MINUS = 'q',J1_MINUS = 'w',J2_MINUS = 'e',J3_MINUS = 'r',J4_MINUS = 't',J5_MINUS = 'y',J6_MINUS = 'u',
  X_PLUS = 'Z',Y_PLUS = 'X',Z_PLUS = 'C',ROLL_PLUS = 'V',PITCH_PLUS = 'B',YAW_PLUS = 'N',
  X_MINUS = 'z', Y_MINUS = 'x',Z_MINUS = 'c', ROLL_MINUS = 'v',PITCH_MINUS = 'b',YAW_MINUS = 'n',
  PRE_TCM = '*', PRE_TC = '+', PRE_T = '/', PRE_OFF = '-',
  PRE_PASSIVE = 32, // SPACE
};

struct Position {
  Position(int size=0) {
    x = std::vector<double>(size);
    y = std::vector<double>(size);
    z = std::vector<double>(size);
  }
  Position(int size, Eigen::Vector3d value) {
    x = std::vector<double>(size,value.x());
    y = std::vector<double>(size,value.y());
    z = std::vector<double>(size,value.z());
  }
  void update(double new_x, double new_y, double new_z) {
    x.erase(x.begin());
    y.erase(y.begin());
    z.erase(z.begin());
    x.push_back(new_x);
    y.push_back(new_y);
    z.push_back(new_z);
  }
  // [index_start, index_end)
  void sum(int index_start, int index_end, double* sum_x, double* sum_y, double* sum_z, int* length, bool padding = false) {
    double _sum_x,_sum_y,_sum_z = 0.0;
    int count = 0;
    for (int i = index_start; i < index_end; ++i) {
      if (i < 0) {
        if (padding) {
          _sum_x += x[0];
          _sum_y += y[0];
          _sum_z += z[0];
          count += 1;
        }
      } else if (i>=x.size()) {
        if (padding) {
          _sum_x += x[x.size()-1];
          _sum_y += y[y.size()-1];
          _sum_z += z[x.size()-1];
          count += 1;
        }
      } else {
        _sum_x += x[i];
        _sum_y += y[i];
        _sum_z += z[i];
        count += 1;
      }
    }
    if (index_start == index_end) {
      _sum_x = x[std::max(index_start,0)];
      _sum_y = y[std::max(index_start,0)];
      _sum_z = z[std::max(index_start,0)];
      count += 1;
    }
    *sum_x = _sum_x;
    *sum_y = _sum_y;
    *sum_z = _sum_z;
    // *length = std::min(static_cast<int>(x.size()),index_end) - std::max(index_start,0);
    *length = count;
  }

  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
};

#endif // DATA_TYPES_H_