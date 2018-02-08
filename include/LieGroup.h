#ifndef _LIEGROUP_H_
#define _LIEGROUP_H_
#include "common.h"
#include <eigen3/Eigen/Dense>

namespace LIE{
  void se3Exp(const Eigen::MatrixXd& xi_temp, Eigen::Matrix4d& g);
  void hat_operate(const Eigen::Vector3d& col_vec, Eigen::Matrix3d& skew_mat);
};

#endif
