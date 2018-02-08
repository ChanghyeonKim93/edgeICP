#include "LieGroup.h"

void LIE::se3Exp(const Eigen::MatrixXd& xi_temp, Eigen::Matrix4d& g){
  // initialize variables
  Eigen::Vector3d v, w;
  float length_w = 0.0;
  Eigen::Matrix3d Wx, R, V;
  Eigen::Vector3d t;

  v(0) = xi(0);
  v(1) = xi(1);
  v(2) = xi(2);
  w(0) = xi(3);
  w(1) = xi(4);
  w(2) = xi(5);

  length_w = std::sqrt(w.transpose() * w);
  hatOperator(w, Wx);
  if (length_w < 1e-7)
  {
      R = Eigen::Matrix3d::Identity(3,3) + Wx + 0.5 * Wx * Wx;
      V = Eigen::Matrix3d::Identity(3,3) + 0.5 * Wx + Wx * Wx / 3.0;
  }
  else
  {

      R = Eigen::Matrix3d::Identity(3,3) + (sin(length_w)/length_w) * Wx + ((1-cos(length_w))/(length_w*length_w)) * (Wx*Wx);
      V = Eigen::Matrix3d::Identity(3,3) + ((1-cos(length_w))/(length_w*length_w)) * Wx + ((length_w-sin(length_w))/(length_w*length_w*length_w)) * (Wx*Wx);
  }
  t = V * v;

  // assign rigid body transformation matrix (in SE(3))
  g = Eigen::MatrixXd::Identity(4,4);
  g(0,0) = R(0,0);
  g(0,1) = R(0,1);
  g(0,2) = R(0,2);

  g(1,0) = R(1,0);
  g(1,1) = R(1,1);
  g(1,2) = R(1,2);

  g(2,0) = R(2,0);
  g(2,1) = R(2,1);
  g(2,2) = R(2,2);

  g(0,3) = t(0);
  g(1,3) = t(1);
  g(2,3) = t(2);

      // for debug
      // std::cout << R << std::endl;
      // std::cout << t << std::endl;
      //usleep(10000000);
}

void LIE::hat_operate(const Eigen::Vector3d& col_vec, Eigen::Matrix3d& skew_mat){
   skew_mat(0,0) = 0;
   skew_mat(0,1) = -col_vec(2);
   skew_mat(0,2) = col_vec(1);

   skew_mat(1,0) = col_vec(2);
   skew_mat(1,1) = 0;
   skew_mat(1,2) = -col_vec(0);

   skew_mat(2,0) = -col_vec(1);
   skew_mat(2,1) = col_vec(0);
   skew_mat(2,2) = 0;
}
