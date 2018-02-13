#ifndef _RGBD_IMAGE_H_
#define _RGBD_IMAGE_H_
#include "common.h"
#include "LieGroup.h"

typedef std::vector<double> Point_2d;
typedef std::vector<double> Point_3d;
typedef std::vector<double> Point_4d;

namespace RGBDIMAGE{
  void downSampleImage(cv::Mat& img_i, cv::Mat& img_o);
  void downSampleDepth(cv::Mat& img_i, cv::Mat& img_o);
  void getAssociationFile(const std::string&, std::vector<std::string>&,std::vector<std::string>&,std::vector<std::string>&);
  void dataSyncronize(const std::string&, std::vector<std::string>&, std::vector<std::string>&, std::vector<double>&);
  void getImage(const std::string&, const std::string&, const double&, cv::Mat&, cv::Mat&);
  void findCannyPixels(cv::Mat&);
  void calcDerivX(cv::Mat&, cv::Mat&);
  void calcDerivY(cv::Mat&, cv::Mat&);
  void calcDerivNorm(cv::Mat&, cv::Mat&, cv::Mat&);
  void calcDerivNorm(cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&);
  void findValidMask(cv::Mat&, cv::Mat&, cv::Mat&, int&);
  void setEdgePoints(cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&, std::vector<double>&, std::vector<double>&, std::vector<double>&, std::vector<double>&, std::vector<double>&);
  void calcResidual(const std::vector<Point_4d>& key_edge_px_4d, const std::vector<Point_4d>& cur_edge_px_4d_sub, const std::vector<int>& ref_ind, std::vector<double>& res_x, std::vector<double>& res_y, std::vector<double>& residual);
  void update_t_distribution(const std::vector<double>& residual, double& sigma);
  void randsample(const int& npoints, const int& N_sample, std::vector<int>& sub_idx);
  void calcJacobian(const std::vector<Point_2d>& warped_edge_px_sub, const std::vector<double>& warped_pt_depth, const std::vector<Point_4d>& key_edge_px_4d, const std::vector<int> ref_ind, const std::vector<double>& residual, const Eigen::Matrix3d& K, Eigen::MatrixXd& J);
  void warpPoints(const std::vector<Point_4d>& cur_edge_px_4d_sub, const std::vector<double>& cur_pt_depth, const Eigen::Matrix3d& K, const Eigen::MatrixXd& xi_temp, std::vector<Point_2d>& warped_edge_px_sub, std::vector<Point_4d>& warped_edge_px_4d_sub , std::vector<double>& warped_pt_depth);
  void se3Exp(const Eigen::MatrixXd& xi_temp, Eigen::Matrix4d& g);
  void hatOperator(const Eigen::Vector3d& col_vec, Eigen::Matrix3d& skew_mat);

  // not use
  void dummyFunc();
};


/*double interp2(const cv::Mat& img_i, const double& u, const double& v);
//void undistort(const cv::Mat& img_i, cv::Mat& img_0);
*/
#endif
