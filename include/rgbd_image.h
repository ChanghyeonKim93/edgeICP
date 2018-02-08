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
  void setEdgePoints(cv::Mat&,cv::Mat&,cv::Mat&, int&, std::vector<double>&, std::vector<double>&,std::vector<double>&, std::vector<double>&);
  void calcResidual(const std::vector<Point_4d>& key_edge_px_4d, const std::vector<Point_4d>& cur_edge_px_4d_sub, const std::vector<int>& ref_ind, std::vector<double>& res_x, std::vector<double>& res_y, std::vector<double>& residual);
  void update_t_distribution(const std::vector<double>& residual, double& sigma);
  void randsample(const int& npoints, const int& N_sample, std::vector<int>& sub_idx);
  void calcJacobian(const std::vector<Point_4d>& cur_edge_px_4d_sub, const std::vector<Point_4d>& key_edge_px_4d, const std::vector<int> ref_ind, const std::vector<double>& residual, Eigen::MatrixXd& J);

  // not use
  void dummyFunc();
};


/*double interp2(const cv::Mat& img_i, const double& u, const double& v);
//void undistort(const cv::Mat& img_i, cv::Mat& img_0);
*/
#endif
