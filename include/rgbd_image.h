#ifndef _RGBD_IMAGE_H_
#define _RGBD_IMAGE_H_
#include "common.h"

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
  void dummyFunc();
  // not use
};


/*double interp2(const cv::Mat& img_i, const double& u, const double& v);
//void undistort(const cv::Mat& img_i, cv::Mat& img_0);
*/
#endif
