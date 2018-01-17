#ifndef _EDGE_ICP_H_
#define _EDGE_ICP_H_
#include "common.h"
#include "rgbd_image.h"
#include "timer.h"

class VOEdgeICP{

public:
  struct Calibration{
    double width,height;
    Eigen::Matrix3d K,invK;
    double fx,fy,cx,cy;
    Calibration(){
      width = 320;
      height = 240;
      K << 535.433105,0.0,320.106653,
      0.0, 539.212524, 247.632131,
      0.0, 0.0, 1.0;
      invK << 0.001867646, 0.0, -0.59784621,
      0.0, 0.001854556, -0.45924773,
      0.0, 0.0, 1.0;
      fx = 535.433105;
      fy = 539.212524;
      cx = 320.106653;
      cy = 247.632131;
    }
  };
  struct Depth{
    double scale, min, max;
    Depth(){//constructor
      scale = 1.0/5000.0;
      min = 0.5; // 50cm
      min = 4.0; // 400cm
    }
  };
  struct Dataset{
    std::string dataset_kind;
    std::string dataset_name;
  };
  // assorting the all above parameters.
  struct Parameters{
    VOEdgeICP::Calibration calib;
    VOEdgeICP::Depth depth;
    VOEdgeICP::Dataset dataset;
    // example - VOEdgeICP::Parameters params;
    // params.calib.width = 320;
    // params.depth.scale = 0.001;
  };


  Parameters params;

  VOEdgeICP(Parameters);
  ~VOEdgeICP();

  void setImages(const cv::Mat& img_i,const cv::Mat& depth_i);
  void setKeyImages();
  void run();


private:
  // image data sync
  std::vector<std::string> rgb_name_vec, depth_name_vec;
	std::vector<double> t_cam_vec;

  // current and key images
  cv::Mat curr_img;
  cv::Mat curr_depth;
  cv::Mat key_img;
  cv::Mat key_depth;
  std::vector<cv::Mat> curr_img_vec;
  std::vector<cv::Mat> curr_depth_vec;

  // time related
  unsigned long t_now;
  std::vector<double> t_save;
  // pixel containers
  std::vector<cv::Point2d> curr_edge_px;
  std::vector<cv::Point2d> key_edge_px; // for release the vector, vector.resize(0);


};


#endif
