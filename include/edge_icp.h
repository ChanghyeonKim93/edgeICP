#ifndef _EDGE_ICP_H_
#define _EDGE_ICP_H_
#include "common.h"
#include "rgbd_image.h"
#include "timer.h"
#include "KDTree.h"


typedef std::vector<double> Point_2d;
typedef std::vector<double> Point_3d;
typedef std::vector<double> Point_4d;

class VOEdgeICP{

public:
  struct Calibration{
    double width,height;
    Eigen::Matrix3d K,invK;
    double fx,fy,cx,cy;
    Calibration(){
      width = 640;
      height = 4800;
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
      min = 5.0; // 400cm
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
  // motion
  Eigen::MatrixXd xi_temp;
  Eigen::MatrixXd delta_xi;

  // image data sync
  std::vector<std::string> rgb_name_vec, depth_name_vec;
	std::vector<double> t_cam_vec;

  // current and key images
  cv::Mat cur_img, cur_depth; // curent image Mat
  cv::Mat key_img, key_depth; // key image Mat
  cv::Mat cur_valid_mask, key_valid_mask; // cur & key image valid mask
  int cur_valid_num_px, key_valid_num_px;
  std::vector<cv::Mat> cur_img_vec, cur_depth_vec; //

  // time related
  unsigned long t_now;
  std::vector<double>   t_save;

  // pixel containers
  std::vector<Point_2d> cur_edge_px,        key_edge_px; // for release the vector, vector.resize(0);
  std::vector<Point_4d> cur_edge_px_4d,     key_edge_px_4d,        warped_edge_px_4d;

  std::vector<Point_2d> cur_edge_px_sub,    warped_edge_px_sub;

  std::vector<Point_2d> cur_edge_px_sub_n;
  std::vector<Point_4d> cur_edge_px_4d_sub_n;
  std::vector<Point_2d> key_edge_px_n;
  std::vector<Point_4d> key_edge_px_4d_n;

  std::vector<Point_4d> cur_edge_px_4d_sub, warped_edge_px_4d_sub;

  std::vector<double>   cur_pt_u,           cur_pt_v,              key_pt_u,          key_pt_v;
  std::vector<double>   cur_pt_depth,       key_pt_depth,          warped_pt_depth;
  std::vector<double>   warp_pt_u,          warp_pt_v;
  std::vector<double>   cur_grad_u,         cur_grad_v,            key_grad_u,        key_grad_v;
  std::vector<int>      ref_ind;
  std::vector<double>   residual,           res_x,                 res_y;

  // KDTree
  KDTree* key_tree_2;
  KDTree* key_tree_4;

  // debug
  cv::Mat debug_img;

};


#endif
