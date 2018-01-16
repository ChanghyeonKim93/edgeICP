#include "edge_icp.h"

// constructor overloadings
VOEdgeICP::VOEdgeICP(Parameters params):params(params){};
// deconstructor
VOEdgeICP::~VOEdgeICP(){};
void VOEdgeICP::setImages(const cv::Mat& img_i, const cv::Mat& depth_i){
  this->curr_img.release();
  this->curr_depth.release();
  img_i.copyTo(this->curr_img);
  depth_i.copyTo(this->curr_depth);
};

void VOEdgeICP::setKeyImages(){
  curr_img.copyTo(key_img);
  curr_depth.copyTo(key_depth);

  curr_img.release();
  curr_depth.release();
};
