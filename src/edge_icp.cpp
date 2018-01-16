#include "edge_icp.h"

// constructor overloadings
VOEdgeICP::VOEdgeICP(Parameters params):params(params){
  // synchronize the rgb images and depth images.
	dataSyncronize(params.dataset.dataset_name, this->rgb_name_vec, this->depth_name_vec, this->t_cam_vec);
};
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

void VOEdgeICP::run(){
  // iterative part
  cv::namedWindow("debug_img",CV_WINDOW_AUTOSIZE);
  int init_num = 1, final_num=16;
  int ind = init_num-1;
  while(ind<=final_num){
    toc();
    getImage(this->rgb_name_vec[ind], this->depth_name_vec[ind], this->params.depth.scale, this->curr_img, this->curr_depth);

    ind++;
    this->t_now=toc();
    std::cout<<"Elapsed time : "<<this->t_now/1000000.0<<" , # of image : "<<ind<<std::endl;
    // exit the program
    if((char)cv::waitKey(0)=='q'){
      std::cout<<std::endl<<std::endl;
      std::cout<<" System : Program is halted by command."<<std::endl;
      break;
    }
  }

};
