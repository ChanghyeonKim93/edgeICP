#include "edge_icp.h"
#include "debugging.h"

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
	std::vector<cv::Mat> null_vec1, null_vec2;

  curr_img.copyTo(key_img);
  curr_depth.copyTo(key_depth);

  curr_img.release();
  curr_depth.release();
	null_vec1.swap(curr_img_vec);
	null_vec2.swap(curr_depth_vec);
};

void VOEdgeICP::run(){
  // iterative part
  cv::namedWindow("debug_img",CV_WINDOW_AUTOSIZE);
  int init_num = 1, final_num=16
	;
  int ind = init_num-1;
	// read all dataset
	while(ind < final_num){
		cv::Mat img_temp,depth_temp;
		getImage(this->rgb_name_vec[ind], this->depth_name_vec[ind], this->params.depth.scale, img_temp, depth_temp);
		this->curr_img_vec.push_back(img_temp);
		this->curr_depth_vec.push_back(depth_temp);
		img_temp.release();
		depth_temp.release();
		++ind;
	}
	std::cout<<" Image loading done"<<std::endl<<std::endl;
	cv::waitKey(2000);

	// algorithm part
	ind = init_num-1;
  while(ind<final_num){
		// dbg::getImageType(cv::Mat&);
		toc();
		cv::Mat edge_map,dx,dy,d_norm;

		downSampleImage(this->curr_img_vec[ind], this->curr_img); // downsample image
		downSampleDepth(this->curr_depth_vec[ind], this->curr_depth); // downsample depth

		calcDerivX(this->curr_img, dx); // gradient map
		calcDerivY(this->curr_img, dy);
		calcDerivNorm(dx,dy,d_norm,dx,dy);
		cv::Canny(this->curr_img,edge_map,170,220); // heuristic, Canny accepts CV_8U only.

		findValidMask(edge_map, this->curr_depth, this->curr_valid_mask); // pixels used as edge pixels.

		



		// end of while loop
    ++ind;
		this->t_now=toc();
		this->t_save.push_back(this->t_now/1000000.0);
		std::cout<<" Elapsed time : "<<this->t_save.back() <<" , # of image : "<<ind<<std::endl;
		// exit the program
    if((char)cv::waitKey(0)=='q'){
      std::cout<<std::endl<<std::endl;
      std::cout<<" System : Program is halted by command."<<std::endl;
      break;
    }
  }
};
