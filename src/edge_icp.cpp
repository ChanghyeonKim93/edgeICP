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
	std::vector<cv::Mat> null_vec1, null_vec2;
	null_vec1.swap(curr_img_vec);
	//curr_img_vec.resize(0);
	null_vec2.swap(curr_depth_vec);
	//curr_depth_vec.resize(0);
};

void VOEdgeICP::run(){
  // iterative part
  cv::namedWindow("debug_img",CV_WINDOW_AUTOSIZE);
  int init_num = 1, final_num=90;
  int ind = init_num-1;


	while(ind<final_num){
		getImage(this->rgb_name_vec[ind], this->depth_name_vec[ind], this->params.depth.scale, this->curr_img, this->curr_depth);
		this->curr_img_vec.push_back(this->curr_img);
		this->curr_depth_vec.push_back(this->curr_depth);
		++ind;
	}
	std::cout<<" Image loading done"<<std::endl<<std::endl;
	cv::waitKey(2000);

	ind = init_num-1;
  while(ind<=final_num){
		toc();

		cv::Mat contours;
		cv::Canny(this->curr_img_vec[ind],contours,150,250); // heuristic
	  findCannyPixels(contours);
		cv::imshow("debug_img",contours);
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
