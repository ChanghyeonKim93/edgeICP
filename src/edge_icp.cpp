#include "edge_icp.h"
#include "debugging.h"

// constructor overloadings
VOEdgeICP::VOEdgeICP(Parameters params):params(params){
  // synchronize the rgb images and depth images.
	RGBDIMAGE::dataSyncronize(params.dataset.dataset_name, this->rgb_name_vec, this->depth_name_vec, this->t_cam_vec);
};
// deconstructor
VOEdgeICP::~VOEdgeICP(){
	delete this->key_tree_2;
	delete this->key_tree_4;
	std::cout<<" !!!!!VOEdgeICP desctruct"<<std::endl;
};

void VOEdgeICP::setImages(const cv::Mat& img_i, const cv::Mat& depth_i){
  this->cur_img.release();
  this->cur_depth.release();
  img_i.copyTo(this->cur_img);
  depth_i.copyTo(this->cur_depth);
};

void VOEdgeICP::setKeyImages(){
	std::vector<cv::Mat> null_vec1, null_vec2;

  cur_img.copyTo(key_img);
  cur_depth.copyTo(key_depth);

  cur_img.release();
  cur_depth.release();
	null_vec1.swap(cur_img_vec);
	null_vec2.swap(cur_depth_vec);
};

// run function

void VOEdgeICP::run(){
  // Parameter settings
  cv::namedWindow("debug",CV_WINDOW_AUTOSIZE);
  int init_num = 1, final_num=16;
  int ind = init_num-1;

	int N_sample = 500;
	int max_num_of_icp_iter = 30;
	double dist_thres = 15.0; // pixels
	double trans_thres = 0.02; // 2cm
	double rot_thres = 3; // 3 degree
	int iter_shift_search = 7; //after 7 iterations, shift to the 2d NN searching.heuristic.

	std::vector<Point_2d> temp2d_vec,null_vec;
	std::vector<Point_4d> temp4d_vec;

	// Read dataset
	while(ind < final_num){
		cv::Mat img_temp,depth_temp;
		RGBDIMAGE::getImage(this->rgb_name_vec[ind], this->depth_name_vec[ind], this->params.depth.scale, img_temp, depth_temp);
		this->cur_img_vec.push_back(img_temp);
		this->cur_depth_vec.push_back(depth_temp);
		img_temp.release();
		depth_temp.release();
		++ind;
	}
	std::cout<<" Image loading done"<<std::endl<<std::endl;
	cv::waitKey(3*1000);

	// Algorithm part
	ind = init_num-1;
  while(ind<final_num){
		cv::Mat edge_map, dx, dy, d_norm;

		toc(); // timing start

		// Keyframe initialize
		if(ind==init_num-1){			// for first keyframe initialization.
			std::cout<<"keyframe initialize"<<std::endl;
			RGBDIMAGE::downSampleImage(this->cur_img_vec[ind],   this->key_img);
			RGBDIMAGE::downSampleDepth(this->cur_depth_vec[ind], this->key_depth); // downsample depth
			RGBDIMAGE::calcDerivX(this->key_img, dx); // gradient map
			RGBDIMAGE::calcDerivY(this->key_img, dy);
			RGBDIMAGE::calcDerivNorm(dx,dy,d_norm,dx,dy);

			cv::Canny(this->key_img,edge_map,170,220);
			RGBDIMAGE::findValidMask(edge_map, this->key_depth, this->key_valid_mask,this->key_valid_num_px); // pixels used as edge pixels.
			RGBDIMAGE::setEdgePoints(this->key_valid_mask, dx, dy, this-> key_valid_num_px, this->key_pt_u, this->key_pt_v, this->key_grad_u, this->key_grad_v); // made the pts sets.
      // Insert points 2d
			for(int k=0; k<this->key_pt_u.size(); k++){
				Point_2d temp2d;
				temp2d.push_back(this->key_pt_u[k]/320.0);
				temp2d.push_back(this->key_pt_v[k]/320.0);
				temp2d_vec.push_back(temp2d);
			}
			key_tree_2 = new KDTree( temp2d_vec, dist_thres*dist_thres/320.0/320.0);
			temp2d_vec.swap(null_vec);
      // Insert points 4d
			for(int k=0; k<this->key_pt_u.size(); k++){
				Point_4d temp4d;
				temp4d.push_back(this->key_pt_u[k]/320.0);
				temp4d.push_back(this->key_pt_v[k]/320.0);
				temp4d.push_back(this->key_grad_u[k]);
				temp4d.push_back(this->key_grad_v[k]);
				temp4d_vec.push_back(temp4d);
			}
		  key_tree_4 = new KDTree( temp4d_vec, dist_thres*dist_thres/320.0/320.0 );
			temp4d_vec.swap(null_vec);
			ind++;
		}

		// get current image
		RGBDIMAGE::downSampleImage(this->cur_img_vec[ind],   this->cur_img);
		RGBDIMAGE::downSampleDepth(this->cur_depth_vec[ind], this->cur_depth); // downsample depth

		// gradient map calculations
		RGBDIMAGE::calcDerivX(this->cur_img, dx); // gradient map
		RGBDIMAGE::calcDerivY(this->cur_img, dy);
		RGBDIMAGE::calcDerivNorm(dx, dy, d_norm, dx, dy);

		// find edge region ( full number w/o sampling )
		cv::Canny(this->cur_img, edge_map,170,220);
		RGBDIMAGE::findValidMask(edge_map, this->cur_depth, this->cur_valid_mask,this->cur_valid_num_px); // pixels used as edge pixels.
		RGBDIMAGE::setEdgePoints(this->cur_valid_mask, dx, dy, this-> cur_valid_num_px, this->cur_pt_u, this->cur_pt_v, this->cur_grad_u, this->cur_grad_v); // made the pts sets.
		std::cout<<" # of curr pts before sampling : " <<cur_pt_u.size()<<std::endl;

		// sub-sampling - random sampling for N_sample.
		std::vector<int> sample_ind;
		int npoints = this->cur_pt_u.size();
		RGBDIMAGE::randsample(npoints,N_sample,sample_ind);

		// initialize the sub-sampled pixels
		Point_2d temp2d(2,0);
		Point_4d temp4d(4,0);
		this->cur_edge_px_sub.resize(N_sample,temp2d);
		this->cur_edge_px_4d_sub.resize(N_sample,temp4d);
	  this->ref_ind.resize(N_sample,-1);

		for(int i=0; i<N_sample;i++){
			int ind = sample_ind[i];
			this->cur_edge_px_sub[i][0] = this->cur_pt_u[ind]/320.0;
			this->cur_edge_px_sub[i][1] = this->cur_pt_v[ind]/320.0;

			this->cur_edge_px_4d_sub[i][0] = this->cur_pt_u[ind]/320.0;
			this->cur_edge_px_4d_sub[i][1] = this->cur_pt_v[ind]/320.0;
			this->cur_edge_px_4d_sub[i][2] = this->cur_grad_u[ind];
			this->cur_edge_px_4d_sub[i][3] = this->cur_grad_v[ind];
		}

		// iterative part
		int icp_iter = 1;
		while(icp_iter <=max_num_of_icp_iter) {
			// warp_points

			// find robust edge match
			if(icp_iter > iter_shift_search)  key_tree_2->kdtree_nearest_neighbor(this->cur_edge_px_sub,    this->ref_ind);
			else 														  key_tree_4->kdtree_nearest_neighbor(this->cur_edge_px_4d_sub, this->ref_ind);

			//for(int i=0;i<ref_ind.size();i++) std::cout<<ref_ind[i]<<std::endl;
			// calc_residual
			RGBDIMAGE::calcResidual(this->key_edge_px_4d,this->cur_edge_px_4d_sub, this->ref_ind, this->res_x, this->res_y, this->residual);

			// calc_jacobian
      //RGBDIMAGE::calcJacobian();

			// calc delta xi


			// augment delta xi

			// /std::cout<<"iter : "<<icp_iter<<std::endl;

			icp_iter++;
		}

		// change the keyframes


//------------------------------------------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------- end of program ----------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------------------------------------------//
// debug part
		debug_img = this->cur_img_vec[ind].clone();
		char debug_str[100];
		sprintf(debug_str,"The number of image : %d",ind);
		double font_scale=0.7;
		cv::Scalar font_color = CV_RGB(0,0,0);
		int font_thickness = 2.8;
		cv::putText(debug_img,debug_str,cv::Point2f(5,240-5),cv::FONT_HERSHEY_TRIPLEX,font_scale,font_color,font_thickness);
		cv::imshow("debug",debug_img);
		// end of while loop
    ++ind;
		this->t_now=toc();
		this->t_save.push_back(this->t_now/1000.0);
		std::cout<<" Elapsed time : "<<this->t_save.back() <<" [ms], # of image : "<<ind<<std::endl;
		// exit the program
    /*if((char)cv::waitKey(0)=='q'){
      std::cout<<std::endl<<std::endl;
      std::cout<<" System : Program is halted by command."<<std::endl;
      break;
    }*/
  }
	double t_sum = 0.0;
	for(int i=0;i<t_save.size();i++) t_sum+=t_save[i];
	std::cout<<" ---- Average time : "<<t_sum/t_save.size()<<" [ms]"<<std::endl;
};
