#include <iostream>
#include "common.h"
#include "timer.h"
#include "edge_icp.h"
#include "rgbd_image.h"

int main(void){
	std::cout<<std::endl<<std::endl<<std::endl;
	unsigned long ti,tf;
	VOEdgeICP::Parameters params;
	VOEdgeICP *vo = new VOEdgeICP(params);

	std::string dataset_name = "rgbd_dataset_freiburg3_long_office_household";
	std::vector<std::string> rgb_assoc_vec, depth_assoc_vec, truth_assoc_vec;
	std::vector<std::string> rgb_name_vec, depth_name_vec;
	std::vector<double> t_cam_vec;

	dataSyncronize(dataset_name, rgb_name_vec, depth_name_vec, t_cam_vec);
//
	cv::namedWindow("color",CV_WINDOW_AUTOSIZE );
	cv::namedWindow("depth",CV_WINDOW_AUTOSIZE );

	cv::Mat img_raw, img_gray, depth_raw, depth_metric;
	cv::Mat img_gray_re;

	for(int k=0;k<80;k++) {
		getImage(rgb_name_vec[k],depth_name_vec[k],img_raw,depth_raw);
		cv::cvtColor(img_raw,img_gray,CV_RGB2GRAY);
		img_gray.convertTo(img_gray,CV_64F);
		depth_raw.convertTo(depth_metric,CV_64F);
		depth_metric*=vo->params.depth.scale;

		vo->setImages(img_gray,depth_metric);

		toc();
			downSampleImage(img_gray,img_gray_re);
		tf=toc();
		std::cout<<"1 : "<<tf<<std::endl;

		toc();
			downSampleImage2(img_gray,img_gray_re);

		tf=toc();
		std::cout<<"2 : "<<tf<<std::endl;

		//std::cout<<img_gray_re<<std::endl;

		cv::imshow("color", img_gray);
		cv::imshow("depth", depth_raw);


		cv::waitKey(0);
	}

	std::cout<<std::endl<<std::endl<<std::endl;
	return 0;
}
