#include <iostream>
#include "common.h"
#include "timer.h"
#include "edge_icp.h"
#include "rgbd_image.h"
#include "KDTree.h"


int main(void){
//=========================================
	std::cout<<std::endl<<std::endl<<std::endl;
//=========================================
	VOEdgeICP::Parameters params;
	params.dataset.dataset_kind = "TUM";
	params.dataset.dataset_name = "rgbd_dataset_freiburg3_long_office_household";

	VOEdgeICP *vo = new VOEdgeICP(params);

	// algorithm running .
	//Eigen::VectorXd a;
	//std::cout<<a.size()<<std::endl;
	//a.resize(6);
	//a(0)=1;
	//std::cout<<a<<std::endl;

	vo->run();

	// release .
	delete vo;
	std::cout<<" Memory is released."<<std::endl;

//=========================================
	std::cout<<"\n\n\n\n\n";
//=========================================
	return 0;
}
