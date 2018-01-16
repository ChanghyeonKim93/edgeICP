#include <iostream>
#include "common.h"
#include "timer.h"
#include "edge_icp.h"
#include "rgbd_image.h"

int main(void){
	unsigned long ti,tf;

	//=========================================
	std::cout<<std::endl<<std::endl<<std::endl;
	//=========================================
	VOEdgeICP::Parameters params;
	params.dataset.dataset_kind = "TUM";
	params.dataset.dataset_name = "rgbd_dataset_freiburg3_long_office_household";

	VOEdgeICP *vo = new VOEdgeICP(params);

	// algorithm running.
	vo->run();

	delete vo;
	std::cout<<" Memory is released."<<std::endl;
	//=========================================
	std::cout<<std::endl<<std::endl<<std::endl;
	//=========================================
	return 0;
}
