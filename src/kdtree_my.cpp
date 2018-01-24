#include "kdtree_my.h"

KdtreeMy::KdtreeMy(int dim){
  if(dim<=0) assert(1);
std::cout<<"kdtree made"<<std::endl;
  this->kd = kd_create(dim);
}

KdtreeMy::~KdtreeMy(){
if(this->kd != NULL)
  kd_free(this->kd);
if(this->kd_res != NULL)
  kd_res_free(this->kd_res);
}

/*KdtreeMy::kdtreeInsertSinglePoint(double& inputArr){
  //assert(kd_insert(this->kd,inputArr[k],0) == 0);
}

KdtreeMy::kdtreeInsertMultiplePoints(double& inputArr){
  for(int k = 0;k<size(inputArr);k++){
    assert(kd_insert(this->kd,inputArr[k],0) == 0);
  }
}
*/
