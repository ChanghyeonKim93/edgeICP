#include "kdtree_my.h"

KdtreeMy::KdtreeMy(int dim){
  this->kd=NULL; // allocate the NULL address
  this->kd_res=NULL; // allocate the NULL address
  if(dim<=0) assert(1);
  std::cout<<" for debug - kdtree made ! size : "<<dim<<std::endl;
  //std::cout<<this->kd <<std::endl; // for debug
  //this->kd = kd_create(dim);
}

KdtreeMy::~KdtreeMy(){
  if(this->kd != NULL){
    //std::cout<<"allocated"<<std::endl;
    kd_free(this->kd);
  }
  if(this->kd_res != NULL){
    //std::cout<<"allocated2"<<std::endl;
    kd_res_free(this->kd_res);
  }
  std::cout<<"   !!!!! KdtreeMy destruct"<<std::endl;
}

void KdtreeMy::initialize(){
  this->kd = kd_create(this->dim);
}

void KdtreeMy::insertSinglePoint(double* inputArr){ // row major needed.
  kd_insert(this->kd,inputArr,0); // third parameter : data term but, I do not use it.
  //assert(kd_insert(this->kd,inputArr[k],0) == 0);
}
void KdtreeMy::findNearest(double* queryArr){
  this->kd_res=kd_nearest(this->kd,queryArr);
}
void KdtreeMy::printResSize(){
  std::cout<<" kd res size : "<<kd_res_size(this->kd_res)<<std::endl;
}

void KdtreeMy::printNearestPoint(){
  std::cout<<"res x : "<<this->kd_res->riter->item->pos[0]<<", res y : "<<this->kd_res->riter->item->pos[1]<<std::endl;
}

/*
KdtreeMy::kdtreeInsertMultiplePoints(double& inputArr){
  for(int k = 0;k<size(inputArr);k++){
    assert(kd_insert(this->kd,inputArr[k],0) == 0);
  }
}
*/
