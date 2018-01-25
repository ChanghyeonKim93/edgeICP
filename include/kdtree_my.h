#ifndef _KDTREE_MY_H_
#define _KDTREE_MY_H_
#include "common.h"
#include "kdtree.h" // neccessary

class KdtreeMy{
public:
KdtreeMy(int dim); // construct kdtree, input : dimension
~KdtreeMy(); // destruct kdtree
//kdtreeInsertSinglePoint(double&);
//kdtreeInsertMultiplePoints(double&);

private:
 kdtree* kd;
 kdres* kd_res;

};


#endif
