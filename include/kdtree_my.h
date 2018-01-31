#ifndef _KDTREE_MY_H_
#define _KDTREE_MY_H_
#include "common.h"
#include "kdtree.h" // neccessary

class KdtreeMy{
public:
KdtreeMy(int dim); // construct kdtree, input : dimension
~KdtreeMy(); // destruct kdtree

// Member methods
void initialize();
void insertSinglePoint(double* );
void findNearest(double* );
void printResSize();
void printNearestPoint();
//kdtreeInsertMultiplePoints(double&);

private:
 kdtree* kd;
 kdres* kd_res;
 int dim;

};


#endif
