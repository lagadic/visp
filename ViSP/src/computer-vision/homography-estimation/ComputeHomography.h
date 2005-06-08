
#ifndef COMPUTEHOMOGRAPHY_H
#define COMPUTEHOMOGRAPHY_H
#include <include/constant.h>

#include <robot/simulation/CServoSimu.h>
#include <basicfeatures/CPoint.h>
#include <basicfeatures/C2D1.5.h>
#include <math/simple-math.h>
#include <device/CDisplayX.h>
#include <data_structure/CList.h>
#include <robust/CRobustEstimator.h>
#include <tools/CPlane.h>

double ComputeHomography(int nbpoint,
		  CPoint *c1P,
		  CPoint *c2P,
		  CMatrixHomogeneous &c2Mc1,
		  int display,
		  CImage<unsigned char> &I,
		  int robust, int save =0) ;

double
ComputeHomographyPlane(int nbpoint,
		       CPoint *c1P,
		       CPoint *c2P,
		       CMatrixHomogeneous &c2Mc1,
		       CMatrixHomogeneous &c2Mo,
		       CPlane &No,
		       int display,
		       CImage<unsigned char> &I,
		       int userobust, int save
		       ) ;

double
ComputePoseHomographyPlane(int nbpoint,
		       CPoint *c1P,
		       CPoint *c2P,
		       CMatrixHomogeneous &c2Mc1,
		       CMatrixHomogeneous &c2Mo,
		       CPlane &No,
		       CPoint *oP,
		       int display,
		       CImage<unsigned char> &I,
		       int userobust, int save
		       ) ;

#endif
