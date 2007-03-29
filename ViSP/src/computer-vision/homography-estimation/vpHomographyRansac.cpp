
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpHomographyRansac.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 * From:      vpHomographyRansac.cpp, ViSP 2
 *
 * Version control
 * ===============
 *
 *  $Id: vpHomographyRansac.cpp,v 1.4 2007-03-29 13:49:02 hatran Exp $
 * optimized by Tran to improve speed.
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#include <visp/vpColVector.h>
#include <visp/vpHomography.h>
#include <visp/vpRansac.h>


#define vpEps 1e-6

/*!
  \file vpHomographyRansac.cpp
  \brief function used to estimate an homography using the Ransac algorithm
*/


bool
iscolinear(double *x1, double *x2, double *x3)
{
  vpColVector p1(3), p2(3), p3(3);
  p1 << x1 ;
  p2 << x2 ;
  p3 << x3 ;
  //vpColVector v;
  //vpColVector::cross(p2-p1, p3-p1, v);
  //return (v.sumSquare() < vpEps);
  // Assume inhomogeneous coords, or homogeneous coords with equal
  // scale.
  return ((vpColVector::cross(p2-p1, p3-p1).sumSquare()) < vpEps);
}


bool
vpHomography::degenerateConfiguration(vpColVector &x,int *ind, double threshold_area)
{

  for (int i=1 ; i < 4 ; i++)
    for (int j=0 ; j<i ; j++)
      if (ind[i]==ind[j]) return true ;

  int n = x.getRows()/4 ;
  double pa[4][3] ;
  double pb[4][3] ;



  for(int i = 0 ; i < 4 ; i++)
  {
    pb[i][0] = x[2*ind[i]] ;
    pb[i][1] = x[2*ind[i]+1] ;
    pb[i][2] = 1;

    pa[i][0] = x[2*n+2*ind[i]] ;
    pa[i][1] = x[2*n+2*ind[i]+1] ;
    pa[i][2] = 1;
  }

  double area2 = (-pa[1][0]*pa[0][1] + pa[2][0]*pa[0][1] +
		  pa[0][0]*pa[1][1] - pa[2][0]*pa[1][1] +
		  -pa[0][0]*pa[2][1] + pa[1][0]*pa[2][1]);

  int i = 0, j = 1, k = 2;

  double area012 = (-pa[j][0]*pa[i][1] + pa[k][0]*pa[i][1] +
		    pa[i][0]*pa[j][1] - pa[k][0]*pa[j][1] +
		    -pa[i][0]*pa[k][1] + pa[1][j]*pa[k][1]);

  i = 0; j = 1, k = 3;
  double area013 = (-pa[j][0]*pa[i][1] + pa[k][0]*pa[i][1] +
		    pa[i][0]*pa[j][1] - pa[k][0]*pa[j][1] +
		    -pa[i][0]*pa[k][1] + pa[1][j]*pa[k][1]);

  i = 0; j = 2, k = 3;
  double area023 = (-pa[j][0]*pa[i][1] + pa[k][0]*pa[i][1] +
		    pa[i][0]*pa[j][1] - pa[k][0]*pa[j][1] +
		    -pa[i][0]*pa[k][1] + pa[1][j]*pa[k][1]);

  i = 1; j = 2, k = 3;
  double area123 = (-pa[j][0]*pa[i][1] + pa[k][0]*pa[i][1] +
		    pa[i][0]*pa[j][1] - pa[k][0]*pa[j][1] +
		    -pa[i][0]*pa[k][1] + pa[1][j]*pa[k][1]);

  double sum_area = area012 + area013 + area023 + area123;

  return ((sum_area < threshold_area) ||
	  (iscolinear(pa[0],pa[1],pa[2]) ||
	   iscolinear(pa[0],pa[1],pa[3]) ||
	   iscolinear(pa[0],pa[2],pa[3]) ||
	   iscolinear(pa[1],pa[2],pa[3]) ||
	   iscolinear(pb[0],pb[1],pb[2]) ||
	   iscolinear(pb[0],pb[1],pb[3]) ||
	   iscolinear(pb[0],pb[2],pb[3]) ||
	   iscolinear(pb[1],pb[2],pb[3])));
}
    /*
\brief
Function to determine if a set of 4 pairs of matched  points give rise
to a degeneracy in the calculation of a homography as needed by RANSAC.
This involves testing whether any 3 of the 4 points in each set is
colinear.

point are coded this way
x1b,y1b, x2b, y2b, ... xnb, ynb
x1a,y1a, x2a, y2a, ... xna, yna
leading to 2*2*n
*/
bool
vpHomography::degenerateConfiguration(vpColVector &x, int *ind)
{
  for (int i = 1; i < 4 ; i++)
    for (int j = 0 ;j < i ; j++)
      if (ind[i] == ind[j]) return true ;

  int n = x.getRows()/4;
  double pa[4][3];
  double pb[4][3];
  int n2 = 2 * n;
  int ind2;
  for(int i = 0; i < 4 ;i++)
    {
      ind2 = 2 * ind[i];
      pb[i][0] = x[ind2];
      pb[i][1] = x[ind2+1];
      pb[i][2] = 1;

      pa[i][0] = x[n2+ind2] ;
      pa[i][1] = x[n2+ind2+1] ;
      pa[i][2] = 1;
    }
  return ( iscolinear(pa[0],pa[1],pa[2]) ||
	   iscolinear(pa[0],pa[1],pa[3]) ||
	   iscolinear(pa[0],pa[2],pa[3]) ||
	   iscolinear(pa[1],pa[2],pa[3]) ||
	   iscolinear(pb[0],pb[1],pb[2]) ||
	   iscolinear(pb[0],pb[1],pb[3]) ||
	   iscolinear(pb[0],pb[2],pb[3]) ||
	   iscolinear(pb[1],pb[2],pb[3]));
}
// Fit model to this random selection of data points.
void
vpHomography::computeTransformation(vpColVector &x,int *ind, vpColVector &M)
{
  int i ;
  int n = x.getRows()/4 ;
  double xa[4], xb[4];
  double ya[4], yb[4];
  int n2 = n * 2;
  int ind2;
  for(i=0 ; i < 4 ; i++)
    {
      ind2 = 2 * ind[i];
      xb[i] = x[ind2] ;
      yb[i] = x[ind2+1] ;

      xa[i] = x[n2+ind2] ;
      ya[i] = x[n2+ind2+1] ;
    }

  vpHomography aHb ;
  try {
    vpHomography::HLM(4,xb, yb, xa, ya, true, aHb);
    //vpHomography::HLM(8, xb, yb, xa, ya, false, aHb); //modified 13/09
  }
  catch(...)
    {
      aHb.setIdentity();
    }

  M.resize(9);
  for (i=0 ; i <9 ; i++)
    {
      M[i] = aHb.data[i] ;
    }
  aHb /= aHb[2][2] ;
}


// Evaluate distances between points and model.
double
vpHomography::computeResidual(vpColVector &x, vpColVector &M, vpColVector &d)
{
  int i ;
  int n = x.getRows()/4 ;
  int n2 = n  * 2;
  int i2;
  vpColVector *pa;
  vpColVector *pb;

  pa = new vpColVector [n];
  pb = new vpColVector [n];

  for( i=0 ; i < n ; i++)
    {
      i2 = 2 * i;
      pb[i].resize(3) ;
      pb[i][0] = x[i2] ;
      pb[i][1] = x[i2+1] ;
      pb[i][2] = 1;

      pa[i].resize(3) ;
      pa[i][0] = x[n2+i2] ;
      pa[i][1] = x[n2+i2+1] ;
      pa[i][2] = 1;
    }

  vpMatrix aHb(3,3) ; aHb /= aHb[2][2] ;

  for (i=0 ; i <9 ; i++)
    {
      aHb.data[i] = M[i];
    }

  d.resize(n);

  vpColVector Hpb  ;
  for (i=0 ; i <n ; i++)
    {
      Hpb = aHb*pb[i] ;
      Hpb /= Hpb[2] ;
      d[i] = sqrt((pa[i] - Hpb ).sumSquare()) ;
    }

  delete [] pa;
  delete [] pb;

  return 0 ;
}


void
vpHomography::initRansac(int n,
			 double *xb, double *yb,
			 double *xa, double *ya,
			 vpColVector &x)
{
  x.resize(4*n) ;
  int n2 = n * 2;
  int i2;
  for (int i=0 ; i < n ; i++)
  {
    i2 = 2 * i;
    x[i2] = xb[i] ;
    x[i2+1] = yb[i] ;
    x[n2+i2] = xa[i] ;
    x[n2+i2+1] = ya[i] ;
  }
}

bool
vpHomography::ransac(int n,
		     double *xb, double *yb,
		     double *xa, double *ya ,
		     vpHomography &aHb,
		     int consensus,
		     double threshold
		     )
{
  vpColVector x ;
  vpHomography::initRansac(n, xb, yb, xa, ya, x) ;

  vpColVector M ;
  vpColVector inliers(n) ;


  bool ransacable = vpRansac<vpHomography>::ransac(n, x, 4, threshold, M, inliers,  consensus);
  //vpRansac<vpHomography>::ransacHomography(n, x, 4, threshold, M, inliers, consensus) ;
  if(ransacable)
  {
    for (int i = 0 ;i < 9 ;i++)
    {
      aHb.data[i] = M[i];
    }
    aHb /= aHb[2][2];
  }
  return ransacable;
}

/*!

  Computes homography matrix \f$ b^H_a \f$ such as \f$X_b = b^H_a X_a \f$ with
  \f$ X_a = (xa, ya)^t \f$ and \f$ X_b = (xb, yb)^t \f$.

  \param n : Number of points.
  \param xb, yb : Coordinates of the points in \f$ X_b \f$ vector.
  \param xa, ya : Coordinates of the points in \f$ X_a \f$ vector.

  \param bHa : Homography matrix computed from \f$ X_a \f$ and \f$ X_b \f$
  vectors.

  \param inliers :  n dimention vector indicating if a point is an inlier
  (value 1.0) or an outlier (value 0). Matches are stocked in inliers vector
  column.

  \param residual : Residual.

  \param consensus : Minimal number of points (less than n) fitting the model.

  \param threshold : Threshold for outlier removing.

  \param areaThreshold : Ensure that the area formed by every 3 points within
  the 4 points used to compute the homography is greater than this
  threshold. If the area is smaller, we are in a degenerate case.

  \return true if the homography could be computed from 4 non-degenerated
  points.

*/
bool vpHomography::ransac(int n,
			  double *xb, double *yb,
			  double *xa, double *ya ,
			  vpHomography &bHa,
			  vpColVector &inliers,
			  double& residual,
			  int consensus,
			  double threshold,
			  double areaThreshold)
{
  vpColVector x ;
  vpHomography::initRansac(n, xb, yb, xa, ya, x);

  vpColVector M ;

  bool ransacable= vpRansac<vpHomography>::ransac(n, x, 4,
						  threshold, M,
						  inliers, consensus,
						  areaThreshold);

  for (int i = 0 ;i < 9 ;i++)
  {
    bHa.data[i] = M[i];
  }

  bHa /= bHa[2][2];
  return ransacable;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
