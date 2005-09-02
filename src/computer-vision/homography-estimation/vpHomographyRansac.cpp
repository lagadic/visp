
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
 *  $Id: vpHomographyRansac.cpp,v 1.2 2005-09-02 14:35:16 fspindle Exp $
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#include <visp/vpColVector.h>
#include <visp/vpHomography.h>
#include <visp/vpRansac.h>


#define eps 1e-6

/*!
  \file vpHomographyRansac.cpp
  \brief function used to estimate an homography using the Ransac algorithm
*/


bool
iscolinear(double *x1, double *x2, double *x3)
{

  vpColVector p1(3), p2(3), p3(3) ;
    p1 << x1 ;
    p2 << x2 ;
    p3 << x3 ;

  // Assume inhomogeneous coords, or homogeneous coords with equal
  // scale.
  return ((vpColVector::cross(p2-p1, p3-p1).sumSquare()) < eps);
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
vpHomography::degenerateConfiguration(vpColVector &x,int *ind)
{

  for (int i=1 ; i < 4 ; i++)
    for (int j=0 ; j<i ; j++)
      if (ind[i]==ind[j]) return true ;

  int n = x.getRows()/4 ;
  double pa[4][3] ;
  double pb[4][3] ;

  for(int i=0 ; i < 4 ; i++)
  {
    pb[i][0] = x[2*ind[i]] ;
    pb[i][1] = x[2*ind[i]+1] ;
    pb[i][2] = 1;

    pa[i][0] = x[2*n+2*ind[i]] ;
    pa[i][1] = x[2*n+2*ind[i]+1] ;
    pa[i][2] = 1;
  }

  return ( iscolinear(pa[0],pa[1],pa[2]) ||
	   iscolinear(pa[0],pa[1],pa[3]) ||
	   iscolinear(pa[0],pa[2],pa[3]) ||
	   iscolinear(pa[1],pa[2],pa[3]) ||
	   iscolinear(pb[0],pb[1],pb[2]) ||
	   iscolinear(pb[0],pb[1],pb[3]) ||
	   iscolinear(pb[0],pb[2],pb[3]) ||
	   iscolinear(pb[1],pb[2],pb[3])  );
}
// Fit model to this random selection of data points.
void
vpHomography::computeTransformation(vpColVector &x,int *ind, vpColVector &M)
{
  int i ;
  int n = x.getRows()/4 ;
  double xa[4], xb[4] ;
  double ya[4], yb[4] ;

  for(i=0 ; i < 4 ; i++)
  {
    xb[i] = x[2*ind[i]] ;
    yb[i] = x[2*ind[i]+1] ;

    xa[i] = x[2*n+2*ind[i]] ;
    ya[i] = x[2*n+2*ind[i]+1] ;
  }

  vpHomography aHb ;
  try {
    vpHomography::HLM(4,xb, yb, xa,  ya, true,  aHb) ;
  }
  catch(...)
  {
    aHb.setIdentity() ;
  }

  M.resize(9) ;
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
  vpColVector *pa;
  vpColVector *pb;

  pa = new vpColVector [n];
  pb = new vpColVector [n];

  for( i=0 ; i < n ; i++)
  {
    pb[i].resize(3) ;
    pb[i][0] = x[2*i] ;
    pb[i][1] = x[2*i+1] ;
    pb[i][2] = 1;

    pa[i].resize(3) ;
    pa[i][0] = x[2*n+2*i] ;
    pa[i][1] = x[2*n+2*i+1] ;
    pa[i][2] = 1;
  }

  vpMatrix aHb(3,3) ; aHb /= aHb[2][2] ;

  for (i=0 ; i <9 ; i++)
  {
     aHb.data[i] = M[i];

  }

  d.resize(n) ;


  vpColVector Hpb  ;
  for (i=0 ; i <n ; i++)
  {

    Hpb = aHb*pb[i] ;
    Hpb /= Hpb[2] ;
    //  cout << pa[i].t() << (Hpb).t() <<endl ; ;

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
  for (int i=0 ; i < n ; i++)
  {
    x[2*i] = xb[i] ;
    x[2*i+1] = yb[i] ;
    x[2*n+2*i] = xa[i] ;
    x[2*n+2*i+1] = ya[i] ;
  }
}

void
vpHomography::ransac(int n,
		     double *xb, double *yb,
		     double *xa, double *ya ,
		     vpHomography &aHb)
{


  vpColVector x ;
  vpHomography::initRansac(n,xb,yb,xa,ya,x) ;

  vpColVector M ;
  vpColVector inliers(n) ;
  vpRansac<vpHomography>::ransac(n,x,4,1e-6,M,inliers) ;

  for (int i=0 ; i <9 ; i++)
  {
     aHb.data[i] = M[i];
  }

  aHb /= aHb[2][2] ;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
