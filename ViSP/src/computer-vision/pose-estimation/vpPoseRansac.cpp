/****************************************************************************
 *
 * $Id: vpPoseRansac.cpp,v 1.4 2006-05-30 08:40:42 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Pose computation.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \file vpPoseRansac.cpp
  \brief function used to estimate a pose using the Ransac algorithm
*/

#include <visp/vpColVector.h>
#include <visp/vpPose.h>
#include <visp/vpRansac.h>
#include <visp/vpTime.h>
#include <visp/vpList.h>


#define eps 1e-6

/*
\brief

determines if there more than one the same point in the chosen set of point

point are coded this way
x1,y1, X1, Y1, Z1
xnm, ynm, Xnm, Ynm, Znm

leading to 5*n*m
*/
bool
vpPose::degenerateConfiguration(vpColVector &x,int *ind)
{

  //  TRACE("%p %p %d",&x, ind, x.getRows()) ;
  for (int i=1 ; i < 4 ; i++)
    for (int j=0 ; j<i ; j++)
    {
      int indi =  5*ind[i] ;
      int indj =  5*ind[j] ;

      if ((fabs(x[indi] - x[indj]) < 1e-6) &&
	  (fabs(x[indi+1] - x[indj+1]) < 1e-6))
      {	  return true ;	}
      if ((fabs(x[indi+2] - x[indj+2]) < 1e-6) &&
	  (fabs(x[indi+3] - x[indj+3]) < 1e-6) &&
	  (fabs(x[indi+4] - x[indj+4]) < 1e-6))	{ return true ;  }
    }

  return false ;
}
/*!
  Fit model to this random selection of data points.

  We chose the Dementhon algorithm to compute the pose
*/
void
vpPose::computeTransformation(vpColVector &x,int *ind, vpColVector &M)
{
  int i ;

  vpPoint p[4] ;

  vpPose pose ;
  pose.clearPoint() ;
  for(i=0 ; i < 4 ; i++)
  {

    int index = 5*ind[i] ;

    p[i].set_x(x[index]) ;
    p[i].set_y(x[index+1]) ;

    p[i].setWorldCoordinates(x[index+2],x[index+3], x[index+4]) ;
    pose.addPoint(p[i]) ;
  }


  //  pose.printPoint() ;
  vpHomogeneousMatrix cMo ;
  try {
    pose.computePose(vpPose::DEMENTHON, cMo) ;
    //    cout << cMo << endl ;
  }
  catch(...)
  {
    cMo.setIdentity() ;
  }

  M.resize(16) ;
  for (i=0 ; i <16 ; i++)
  {
    M[i] = cMo.data[i] ;
  }

}


/*!
  Evaluate distances between points and model.

  this function can certainly be optimized...
*/

double
vpPose::computeResidual(vpColVector &x, vpColVector &M, vpColVector &d)
{

  int i ;
  int n = x.getRows()/5 ;

  vpPoint *p;
  p = new vpPoint [n] ;
  {
    //    firsttime=1 ;
    for( i=0 ; i < n ; i++)
    {
      p[i].setWorldCoordinates(x[5*i+2],x[5*i+3], x[5*i+4]) ;
    }
  }

  vpHomogeneousMatrix cMo ;
  for (i=0 ; i <16 ; i++)
  {
    cMo.data[i] = M[i];
  }


  d.resize(n) ;
  vpColVector cP, xy ;

  for( i=0 ; i < n ; i++)
  {
    p[i].changeFrame(cMo,cP) ;
    p[i].projection(cP,xy) ;
    d[i] = sqrt(vpMath::sqr(x[5*i]-xy[0])+vpMath::sqr(x[5*i+1]-xy[1])) ;
  }

  delete [] p;

  return 0 ;
}


void
vpPose::initRansac(const int n,
		   const double *x, const double *y,
		   const int m,
		   const double *X, const double *Y, const double *Z,
		   vpColVector &data)
{
  data.resize(5*n*m) ;
  int k =0 ;
  for (int i=0 ; i < n ; i++)
  {
    for (int j=0 ; j < m ; j++)
    {
      data[k] = x[i] ;
      data[k+1] = y[i] ;
      data[k+2] = X[j] ;
      data[k+3] = Y[j] ;
      data[k+4] = Z[j] ;

      k+=5 ;
    }
  }
}

/*!
  Compute the pose from a set of n 2D point (x,y) and m 3D points
  (X,Y,Z) using the Ransac algorithm. It is not assumed that
  the 2D and 3D points are registred (there is nm posibilities)

  at least numberOfInlierToReachAConsensus of true correspondance are required
  to validate the pose

  the inliers are given in xi, yi, Xi, Yi, Zi

  the pose is return in cMo
 */
void
vpPose::ransac(const int n,
	       const double *x, const double *y,
	       const int m,
	       const double *X, const double *Y, const double *Z,
	       const int  numberOfInlierToReachAConsensus,
	       const double threshold,
	       int &ninliers,
	       vpColVector &xi,  vpColVector &yi,
	       vpColVector &Xi,  vpColVector &Yi,  vpColVector &Zi,
	       vpHomogeneousMatrix &cMo)
{


  double tms = vpTime::measureTimeMs() ;
  vpColVector data ;
  int i;
  vpPose::initRansac(n,x,y,m,X,Y,Z, data) ;

  vpColVector M(16) ;
  vpColVector inliers(n*m) ;
  vpRansac<vpPose>::ransac(n*m,data,4,
			   threshold, M,inliers,
			   numberOfInlierToReachAConsensus) ;


  // we count the number of inliers
  ninliers = 0 ;
  for(i=0 ; i < n*m ; i++)
  {
    if (inliers[i]==1)
    {
      ninliers++ ;
    }
  }

  xi.resize(ninliers) ;
  yi.resize(ninliers) ;
  Xi.resize(ninliers) ;
  Yi.resize(ninliers) ;
  Zi.resize(ninliers) ;

  int k =0 ;
  for(i=0 ; i < n*m ; i++)
  {
    if (inliers[i]==1)
    {
      xi[k] = data[5*i] ;
      yi[k] = data[5*i+1] ;
      Xi[k] = data[5*i+2] ;
      Yi[k] = data[5*i+3] ;
      Zi[k] = data[5*i+4] ;
      k++ ;
    }
  }

  for (i=0 ; i <16 ; i++)
  {
      cMo.data[i] = M[i];
  }

  cout << vpTime::measureTimeMs() - tms << "ms" << endl ;

}

/*!
  Compute the pose from a set of n 2D point (x,y) in p and m 3D points
  (X,Y,Z) in P using the Ransac algorithm. It is not assumed that
  the 2D and 3D points are registred (there is nm posibilities)

  at least numberOfInlierToReachAConsensus of true correspondance are required
  to validate the pose

  the inliers are given in a list of vpPoint

  the pose is return in cMo
 */
void
vpPose::ransac(const int n,
	       const vpPoint *p,
	       const int m,
	       const vpPoint *P,
	       const int   numberOfInlierToReachAConsensus,
	       const double threshold,
	       int &ninliers,
	       vpList<vpPoint> &lPi,
	       vpHomogeneousMatrix &cMo)
{


  double *x, *y;
  x = new double [n];
  y = new double [n] ;
  int i;
  for (i=0 ; i < n ; i++)
  {
    x[i] = p[i].get_x() ;
    y[i] = p[i].get_y() ;
  }
  double *X, *Y, *Z;
  X = new double [m];
  Y = new double [m];
  Z = new double [m];
  for (i=0 ; i < m ; i++)
  {
    X[i] = P[i].get_oX() ;
    Y[i] = P[i].get_oY() ;
    Z[i] = P[i].get_oZ() ;
  }

  vpColVector xi,yi,Xi,Yi,Zi ;

  ransac(n,x,y,
	 m,X,Y,Z, numberOfInlierToReachAConsensus,
	 threshold,
	 ninliers,
	 xi,yi,Xi,Yi,Zi,
	 cMo) ;


  for(i=0 ; i < ninliers ; i++)
  {
    vpPoint Pi ;
    Pi.setWorldCoordinates(Xi[i],Yi[i], Zi[i]) ;
    Pi.set_x(xi[i]) ;
    Pi.set_y(yi[i]) ;
    lPi += Pi ;
  }

  delete [] x;
  delete [] y;
  delete [] X;
  delete [] Y;
  delete [] Z;
}



/*!
  Compute the pose from a list lp of  2D point (x,y)  and  a list lP 3D points
  (X,Y,Z) in P using the Ransac algorithm. It is not assumed that
  the 2D and 3D points are registred

  at least numberOfInlierToReachAConsensus of true correspondance are required
  to validate the pose

  the inliers are given in a list of lPi vpPoint

  the pose is return in cMo
 */
void
vpPose::ransac(vpList<vpPoint> &lp,
	       vpList<vpPoint> &lP,
	       const int numberOfInlierToReachAConsensus,
	       const double threshold,
	       int &ninliers,
	       vpList<vpPoint> &lPi,
	       vpHomogeneousMatrix &cMo)
{
  int n = lp.nbElement() ;
  int m = lP.nbElement() ;

  double *x, *y;
  x = new double [n];
  y = new double [n];

  vpPoint pin ;

  lp.front() ;
  int i = 0 ;
  while(!lp.outside())
  {
    pin = lp.value() ; lp.next() ;
    x[i] = pin.get_x() ;
    y[i] = pin.get_y() ;
    i++ ;
  }

  double *X, *Y, *Z;
  X = new double [m];
  Y = new double [m];
  Z = new double [m];
  lP.front() ;
   i = 0 ;
  while(!lP.outside())
  {
    pin = lP.value() ; lP.next() ;
    X[i] = pin.get_oX() ;
    Y[i] = pin.get_oY() ;
    Z[i] = pin.get_oZ() ;
    i++ ;
  }

  vpColVector xi,yi,Xi,Yi,Zi ;

  ransac(n,x,y,
	 m,X,Y,Z, numberOfInlierToReachAConsensus,
	 threshold,
	 ninliers,
	 xi,yi,Xi,Yi,Zi,
	 cMo) ;


  for( i=0 ; i < ninliers ; i++)
  {
    vpPoint Pi ;
    Pi.setWorldCoordinates(Xi[i],Yi[i], Zi[i]) ;
    Pi.set_x(xi[i]) ;
    Pi.set_y(yi[i]) ;
    lPi += Pi ;
  }


  delete [] x;
  delete [] y;
  delete [] X;
  delete [] Y;
  delete [] Z;

}




/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
