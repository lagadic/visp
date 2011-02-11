/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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
#include <iostream>
#include <cmath>    // std::fabs
#include <limits>   // numeric_limits

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
vpPose::degenerateConfiguration(vpColVector &x, unsigned int *ind)
{

  //  vpTRACE("%p %p %d",&x, ind, x.getRows()) ;
  for (int i=1 ; i < 4 ; i++)
    for (int j=0 ; j<i ; j++)
    {
      unsigned int indi =  5*ind[i] ;
      unsigned int indj =  5*ind[j] ;

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
vpPose::computeTransformation(vpColVector &x, unsigned int *ind, vpColVector &M)
{
  unsigned int i ;

  vpPoint p[4] ;

  vpPose pose ;
  pose.clearPoint() ;
  for(i=0 ; i < 4 ; i++)
  {

    unsigned int index = 5*ind[i] ;

    p[i].set_x(x[index]) ;
    p[i].set_y(x[index+1]) ;

    p[i].setWorldCoordinates(x[index+2],x[index+3], x[index+4]) ;
    pose.addPoint(p[i]) ;
  }


  //  pose.printPoint() ;
  vpHomogeneousMatrix cMo ;
  try {
    pose.computePose(vpPose::DEMENTHON, cMo) ;
    //    std::cout << cMo << std::endl ;
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

  unsigned int i ;
  unsigned int n = x.getRows()/5 ;

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
vpPose::initRansac(const unsigned int n,
		   const double *x, const double *y,
		   const unsigned int m,
		   const double *X, const double *Y, const double *Z,
		   vpColVector &data)
{
  data.resize(5*n*m) ;
  unsigned int k =0 ;
  for (unsigned int i=0 ; i < n ; i++)
  {
    for (unsigned int j=0 ; j < m ; j++)
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

  At least numberOfInlierToReachAConsensus of true correspondance are required
  to validate the pose

  The inliers are given in xi, yi, Xi, Yi, Zi

  The pose is returned in cMo.

  \param n : Number of 2d points.
  \param x : Array (of size \e n) of the x coordinates of the 2d points.
  \param y : Array (of size \e n) of the y coordinates of the 2d points.
  \param m : Number of 3d points.
  \param X : Array (of size \e m) of the oX coordinates of the 3d points.
  \param Y : Array (of size \e m) of the oY coordinates of the 3d points.
  \param Z : Array (of size \e m) of the oZ coordinates of the 3d points.
  \param numberOfInlierToReachAConsensus : The minimum number of inlier to have
  to consider a trial as correct.
  \param threshold : The maximum error allowed between the 2d points and the
  reprojection of its associated 3d points by the current pose (in meter).
  \param ninliers : Number of inliers found for the best solution.
  \param xi : Array (of size \e ninliers) of the x coordinates of the inliers.
  \param yi : Array (of size \e ninliers) of the y coordinates of the inliers.
  \param Xi : Array (of size \e ninliers) of the oX coordinates of the inliers.
  \param Yi : Array (of size \e ninliers) of the oY coordinates of the inliers.
  \param Zi : Array (of size \e ninliers) of the oZ coordinates of the inliers.
  \param cMo : The computed pose (best solution).
  \param maxNbTrials : Maximum number of trials before considering a solution
  fitting the required \e numberOfInlierToReachAConsensus and \e threshold
  cannot be found.
 */
void
vpPose::ransac(const unsigned int n,
	       const double *x, const double *y,
	       const unsigned int m,
	       const double *X, const double *Y, const double *Z,
	       const int  numberOfInlierToReachAConsensus,
	       const double threshold,
	       unsigned int &ninliers,
	       vpColVector &xi,  vpColVector &yi,
	       vpColVector &Xi,  vpColVector &Yi,  vpColVector &Zi,
         vpHomogeneousMatrix &cMo,
         const int maxNbTrials)
{


  double tms = vpTime::measureTimeMs() ;
  vpColVector data ;
  unsigned int i;
  vpPose::initRansac(n,x,y,m,X,Y,Z, data) ;

  vpColVector M(16) ;
  vpColVector inliers(n*m) ;
  vpRansac<vpPose>::ransac(n*m,data,4,
			   threshold, M,inliers,
         numberOfInlierToReachAConsensus, 0.0, maxNbTrials) ;


  // we count the number of inliers
  ninliers = 0 ;
  for(i=0 ; i < n*m ; i++)
  {
    //if (inliers[i]==1)
    if (std::fabs(inliers[i]-1) <= std::fabs(vpMath::maximum(inliers[i], 1.)) * std::numeric_limits<double>::epsilon())
    {
      ninliers++ ;
    }
  }

  xi.resize(ninliers) ;
  yi.resize(ninliers) ;
  Xi.resize(ninliers) ;
  Yi.resize(ninliers) ;
  Zi.resize(ninliers) ;

  unsigned int k =0 ;
  for(i=0 ; i < n*m ; i++)
  {
    //if (inliers[i]==1)
    if (std::fabs(inliers[i]-1) <= std::fabs(vpMath::maximum(inliers[i], 1.)) * std::numeric_limits<double>::epsilon())
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

  std::cout << vpTime::measureTimeMs() - tms << "ms" << std::endl ;

}

/*!
  Compute the pose from a set of n 2D point (x,y) in p and m 3D points
  (X,Y,Z) in P using the Ransac algorithm. It is not assumed that
  the 2D and 3D points are registred (there is nm posibilities)

  At least numberOfInlierToReachAConsensus of true correspondance are required
  to validate the pose

  The inliers are given in a list of vpPoint

  The pose is returned in cMo.

  \param n : Number of 2d points.
  \param p : Array (of size n) of 2d points (x and y attributes are used).
  \param m : Number of 3d points.
  \param P : Array of size m of 3d points (oX, oY and oZ attributes are used).
  \param numberOfInlierToReachAConsensus : The minimum number of inlier to have
  to consider a trial as correct.
  \param threshold : The maximum error allowed between the 2d points and the
  reprojection of its associated 3d points by the current pose (in meter).
  \param ninliers : Number of inliers found for the best solution.
  \param lPi : List of points (2d and 3d) that are inliers for the best solution.
  \param cMo : The computed pose (best solution).
  \param maxNbTrials : Maximum number of trials before considering a solution
  fitting the required \e numberOfInlierToReachAConsensus and \e threshold
  cannot be found.
 */
void
vpPose::ransac(const unsigned int n,
	       const vpPoint *p,
	       const unsigned int m,
	       const vpPoint *P,
	       const int   numberOfInlierToReachAConsensus,
	       const double threshold,
	       unsigned int &ninliers,
	       vpList<vpPoint> &lPi,
         vpHomogeneousMatrix &cMo,
         const int maxNbTrials)
{


  double *x, *y;
  x = new double [n];
  y = new double [n] ;
  for (unsigned int i=0 ; i < n ; i++)
  {
    x[i] = p[i].get_x() ;
    y[i] = p[i].get_y() ;
  }
  double *X, *Y, *Z;
  X = new double [m];
  Y = new double [m];
  Z = new double [m];
  for (unsigned int i=0 ; i < m ; i++)
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
   cMo, maxNbTrials) ;


  for(unsigned int i=0 ; i < ninliers ; i++)
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

  At least numberOfInlierToReachAConsensus of true correspondance are required
  to validate the pose

  The inliers are given in a list of vpPoint lPi.

  The pose is returned in cMo.

  \param lp : List of 2d points (x and y attributes are used).
  \param lP : List of 3d points (oX, oY and oZ attributes are used).
  \param numberOfInlierToReachAConsensus : The minimum number of inlier to have
  to consider a trial as correct.
  \param threshold : The maximum error allowed between the 2d points and the
  reprojection of its associated 3d points by the current pose (in meter).
  \param ninliers : Number of inliers found for the best solution.
  \param lPi : List of points (2d and 3d) that are inliers for the best solution.
  \param cMo : The computed pose (best solution).
  \param maxNbTrials : Maximum number of trials before considering a solution
  fitting the required \e numberOfInlierToReachAConsensus and \e threshold
  cannot be found.
 */
void
vpPose::ransac(vpList<vpPoint> &lp,
	       vpList<vpPoint> &lP,
	       const int numberOfInlierToReachAConsensus,
	       const double threshold,
	       unsigned int &ninliers,
	       vpList<vpPoint> &lPi,
         vpHomogeneousMatrix &cMo,
         const int maxNbTrials)
{
  unsigned int n = lp.nbElement() ;
  unsigned int m = lP.nbElement() ;

  double *x, *y;
  x = new double [n];
  y = new double [n];

  vpPoint pin ;

  lp.front() ;
  unsigned int i = 0 ;
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
   cMo, maxNbTrials) ;


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
