/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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
 * Aurelien Yol
 *
 *****************************************************************************/


/*!
  \file vpPoseRansac.cpp
  \brief function used to estimate a pose using the Ransac algorithm
*/

#include <visp/vpPose.h>
#include <visp/vpColVector.h>
#include <visp/vpRansac.h>
#include <visp/vpTime.h>
#include <visp/vpList.h>
#include <visp/vpPoseException.h>

#include <iostream>
#include <cmath>    // std::fabs
#include <limits>   // numeric_limits
#include <stdlib.h>

#define eps 1e-6


/*! 
  Compute the pose using the Ransac approach. 
 
  \param cMo : Computed pose
*/
void vpPose::poseRansac(vpHomogeneousMatrix & cMo)
{  
  srand(0);
  std::vector<unsigned int> best_consensus;
  std::vector<unsigned int> cur_consensus;
  std::vector<unsigned int> cur_outliers;
  std::vector<unsigned int> cur_randoms;
  unsigned int size = listP.size();
  int nbTrials = 0;
  unsigned int nbMinRandom = 4 ;
  unsigned int nbInliers = 0;
  
  bool foundSolution = false;
  
  while (nbTrials < ransacMaxTrials && nbInliers < (unsigned)ransacNbInlierConsensus)
  { 
    cur_outliers.clear();
    cur_randoms.clear();
    
    std::vector<bool> usedPt(size, false);
    
    vpPose poseMin ;
    for(unsigned int i = 0; i < nbMinRandom; i++)
    {
      unsigned int r = (unsigned int)rand()%size;
      while(usedPt[r] ) r = (unsigned int)rand()%size;
      usedPt[r] = true;        
      
      std::list<vpPoint>::const_iterator iter = listP.begin();
      std::advance(iter, r);
      vpPoint pt = *iter;
      
      bool degenerate = false;
      for(std::list<vpPoint>::const_iterator it = poseMin.listP.begin(); it != poseMin.listP.end(); ++it){
          vpPoint ptdeg = *it;
          if( ((fabs(pt.get_x() - ptdeg.get_x()) < 1e-6) && (fabs(pt.get_y() - ptdeg.get_y()) < 1e-6))  ||
              ((fabs(pt.get_oX() - ptdeg.get_oX()) < 1e-6) && (fabs(pt.get_oY() - ptdeg.get_oY()) < 1e-6) && (fabs(pt.get_oZ() - ptdeg.get_oZ()) < 1e-6))){
            degenerate = true;
            break;
          }
      }
      
      if(!degenerate){
        poseMin.addPoint(pt) ;
        cur_randoms.push_back(r);
      }
      else
        i--;
    }
    
    poseMin.computePose(vpPose::DEMENTHON,cMo) ;
    double r = poseMin.computeResidual(cMo) ;
    r = sqrt(r)/(double)nbMinRandom;
    
    if (r < ransacThreshold)
    {
      unsigned int nbInliersCur = 0;
      //std::cout << "Résultat : " << r << " / " << vpPoseVector(cMo).sumSquare()<< std::endl ;
      unsigned int iter = 0;
      for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it)
      { 
        vpPoint pt = *it;
        vpPoint p(pt) ;
        p.track(cMo) ;

        double d = vpMath::sqr(p.get_x() - pt.get_x()) + vpMath::sqr(p.get_y() - pt.get_y()) ;
        double error = sqrt(d) ;
        if(error < ransacThreshold){ // the point is considered an inlier if the error is below the threshold
          nbInliersCur++;
          cur_consensus.push_back(iter);
        }    
        else
          cur_outliers.push_back(iter);
        
        iter++;
      }
      //std::cout << "Nombre d'inliers " << nbInliersCur << "/" << nbInliers << std::endl ;
      
      if(nbInliersCur > nbInliers)
      {
        foundSolution = true;
        best_consensus = cur_consensus;
        nbInliers = nbInliersCur;
      }
      
      nbTrials++;
      cur_consensus.clear();
      
      if(nbTrials >= ransacMaxTrials){
        vpERROR_TRACE("Ransac reached the maximum number of trials");
        foundSolution = true;
      }
    }
  }
    
  if(foundSolution){
    //std::cout << "Nombre d'inliers " << nbInliers << std::endl ;
    
    //Display the random picked points
    /*
    std::cout << "Randoms : "; 
    for(unsigned int i = 0 ; i < cur_randoms.size() ; i++)
      std::cout << cur_randoms[i] << " ";
    std::cout << std::endl;
    */
    
    //Display the outliers
    /*
    std::cout << "Outliers : "; 
    for(unsigned int i = 0 ; i < cur_outliers.size() ; i++)
      std::cout << cur_outliers[i] << " ";
    std::cout << std::endl;
    */
    
    if(nbInliers >= (unsigned)ransacNbInlierConsensus)
    {    
      vpPose pose ;
      for(unsigned i = 0 ; i < best_consensus.size(); i++)
      {
        std::list<vpPoint>::const_iterator iter = listP.begin();
        std::advance(iter, best_consensus[i]);
        vpPoint pt = *iter;
      
        pose.addPoint(pt) ;
        ransacInliers.push_back(pt);
      }
        
      pose.computePose(vpPose::LAGRANGE_VIRTUAL_VS,cMo) ;
      //std::cout << "Residue finale "<< pose.computeResidual(cMo)  << std::endl ;
    }
  }
}

/*!
  Match a vector p2D of  2D point (x,y)  and  a vector p3D of 3D points
  (X,Y,Z) using the Ransac algorithm.

  At least numberOfInlierToReachAConsensus of true correspondance are required
  to validate the pose

  The inliers are given in a vector of vpPoint listInliers.

  The pose is returned in cMo.

  \param p2D : Vector of 2d points (x and y attributes are used).
  \param p3D : Vector of 3d points (oX, oY and oZ attributes are used).
  \param numberOfInlierToReachAConsensus : The minimum number of inlier to have
  to consider a trial as correct.
  \param threshold : The maximum error allowed between the 2d points and the
  reprojection of its associated 3d points by the current pose (in meter).
  \param ninliers : Number of inliers found for the best solution.
  \param listInliers : Vector of points (2d and 3d) that are inliers for the best solution.
  \param cMo : The computed pose (best solution).
  \param maxNbTrials : Maximum number of trials before considering a solution
  fitting the required \e numberOfInlierToReachAConsensus and \e threshold
  cannot be found.
*/
void vpPose::findMatch(std::vector<vpPoint> &p2D, 
            std::vector<vpPoint> &p3D, 
            const unsigned int &numberOfInlierToReachAConsensus,
            const double &threshold,
            unsigned int &ninliers,
            std::vector<vpPoint> &listInliers,
            vpHomogeneousMatrix &cMo,
            const int &maxNbTrials )
{
  vpPose pose;
  
  int nbPts = 0;
  for(unsigned int i = 0 ; i < p2D.size() ; i++)
  {
    for(unsigned int j = 0 ; j < p3D.size() ; j++)
    {
      vpPoint pt;
      pt.set_x(p2D[i].get_x());
      pt.set_y(p2D[i].get_y());
      pt.setWorldCoordinates(p3D[j].getWorldCoordinates());
      pose.addPoint(pt);
      nbPts++;
    }
  }
  
  if (pose.listP.size() < 4)
  {
    vpERROR_TRACE("Ransac method cannot be used in that case ") ;
    vpERROR_TRACE("(at least 4 points are required)") ;
    vpERROR_TRACE("Not enough point (%d) to compute the pose  ",pose.listP.size()) ;
    throw(vpPoseException(vpPoseException::notEnoughPointError,
      "Not enough points ")) ;
  }
  else
  {
    pose.setRansacMaxTrials(maxNbTrials);
    pose.setRansacNbInliersToReachConsensus(numberOfInlierToReachAConsensus);
    pose.setRansacThreshold(threshold);
    pose.computePose(vpPose::RANSAC, cMo);
    ninliers = pose.getRansacNbInliers();
    listInliers = pose.getRansacInliers();
  }
}

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS

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

      if ((fabs(x[indi] - x[indj]) < 1e-6) && (fabs(x[indi+1] - x[indj+1]) < 1e-6))
      { return true ; }
      
      if ((fabs(x[indi+2] - x[indj+2]) < 1e-6) && (fabs(x[indi+3] - x[indj+3]) < 1e-6) && (fabs(x[indi+4] - x[indj+4]) < 1e-6)) 
      { return true ;  }
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
               std::list<vpPoint> &lPi,
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
    Pi.setWorldCoordinates(Xi[i], Yi[i], Zi[i]) ;
    Pi.set_x(xi[i]) ;
    Pi.set_y(yi[i]) ;
    lPi.push_back(Pi) ;
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
vpPose::ransac(std::list<vpPoint> &lp,
               std::list<vpPoint> &lP,
               const int numberOfInlierToReachAConsensus,
               const double threshold,
               unsigned int &ninliers,
               std::list<vpPoint> &lPi,
               vpHomogeneousMatrix &cMo,
               const int maxNbTrials)
{
  unsigned int i;
  unsigned int n = lp.size() ;
  unsigned int m = lP.size() ;

  double *x, *y;
  x = new double [n];
  y = new double [n];

  vpPoint pin ;
  i = 0;
  for (std::list<vpPoint>::const_iterator it = lp.begin(); it != lp.end(); ++it)
  {
    pin = *it;
    x[i] = pin.get_x() ;
    y[i] = pin.get_y() ;
    ++ i;
  }

  double *X, *Y, *Z;
  X = new double [m];
  Y = new double [m];
  Z = new double [m];
  i = 0;
  for (std::list<vpPoint>::const_iterator it = lP.begin(); it != lP.end(); ++it)
  {
    pin = *it;
    X[i] = pin.get_oX() ;
    Y[i] = pin.get_oY() ;
    Z[i] = pin.get_oZ() ;
    ++i;
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
    lPi.push_back(Pi);
  }

  delete [] x;
  delete [] y;
  delete [] X;
  delete [] Y;
  delete [] Z;

}
#endif // VISP_BUILD_DEPRECATED_FUNCTIONS

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
