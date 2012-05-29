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

#include <iostream>
#include <cmath>    // std::fabs
#include <limits>   // numeric_limits
#include <stdlib.h>

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

/*! compute the pose using the Ransac approach 
 
  \param cMo : Computed pose
*/
void vpPose::poseRansac(vpHomogeneousMatrix & cMo)
{
  std::vector<vpPoint> p2D;
  std::vector<vpPoint> p3D;
  
  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it)
  {
    vpPoint pt = *it;
    
    vpPoint pt2D;
    pt2D.set_x(pt.get_x());
    pt2D.set_y(pt.get_y());
    p2D.push_back(pt2D);
    
    vpPoint pt3D;
    pt3D.setWorldCoordinates(pt.get_oX(),pt.get_oY(),pt.get_oZ());
    p3D.push_back(pt3D);
  }
  
  unsigned int nbInliers;
  vpPose::ransac(p2D,p3D,ransacNbInlierConsensus,ransacThreshold,nbInliers, ransacInliers,cMo,ransacMaxTrials);
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
  if(n != m)
    vpERROR_TRACE("Different number of 2D points from 3D points");
  else{
    std::vector<vpPoint> p2D;
    std::vector<vpPoint> p3D;
    
    for (unsigned int i = 0 ; i < n ; i++)
    {
      vpPoint pt2D;
      pt2D.set_x(x[i]);
      pt2D.set_y(y[i]);
      p2D.push_back(pt2D);
      
      vpPoint pt3D;
      pt3D.setWorldCoordinates(X[i], Y[i], Z[i]);
      p3D.push_back(pt3D);
    }
    
    std::vector<vpPoint> listInliers;
    
    vpPose::ransac(p2D,p3D,numberOfInlierToReachAConsensus,
                  threshold,ninliers,listInliers,cMo,maxNbTrials);
    
    xi = vpColVector(listInliers.size());
    yi = vpColVector(listInliers.size());
    Xi = vpColVector(listInliers.size());
    Yi = vpColVector(listInliers.size());
    Zi = vpColVector(listInliers.size());
    for(unsigned int i = 0 ; i < listInliers.size() ; i++){
      xi[i] = listInliers[i].get_x();
      yi[i] = listInliers[i].get_y();
      
      Xi[i] = listInliers[i].get_oX();
      Yi[i] = listInliers[i].get_oY();
      Zi[i] = listInliers[i].get_oZ();
    }
  }
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
  if(n != m)
    vpERROR_TRACE("Different number of 2D points from 3D points");
  else{
    std::vector<vpPoint> p2D;
    std::vector<vpPoint> p3D;
    
    for (unsigned int i = 0 ; i < n ; i++)
    {
      vpPoint pt2D = p[i];
      p2D.push_back(pt2D);
      
      vpPoint pt3D = P[i];
      p3D.push_back(pt3D);
    }
    
    std::vector<vpPoint> listInliers;
    
    vpPose::ransac(p2D,p3D,numberOfInlierToReachAConsensus,
                  threshold,ninliers,listInliers,cMo,maxNbTrials);
    
    for(unsigned int i = 0 ; i < listInliers.size() ; i++)
      lPi.push_back(listInliers[i]);
  }
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
  if(lp.size() != lP.size())
    vpERROR_TRACE("Different number of 2D points from 3D points");
  else{
    std::vector<vpPoint> p2D;
    std::vector<vpPoint> p3D;
    
    for (std::list<vpPoint>::const_iterator it = lp.begin(); it != lp.end(); ++it)
    {
      vpPoint pt = *it;
      p2D.push_back(pt);
    }
    
    for (std::list<vpPoint>::const_iterator it = lP.begin(); it != lP.end(); ++it)
    {
      vpPoint pt = *it;
      p3D.push_back(pt);
    }
    
    std::vector<vpPoint> listInliers;
    
    vpPose::ransac(p2D,p3D,numberOfInlierToReachAConsensus,
                  threshold,ninliers,listInliers,cMo,maxNbTrials);
    
    for(unsigned int i = 0 ; i < listInliers.size() ; i++)
      lPi.push_back(listInliers[i]);
  }
}

/*!
  Compute the pose from a vector p2D of  2D point (x,y)  and  a vector p3D of 3D points
  (X,Y,Z) in P using the Ransac algorithm. It is not assumed that
  the 2D and 3D points are registred

  At least numberOfInlierToReachAConsensus of true correspondance are required
  to validate the pose

  The inliers are given in a list of vpPoint listInliers.

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
void vpPose::ransac(std::vector<vpPoint> &p2D, 
            std::vector<vpPoint> &p3D, 
            const int &numberOfInlierToReachAConsensus, 
            const double &threshold,
            unsigned int &ninliers,
            std::vector<vpPoint> &listInliers,
            vpHomogeneousMatrix &cMo,
            const int &maxNbTrials )
{
  if(p2D.size() != p3D.size())
    vpERROR_TRACE("Number of 2D points different from number of 3D points");
  else{
    //Init Points
    std::vector<vpPoint> points;
    for(unsigned int i = 0 ; i < p2D.size() ; i++){
      vpPoint pt;
      pt.set_x(p2D[i].get_x());
      pt.set_y(p2D[i].get_y());
      pt.setWorldCoordinates(p3D[i].get_oX(),p3D[i].get_oY(),p3D[i].get_oZ()) ;
      points.push_back(pt);
    }
    
    srand(0);
    std::vector<unsigned int> best_consensus;
    std::vector<unsigned int> cur_consensus;
    std::vector<unsigned int> cur_outliers;
    std::vector<unsigned int> cur_randoms;
    unsigned int size = points.size();
    int nbTrials = 0;
    unsigned int nbMinRandom = 4 ;
    ninliers = 0;
    
    bool foundSolution = false;
    
    std::cout << "Error : " << threshold << std::endl;
    
    while (nbTrials < maxNbTrials && ninliers < (unsigned)numberOfInlierToReachAConsensus)
    { 
      cur_outliers.clear();
      cur_randoms.clear();
      
      std::vector<bool> usedPt(size, false);
      
      vpPose poseMin ;
      for(unsigned int i = 0; i < nbMinRandom; i++)
      {
        int r = rand()%size;
        while(usedPt[r] ) r = rand()%size;
        usedPt[r] = true;
        poseMin.addPoint(points[r]) ;
        cur_randoms.push_back(r);
      }
      poseMin.computePose(vpPose::DEMENTHON,cMo) ;

      double r = poseMin.computeResidual(cMo) ;
      r = sqrt(r)/(double)nbMinRandom;
      
      if (r < threshold)
      {
        unsigned int nbInliersCur = 0;
        //std::cout << "Résultat : " << r << " / " << vpPoseVector(cMo).sumSquare()<< std::endl ;
        for (unsigned int i=0 ; i < size ; i++) 
        { 
          vpPoint p(points[i]) ;
          p.track(cMo) ;

          double d = vpMath::sqr(p.get_x() - points[i].get_x()) + vpMath::sqr(p.get_y() - points[i].get_y()) ;
          double error = sqrt(d) ;
          if(error < threshold){ // the point is considered an inlier if the error is below the threshold
            nbInliersCur++;
            cur_consensus.push_back(i);
          }    
          else
            cur_outliers.push_back(i);
        }
      // std::cout << "Nombre d'inliers " << nbInliersCur << "/" << ninliers << std::endl ;
        
        if(nbInliersCur > ninliers)
        {
          foundSolution = true;
          best_consensus = cur_consensus;
          ninliers = nbInliersCur;
        }
        
        nbTrials++;
        cur_consensus.clear();
        
        if(nbTrials >= maxNbTrials){
          vpERROR_TRACE("Ransac reached the maximum number of trials");
          foundSolution = true;
        }
      }
    }
    
    if(foundSolution){
      std::cout << "Nombre d'inliers " << ninliers << std::endl ;
      
      //Display the random picked points
      std::cout << "Randoms : "; 
      for(unsigned int i = 0 ; i < cur_randoms.size() ; i++)
        std::cout << cur_randoms[i] << " ";
      std::cout << std::endl;
      
      //Display the outliers
      std::cout << "Outliers : "; 
      for(unsigned int i = 0 ; i < cur_outliers.size() ; i++)
        std::cout << cur_outliers[i] << " ";
      std::cout << std::endl;
      
      if(ninliers >= (unsigned)numberOfInlierToReachAConsensus)
      {    
        vpPose pose ;
        for(unsigned i = 0 ; i < best_consensus.size(); i++)
        {
          pose.addPoint(points[best_consensus[i]]) ;
          listInliers.push_back(points[best_consensus[i]]);
        }
          
        pose.computePose(vpPose::LAGRANGE_VIRTUAL_VS,cMo) ;
        std::cout << "Residue finale "<< pose.computeResidual(cMo)  << std::endl ;
      }
    }
  }
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
