/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Pose computation.
 *
 * Authors:
 * Eric Marchand
 * Aurelien Yol
 * Souriya Trinh
 *
 *****************************************************************************/


/*!
  \file vpPoseRansac.cpp
  \brief function used to estimate a pose using the Ransac algorithm
*/

#include <iostream>
#include <cmath>        // std::fabs
#include <limits>       // numeric_limits
#include <stdlib.h>
#include <algorithm>    // std::count
#include <float.h>      // DBL_MAX

#include <visp3/vision/vpPose.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpRansac.h>
#include <visp3/core/vpTime.h>
#include <visp3/core/vpList.h>
#include <visp3/vision/vpPoseException.h>
#include <visp3/core/vpMath.h>

#define eps 1e-6


/*! 
  Compute the pose using the Ransac approach. 
 
  \param cMo : Computed pose
  \param func : Pointer to a function that takes in parameter a vpHomogeneousMatrix
  and returns true if the pose check is OK or false otherwise
  \return True if we found at least 4 points with a reprojection error below ransacThreshold.
*/
bool vpPose::poseRansac(vpHomogeneousMatrix & cMo, bool (*func)(vpHomogeneousMatrix *))
{  
  ransacInliers.clear();
  ransacInlierIndex.clear();

  srand(0); //Fix seed here so we will have the same pseudo-random series at each run.
  std::vector<unsigned int> best_consensus;
  std::vector<unsigned int> cur_consensus;
  std::vector<unsigned int> cur_outliers;
  std::vector<unsigned int> cur_randoms;
  unsigned int size = (unsigned int)listP.size();
  int nbTrials = 0;
  unsigned int nbMinRandom = 4 ;
  unsigned int nbInliers = 0;
  double r, r_lagrange, r_dementhon;

  vpHomogeneousMatrix cMo_lagrange, cMo_dementhon;

  if (size < 4) {
    //vpERROR_TRACE("Not enough point to compute the pose");
    throw(vpPoseException(vpPoseException::notInitializedError,
                          "Not enough point to compute the pose")) ;
  }

  bool foundSolution = false;
  
  while (nbTrials < ransacMaxTrials && nbInliers < (unsigned)ransacNbInlierConsensus)
  {
    //Hold the list of the index of the inliers (points in the consensus set)
    cur_consensus.clear();

    //Use a temporary variable because if not, the cMo passed in parameters will be modified when
    // we compute the pose for the minimal sample sets but if the pose is not correct when we pass
    // a function pointer we do not want to modify the cMo passed in parameters
    vpHomogeneousMatrix cMo_tmp;
    cur_outliers.clear();
    cur_randoms.clear();
    
    //Vector of used points, initialized at false for all points
    std::vector<bool> usedPt(size, false);
    
    vpPose poseMin;
    for(unsigned int i = 0; i < nbMinRandom;)
    {
      if((size_t) std::count(usedPt.begin(), usedPt.end(), true) == usedPt.size()) {
        //All points was picked once, break otherwise we stay in an infinite loop
        break;
      }

      //Pick a point randomly
      unsigned int r_ = (unsigned int) rand() % size;
      while(usedPt[r_]) {
        //If already picked, pick another point randomly
        r_ = (unsigned int) rand() % size;
      }
      //Mark this point as already picked
      usedPt[r_] = true;
      
      std::list<vpPoint>::const_iterator iter = listP.begin();
      std::advance(iter, r_);
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
      if(!degenerate) {
        poseMin.addPoint(pt);
        cur_randoms.push_back(r_);
        //Increment the number of points picked
        i++;
      }
    }

    if(poseMin.npt < nbMinRandom) {
      nbTrials++;
      continue;
    }

    //Flags set if pose computation is OK
    bool is_valid_lagrange = false;
    bool is_valid_dementhon = false;

    //Set maximum value for residuals
    r_lagrange = DBL_MAX;
    r_dementhon = DBL_MAX;

    try {
      poseMin.computePose(vpPose::LAGRANGE, cMo_lagrange);
      r_lagrange = poseMin.computeResidual(cMo_lagrange);
      is_valid_lagrange = true;
    } catch(/*vpException &e*/...) {
//      std::cerr << e.what() << std::endl;
    }

    try {
      poseMin.computePose(vpPose::DEMENTHON, cMo_dementhon);
      r_dementhon = poseMin.computeResidual(cMo_dementhon);
      is_valid_dementhon = true;
    } catch(/*vpException &e*/...) {
//      std::cerr << e.what() << std::endl;
    }

    //If residual returned is not a number (NAN), set valid to false
    if(vpMath::isNaN(r_lagrange)) {
      is_valid_lagrange = false;
      r_lagrange = DBL_MAX;
    }

    if(vpMath::isNaN(r_dementhon)) {
      is_valid_dementhon = false;
      r_dementhon = DBL_MAX;
    }

    //If at least one pose computation is OK,
    //we can continue, otherwise pick another random set
    if(is_valid_lagrange || is_valid_dementhon) {
      if (r_lagrange < r_dementhon) {
        r = r_lagrange;
//        cMo = cMo_lagrange;
        cMo_tmp = cMo_lagrange;
      }
      else {
        r = r_dementhon;
//        cMo = cMo_dementhon;
        cMo_tmp = cMo_dementhon;
      }
      r = sqrt(r) / (double) nbMinRandom;

      //Filter the pose using some criterion (orientation angles, translations, etc.)
      bool isPoseValid = true;
      if(func != NULL) {
        isPoseValid = func(&cMo_tmp);
        if(isPoseValid) {
          cMo = cMo_tmp;
        }
      } else {
        //No post filtering on pose, so copy cMo_temp to cMo
        cMo = cMo_tmp;
      }

      if (isPoseValid && r < ransacThreshold)
      {
        unsigned int nbInliersCur = 0;
        unsigned int iter = 0;
        for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it)
        {
          vpPoint pt = *it;
          vpPoint p(pt) ;
          p.track(cMo) ;

          double d = vpMath::sqr(p.get_x() - pt.get_x()) + vpMath::sqr(p.get_y() - pt.get_y()) ;
          double error = sqrt(d) ;
          if(error < ransacThreshold) {
            // the point is considered as inlier if the error is below the threshold
            // But, we need also to check if it is not a degenerate point
            bool degenerate = false;

            for(unsigned int it_inlier_index = 0; it_inlier_index< cur_consensus.size(); it_inlier_index++){
              std::list<vpPoint>::const_iterator it_point = listP.begin();
              std::advance(it_point, cur_consensus[it_inlier_index]);
              pt = *it_point;

              vpPoint ptdeg = *it;
              if( ((fabs(pt.get_x() - ptdeg.get_x()) < 1e-6) && (fabs(pt.get_y() - ptdeg.get_y()) < 1e-6))  ||
                  ((fabs(pt.get_oX() - ptdeg.get_oX()) < 1e-6) && (fabs(pt.get_oY() - ptdeg.get_oY()) < 1e-6) && (fabs(pt.get_oZ() - ptdeg.get_oZ()) < 1e-6))){
                degenerate = true;
                break;
              }
            }

            if(!degenerate) {
              nbInliersCur++;
              cur_consensus.push_back(iter);
            }
            else {
              cur_outliers.push_back(iter);
            }
          }
          else {
            cur_outliers.push_back(iter);
          }

          iter++;
        }

        if(nbInliersCur > nbInliers)
        {
          foundSolution = true;
          best_consensus = cur_consensus;
          nbInliers = nbInliersCur;
        }

        nbTrials++;
        
        if(nbTrials >= ransacMaxTrials) {
//          vpERROR_TRACE("Ransac reached the maximum number of trials");
          foundSolution = true;
        }
      }
      else {
        nbTrials++;

        if(nbTrials >= ransacMaxTrials) {
//          vpERROR_TRACE("Ransac reached the maximum number of trials");
        }
      }
    } else {
      nbTrials++;

      if(nbTrials >= ransacMaxTrials) {
//        vpERROR_TRACE("Ransac reached the maximum number of trials");
      }
    }
  }
    
  if(foundSolution) {
//    std::cout << "Nombre d'inliers " << nbInliers << std::endl ;
    
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
    
    //Even if the cardinality of the best consensus set is inferior to ransacNbInlierConsensus,
    //we want to refine the solution with data in best_consensus and return this pose.
    //This is an approach used for example in p118 in Multiple View Geometry in Computer Vision, Hartley, R.~I. and Zisserman, A.
    if(nbInliers >= nbMinRandom) //if(nbInliers >= (unsigned)ransacNbInlierConsensus)
    {
      //Refine the solution using all the points in the consensus set and with VVS pose estimation
      vpPose pose ;
      for(unsigned i = 0 ; i < best_consensus.size(); i++)
      {
        std::list<vpPoint>::const_iterator iter = listP.begin();
        std::advance(iter, best_consensus[i]);
        vpPoint pt = *iter;
      
        pose.addPoint(pt) ;
        ransacInliers.push_back(pt);
      }

      //Update the list of inlier index
      ransacInlierIndex = best_consensus;

      //Flags set if pose computation is OK
      bool is_valid_lagrange = false;
      bool is_valid_dementhon = false;

      //Set maximum value for residuals
      r_lagrange = DBL_MAX;
      r_dementhon = DBL_MAX;

      try {
        pose.computePose(vpPose::LAGRANGE, cMo_lagrange);
        r_lagrange = pose.computeResidual(cMo_lagrange);
        is_valid_lagrange = true;
      } catch(/*vpException &e*/...) {
//        std::cerr << e.what() << std::endl;
      }

      try {
        pose.computePose(vpPose::DEMENTHON, cMo_dementhon);
        r_dementhon = pose.computeResidual(cMo_dementhon);
        is_valid_dementhon = true;
      } catch(/*vpException &e*/...) {
//        std::cerr << e.what() << std::endl;
      }

      //If residual returned is not a number (NAN), set valid to false
      if(vpMath::isNaN(r_lagrange)) {
        is_valid_lagrange = false;
        r_lagrange = DBL_MAX;
      }

      if(vpMath::isNaN(r_dementhon)) {
        is_valid_dementhon = false;
        r_dementhon = DBL_MAX;
      }

      if(is_valid_lagrange || is_valid_dementhon) {
        if (r_lagrange < r_dementhon) {
          cMo = cMo_lagrange;
        }
        else {
          cMo = cMo_dementhon;
        }

        pose.setCovarianceComputation(computeCovariance);
        pose.computePose(vpPose::VIRTUAL_VS, cMo);

        //In some rare cases, the final pose could not respect the pose criterion even
        //if the 4 minimal points picked respect the pose criterion.
        if(func != NULL && !func(&cMo)) {
          return false;
        }

        if(computeCovariance) {
          covarianceMatrix = pose.covarianceMatrix;
        }
      }
    } else {
      return false;
    }
  }

  return foundSolution;
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
      vpPoint pt(p3D[j].getWorldCoordinates());
      pt.set_x(p2D[i].get_x());
      pt.set_y(p2D[i].get_y());
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

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
