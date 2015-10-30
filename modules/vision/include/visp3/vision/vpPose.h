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
 * Francois Chaumette
 * Aurelien Yol
 *
 *****************************************************************************/

/*!
  \file vpPose.h
  \brief Tools for pose computation (pose from point only).

  \author Eric Marchand (INRIA) using code from Francois Chaumette (INRIA)
  \date   April, 6 1999 (first issue)
*/

#ifndef vpPOSE_HH
#define vpPOSE_HH

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/vision/vpHomography.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpRGBa.h>
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
#  include <visp3/core/vpList.h>
#endif

#include <math.h>
#include <list>
#include <vector>

/*!
  \class vpPose
  \ingroup group_vision_pose
  \brief Class used for pose computation from N points (pose from point only).

  \note It is also possible to estimate a pose from other features using vpPoseFeatures class.

  To see how to use this class you can follow the \ref tutorial-pose-estimation.
*/


class VISP_EXPORT vpPose
{  
public:
  typedef enum
    {
      LAGRANGE         ,
      DEMENTHON        ,
      LOWE             ,
      RANSAC           ,
      LAGRANGE_LOWE    ,
      DEMENTHON_LOWE   ,
      VIRTUAL_VS       ,
      DEMENTHON_VIRTUAL_VS,
      LAGRANGE_VIRTUAL_VS
    } vpPoseMethodType;

  unsigned int npt ;       //!< number of point used in pose computation
  std::list<vpPoint> listP ;     //!< array of point (use here class vpPoint)

  double residual ;     //!< compute the residual in meter

protected :
  double lambda ;//!< parameters use for the virtual visual servoing approach

private:
  int vvsIterMax ; //! define the maximum number of iteration in VVS
  //! variable used in the Dementhon approach
  std::vector<vpPoint> c3d ;
  //! Flag used to specify if the covariance matrix has to be computed or not.
  bool computeCovariance;
  //! Covariance matrix
  vpMatrix covarianceMatrix;
  
  unsigned int ransacNbInlierConsensus;
  int ransacMaxTrials;
  std::vector<vpPoint> ransacInliers;
  std::vector<unsigned int> ransacInlierIndex;
  double ransacThreshold;

protected:
  double computeResidualDementhon(const vpHomogeneousMatrix &cMo) ;

  // method used in poseDementhonPlan()
  int calculArbreDementhon(vpMatrix &b, vpColVector &U, vpHomogeneousMatrix &cMo) ;

public:
  // constructor
  vpPose() ;
  //! destructor
  virtual ~vpPose() ;
  //! Add a new point in this array
  void addPoint(const vpPoint& P) ;
  //! suppress all the point in the array of point
  void clearPoint() ;

  //! compute the pose for a given method
  bool computePose(vpPoseMethodType methode, vpHomogeneousMatrix &cMo, bool (*func)(vpHomogeneousMatrix *)=NULL) ;
  //! compute the residual (i.e., the quality of the result)
  //! compute the residual (in meter for pose M)
  double computeResidual(const vpHomogeneousMatrix &cMo) const ;
  //! test the coplanarity of the points
  bool coplanar(int &coplanar_plane_type) ;
  void displayModel(vpImage<unsigned char> &I,
                    vpCameraParameters &cam,
                    vpColor col=vpColor::none) ;
  void displayModel(vpImage<vpRGBa> &I,
                    vpCameraParameters &cam,
                    vpColor col=vpColor::none) ;
  double distanceToPlaneForCoplanarityTest ;
  void init() ;
  //! compute the pose using Dementhon approach (planar object)
  void poseDementhonPlan(vpHomogeneousMatrix &cMo) ;
  //! compute the pose using Dementhon approach (non planar object)
  void poseDementhonNonPlan(vpHomogeneousMatrix &cMo) ;
  //! compute the pose using Lagrange approach (planar object)
  void poseLagrangePlan(vpHomogeneousMatrix &cMo, const int coplanar_plane_type=0) ;
  //! compute the pose using Lagrange approach (non planar object)
  void poseLagrangeNonPlan(vpHomogeneousMatrix &cMo) ;
  //! compute the pose using the Lowe approach (i.e., using the
  //! Levenberg Marquartd non linear minimization approach)
  void poseLowe(vpHomogeneousMatrix & cMo) ;
  //! compute the pose using the Ransac approach 
  bool poseRansac(vpHomogeneousMatrix & cMo, bool (*func)(vpHomogeneousMatrix *)=NULL) ;
  //! compute the pose using a robust virtual visual servoing approach
  void poseVirtualVSrobust(vpHomogeneousMatrix & cMo) ;
  //! compute the pose using virtual visual servoing approach
  void poseVirtualVS(vpHomogeneousMatrix & cMo) ;
  void printPoint() ; 
  void setDistanceToPlaneForCoplanarityTest(double d) ;
  void setLambda(double a) { lambda = a ; }
  void setVvsIterMax(int nb) { vvsIterMax = nb ; }
  
  void setRansacNbInliersToReachConsensus(const unsigned int &nbC){ ransacNbInlierConsensus = nbC; }
  void setRansacThreshold(const double &t) {
    //Test whether or not t is > 0
    if(t > 0) {
      ransacThreshold = t;
    } else {
      throw vpException(vpException::badValue, "The Ransac threshold must be positive as we deal with distance.");
    }
  }
  void setRansacMaxTrials(const int &rM){ ransacMaxTrials = rM; }
  unsigned int getRansacNbInliers() const { return (unsigned int) ransacInliers.size(); }
  std::vector<unsigned int> getRansacInlierIndex() const{ return ransacInlierIndex; }
  std::vector<vpPoint> getRansacInliers() const{ return ransacInliers; }
  
  /*!
    Set if the covaraince matrix has to be computed in the Virtual Visual Servoing approach.

    \param flag : True if the covariance has to be computed, false otherwise.
  */
  void setCovarianceComputation(const bool& flag) { computeCovariance = flag; }
  
  /*!
    Get the covariance matrix computed in the Virtual Visual Servoing approach.
    
    \warning The compute covariance flag has to be true if you want to compute the covariance matrix.
    
    \sa setCovarianceComputation
  */
  vpMatrix getCovarianceMatrix() const { 
    if(!computeCovariance)
      vpTRACE("Warning : The covariance matrix has not been computed. See setCovarianceComputation() to do it.");
    
    return covarianceMatrix; 
  }
  
  static void display(vpImage<unsigned char> &I, vpHomogeneousMatrix &cMo,
                      vpCameraParameters &cam, double size,
                      vpColor col=vpColor::none) ;
  static void display(vpImage<vpRGBa> &I, vpHomogeneousMatrix &cMo,
                      vpCameraParameters &cam, double size,
                      vpColor col=vpColor::none) ;
  static double poseFromRectangle(vpPoint &p1,vpPoint &p2,
                                  vpPoint &p3,vpPoint &p4,
                                  double lx, vpCameraParameters & cam,
                                  vpHomogeneousMatrix & cMo) ;
                     
  static void findMatch(std::vector<vpPoint> &p2D, 
                     std::vector<vpPoint> &p3D, 
                     const unsigned int &numberOfInlierToReachAConsensus,
                     const double &threshold,
                     unsigned int &ninliers,
                     std::vector<vpPoint> &listInliers,
                     vpHomogeneousMatrix &cMo,
                     const int &maxNbTrials = 10000);
} ;


#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
