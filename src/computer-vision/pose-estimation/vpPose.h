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
 * Francois Chaumette
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

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpHomography.h>
#include <visp/vpPoint.h>
#include <visp/vpRGBa.h>
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
#  include <visp/vpList.h>
#endif

#include <math.h>
#include <list>

/*!
  \class vpPose
  \ingroup Pose
  \brief Class used for pose computation from N points (pose from point only).


  \author Eric Marchand (INRIA) using code from Francois Chaumette (INRIA)

  \date   April, 6 1999 (first issue)
*/


class VISP_EXPORT vpPose
{  
public:
  typedef enum
    {
      LAGRANGE         ,
      DEMENTHON        ,
      LOWE             ,
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
  vpPoint *c3d ;
  //! Flag used to specify if the covariance matrix has to be computed or not.
  bool computeCovariance;
  //! Covariance matrix
  vpMatrix covarianceMatrix;

protected:
  double computeResidualDementhon(vpHomogeneousMatrix &cMo) ;

  // method used in poseDementhonPlan()
  int calculArbreDementhon(vpMatrix &b, vpColVector &U, vpHomogeneousMatrix &cMo) ;

public:
  //! constructor
  vpPose() ;
  //! destructor
  virtual ~vpPose() ;
  //! Add a new point in this array
  void addPoint(const vpPoint& P) ;
  //! suppress all the point in the array of point
  void clearPoint() ;

  //! compute the pose for a given method
  void computePose(vpPoseMethodType methode, vpHomogeneousMatrix &cMo) ;
  //! compute the residual (i.e., the quality of the result)
  //! compute the residual (in meter for pose M)
  double computeResidual(vpHomogeneousMatrix &cMo) ;
  //! compute the residual (in meter)
  double computeResidual() ;
  //! test the coplanarity of the points
  bool coplanar() ;
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
  void poseLagrangePlan(vpHomogeneousMatrix &cMo) ;
  //! compute the pose using Lagrange approach (non planar object)
  void poseLagrangeNonPlan(vpHomogeneousMatrix &cMo) ;
  //! compute the pose using the Lowe approach (i.e., using the
  //! Levenberg Marquartd non linear minimization approach)
  void poseLowe(vpHomogeneousMatrix & cMo) ;
  //! compute the pose using a robust virtual visual servoing approach
  void poseVirtualVSrobust(vpHomogeneousMatrix & cMo) ;
  //! compute the pose using virtual visual servoing approach
  void poseVirtualVS(vpHomogeneousMatrix & cMo) ;
  void printPoint() ; 
  void setDistanceToPlaneForCoplanarityTest(double d) ;
  void setLambda(double a) { lambda = a ; }
  void setVvsIterMax(int nb) { vvsIterMax = nb ; }
  
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

  static void computeTransformation(vpColVector &x, unsigned int *ind, vpColVector &M) ;
  static double computeResidual(vpColVector &x,  vpColVector &M, vpColVector &d) ;
  static bool degenerateConfiguration(vpColVector &x, unsigned int *ind) ;
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
  static void ransac(const unsigned int n,
                     const double *x, const double *y,
                     const unsigned int m,
                     const double *X, const double *Y, const double *Z,
                     const int numberOfInlierToReachAConsensus,
                     const double threshold,
                     unsigned int &ninliers,
                     vpColVector &xi,  vpColVector &yi,
                     vpColVector &Xi,  vpColVector &Yi,  vpColVector &Zi,
                     vpHomogeneousMatrix &cMo, const int maxNbTrials = 10000) ;
  static void ransac(const unsigned int n,
                     const vpPoint *p,
                     const unsigned int m,
                     const vpPoint *P,
                     const int numberOfInlierToReachAConsensus,
                     const double threshold,
                     unsigned int &ninliers,
                     std::list<vpPoint> &Pi,
                     vpHomogeneousMatrix &cMo, const int maxNbTrials = 10000) ;

  static void ransac(std::list<vpPoint> &p,
                     std::list<vpPoint> &P,
                     const int numberOfInlierToReachAConsensus,
                     const double threshold,
                     unsigned int &ninliers,
                     std::list<vpPoint> &lPi,
                     vpHomogeneousMatrix &cMo, const int maxNbTrials = 10000) ;

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  /*!
    @name Deprecated functions
  */
  vp_deprecated  bool coplanaire() ;
  vp_deprecated  static void ransac(const unsigned int n,
         const vpPoint *p,
         const unsigned int m,
         const vpPoint *P,
         const int numberOfInlierToReachAConsensus,
         const double threshold,
         unsigned int &ninliers,
         vpList<vpPoint> &Pi,
         vpHomogeneousMatrix &cMo, const int maxNbTrials = 10000) ;

  vp_deprecated static void ransac(vpList<vpPoint> &p,
         vpList<vpPoint> &P,
         const int numberOfInlierToReachAConsensus,
         const double threshold,
         unsigned int &ninliers,
         vpList<vpPoint> &lPi,
         vpHomogeneousMatrix &cMo, const int maxNbTrials = 10000) ;
#endif

private:
  static void initRansac(const unsigned int n,
       const double *x, const double *y,
       const unsigned int m,
       const double *X, const double *Y, const double *Z,
       vpColVector &data) ;
} ;



#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
