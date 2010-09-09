/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.GPL at the root directory of this source
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

#include <math.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpHomography.h>
#include <visp/vpPoint.h>
#include <visp/vpList.h>
#include <visp/vpRGBa.h>

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
public:
  int npt ;       //!< number of point used in pose computation
  vpList<vpPoint> listP ;     //!< array of point (use here class vpPoint)

  double residual ;     //!< compute the residual in meter
public:

protected :
  double lambda ;//!< parameters use for the virtual visual servoing approach
public:
  void setLambda(double a) { lambda = a ; }

public:
  void init() ;

  //! suppress all the point in the array of point
  void clearPoint() ;
  //! Add a new point in this array
  void addPoint(const vpPoint& P) ;
  // int AddPoint(const vpPointDot& P) ;

  //! constructor
  vpPose() ;
  //! destructor
  virtual ~vpPose() ;

  //! compute the residual (i.e., the quality of the result)
  //! compute the residual (in meter for pose M)
  double computeResidual(vpHomogeneousMatrix &cMo) ;
  //! compute the residual (in meter)
  double computeResidual() ;

  //! test the coplanarity of the points
  bool coplanaire() ;
  double   distanceToPlaneForCoplanarityTest ;
  void setDistanceToPlaneForCoplanarityTest(double d) ;

private:
  //! variable used in the Dementhon approach
  vpPoint *c3d ; 
protected:
  double computeResidualDementhon(vpHomogeneousMatrix &cMo) ;

  // method used in poseDementhonPlan()
  int calculArbreDementhon(vpMatrix &b, vpColVector &U, vpHomogeneousMatrix &cMo) ;
public:
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
  //! compute the pose for a given method
  void computePose(vpPoseMethodType methode, vpHomogeneousMatrix &cMo) ;
  void printPoint() ;
	
  static void display(vpImage<unsigned char> &I, vpHomogeneousMatrix &cMo,
		      vpCameraParameters &cam, double size,
		      vpColor col=vpColor::none) ;
  static void display(vpImage<vpRGBa> &I, vpHomogeneousMatrix &cMo,
		      vpCameraParameters &cam, double size,
		      vpColor col=vpColor::none) ;
  void displayModel(vpImage<unsigned char> &I, 
		    vpCameraParameters &cam,
		    vpColor col=vpColor::none) ;
  void displayModel(vpImage<vpRGBa> &I, 
		    vpCameraParameters &cam,
		    vpColor col=vpColor::none) ;
  
private:
  int vvsIterMax ; //! define the maximum number of iteration in VVS
public:
  void setVvsIterMax(int nb) { vvsIterMax = nb ; }
public:
  static bool degenerateConfiguration(vpColVector &x,int *ind) ;
  static void computeTransformation(vpColVector &x,int *ind, vpColVector &M) ;
  static double computeResidual(vpColVector &x,  vpColVector &M, vpColVector &d) ;
private:
  static void initRansac(const int n,
			 const double *x, const double *y,
			 const int m,
			 const double *X, const double *Y, const double *Z,
			 vpColVector &data) ;
public:
  static void ransac(const int n,
		     const double *x, const double *y,
		     const int m,
		     const double *X, const double *Y, const double *Z,
		     const int numberOfInlierToReachAConsensus,
		     const double threshold,
		     int &ninliers,
		     vpColVector &xi,  vpColVector &yi,
		     vpColVector &Xi,  vpColVector &Yi,  vpColVector &Zi,
		     vpHomogeneousMatrix &cMo) ;
  static void ransac(const int n,
		     const vpPoint *p,
		     const int m,
		     const vpPoint *P,
		     const int numberOfInlierToReachAConsensus,
		     const double threshold,
		     int &ninliers,
		     vpList<vpPoint> &Pi,
		     vpHomogeneousMatrix &cMo) ;


  static void ransac(vpList<vpPoint> &p,
		     vpList<vpPoint> &P,
		     const int numberOfInlierToReachAConsensus,
		     const double threshold,
		     int &ninliers,
		     vpList<vpPoint> &lPi,
		     vpHomogeneousMatrix &cMo) ;

  static double poseFromRectangle(vpPoint &p1,vpPoint &p2,
				vpPoint &p3,vpPoint &p4,
				double lx, vpCameraParameters & cam,
				vpHomogeneousMatrix & cMo) ;
} ;



#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
