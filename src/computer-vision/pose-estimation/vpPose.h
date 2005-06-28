
/*                                                                -*-c++-*-
    Copyright (C) 1998  IRISA-INRIA Rennes Vista Project

    Contact:
       Eric Marchand
       IRISA-INRIA Rennes
       35042 Rennes Cedex
       France

    email: marchand@irisa.fr
    www  : http://www.irisa.fr/vista

*/


/*!
  \file vpPose.h
  \brief Tools for pose computation (pose from point only)
  \ingroup libpose

  \author Eric Marchand (INRIA) using code from Francois Chaumette (INRIA)

  \sa the examlpe in ComputePose.cpp
  \date   April, 6 1999 (first issue)
*/

#ifndef vpPOSE_HH
#define vpPOSE_HH

#include <math.h>
#include <visp/vpHomogeneousMatrix.h>

#include <visp/vpPoint.h>
#include <visp/vpList.h>

/*!
  \class vpPose
  \brief  class used for pose computation from N points (pose from point only)


  \author Eric Marchand (INRIA) using code from Francois Chaumette (INRIA)

  \sa the example in ComputePose.cpp
  \date   April, 6 1999 (first issue)
*/

class vpPose
{

public:
  enum poseMethodEnum
    {
      LAGRANGE         ,
      DEMENTHON        ,
      LOWE             ,
      LAGRANGE_LOWE    ,
      DEMENTHON_LOWE   ,
      VIRTUAL_VS       ,
      DEMENTHON_VIRTUAL_VS,
      LAGRANGE_VIRTUAL_VS
    } vpPoseMethodEnum;
public:
  int npt ;       //!< number of point used in pose computation
  vpList<vpPoint> listP ;     //!< array of point (use here class CPoint)

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
  ~vpPose() ;

  //! compute the residual (i.e., the quality of the result)
  //! compute the residual (in meter for pose M)
  double computeResidual(vpHomogeneousMatrix &cMo) ;
  //! compute the residual (in meter)
  double computeResidual() ;

  //! test the coplanarity of the points
  bool coplanaire() ;

private:
  //! variable used in the Dementhon approach
  vpPoint *c3d ;
protected:
  double computeResidualDementhon(vpHomogeneousMatrix &cMo) ;

  // method used in poseDementhonPlan()
  int calculArbreDementhon(vpMatrix &b, vpColVector &U, vpHomogeneousMatrix &cMo) ;
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
  //! compute the pose using virtual visual servoing approach
  void poseVirtualVS(vpHomogeneousMatrix & cMo) ;
public:
  //! compute the pose for a given method
  void computePose(poseMethodEnum methode, vpHomogeneousMatrix &cMo) ;
  void printPoint() ;
  void display(vpImage<unsigned char> &I, vpHomogeneousMatrix &cMo, vpCameraParameters &cam, double size, int col=vpColor::none) ;


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
} ;



#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
