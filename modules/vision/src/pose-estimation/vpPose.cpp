/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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
 *
 *****************************************************************************/

/*!
\file vpPose.cpp
\brief Fichier contenant la classe vpPose (tout ce qui est necessaire
pour faire du calcul de pose par difference methode
*/

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/vision/vpPose.h>
#include <visp3/vision/vpPoseException.h>

#include <cmath>  // std::fabs
#include <limits> // numeric_limits

#define DEBUG_LEVEL1 0
/*!
  Basic initialisation that is called by the constructors.
*/
void vpPose::init()
{
#if (DEBUG_LEVEL1)
  std::cout << "begin vpPose::Init() " << std::endl;
#endif

  npt = 0;
  listP.clear();
  residual = 0;
  lambda = 1.;
  vvsIterMax = 200;
  c3d.clear();
  computeCovariance = false;
  covarianceMatrix.clear();
  ransacNbInlierConsensus = 4;
  ransacMaxTrials = 1000;
  ransacInliers.clear();
  ransacInlierIndex.clear();
  ransacThreshold = 0.0001;
  distanceToPlaneForCoplanarityTest = 0.001;
  ransacFlag = NO_FILTER;
  listOfPoints.clear();
  useParallelRansac = false;
  nbParallelRansacThreads = 0;
  vvsEpsilon = 1e-8;

#if (DEBUG_LEVEL1)
  std::cout << "end vpPose::Init() " << std::endl;
#endif
}

/*! Default constructor. */
vpPose::vpPose()
  : npt(0), listP(), residual(0), lambda(0.25), vvsIterMax(200), c3d(), computeCovariance(false), covarianceMatrix(),
    ransacNbInlierConsensus(4), ransacMaxTrials(1000), ransacInliers(), ransacInlierIndex(), ransacThreshold(0.0001),
    distanceToPlaneForCoplanarityTest(0.001), ransacFlag(vpPose::NO_FILTER), listOfPoints(),
    useParallelRansac(false),
    nbParallelRansacThreads(0), // 0 means that we use C++11 (if available) to get the number of threads
    vvsEpsilon(1e-8)
{
}

/*!
  Destructor that deletes the array of point (freed the memory).
*/
vpPose::~vpPose()
{
#if (DEBUG_LEVEL1)
  std::cout << "begin vpPose::~vpPose() " << std::endl;
#endif

  listP.clear();

#if (DEBUG_LEVEL1)
  std::cout << "end vpPose::~vpPose() " << std::endl;
#endif
}
/*!
  Delete the array of point
*/
void vpPose::clearPoint()
{
  listP.clear();
  listOfPoints.clear();
  npt = 0;
}

/*!
  Add a new point in the array of points.
  \param  newP : New point to add  in the array of point.
  \warning Considering a point from the class vpPoint, oX, oY, and oZ will
  represent the 3D coordinates of the point in the object frame and x and y
  its 2D coordinates in the image plane. These 5 fields must be initialized
  to be used within this function.
*/
void vpPose::addPoint(const vpPoint &newP)
{
  listP.push_back(newP);
  listOfPoints.push_back(newP);
  npt++;
}

/*!
  Add (append) a list of points in the array of points.
  \param  lP : List of points to add (append).
  \warning Considering a point from the class vpPoint, oX, oY, and oZ will
  represent the 3D coordinates of the point in the object frame and x and y
  its 2D coordinates in the image plane. These 5 fields must be initialized
  to be used within this function.
*/
void vpPose::addPoints(const std::vector<vpPoint> &lP)
{
  listP.insert(listP.end(), lP.begin(), lP.end());
  listOfPoints.insert(listOfPoints.end(), lP.begin(), lP.end());
  npt = (unsigned int)listP.size();
}

void vpPose::setDistanceToPlaneForCoplanarityTest(double d) { distanceToPlaneForCoplanarityTest = d; }

/*!
  Test the coplanarity of the set of points

  \param coplanar_plane_type:
   1: if plane x=cst
   2: if plane y=cst
   3: if plane z=cst
   4: if the points are collinear.
   0: any other plane
  \return true if points are coplanar false otherwise.
*/
bool vpPose::coplanar(int &coplanar_plane_type)
{
  coplanar_plane_type = 0;
  if (npt < 2) {
    vpERROR_TRACE("Not enough point (%d) to compute the pose  ", npt);
    throw(vpPoseException(vpPoseException::notEnoughPointError, "Not enough points "));
  }

  if (npt == 3)
    return true;

  double x1 = 0, x2 = 0, x3 = 0, y1 = 0, y2 = 0, y3 = 0, z1 = 0, z2 = 0, z3 = 0;

  std::list<vpPoint>::const_iterator it = listP.begin();

  vpPoint P1, P2, P3;

  // Get three 3D points that are not collinear and that is not at origin
  bool degenerate = true;
  bool not_on_origin = true;
  std::list<vpPoint>::const_iterator it_tmp;

  std::list<vpPoint>::const_iterator it_i, it_j, it_k;
  for (it_i = listP.begin(); it_i != listP.end(); ++it_i) {
    if (degenerate == false) {
      // std::cout << "Found a non degenerate configuration" << std::endl;
      break;
    }
    P1 = *it_i;
    // Test if point is on origin
    if ((std::fabs(P1.get_oX()) <= std::numeric_limits<double>::epsilon()) &&
        (std::fabs(P1.get_oY()) <= std::numeric_limits<double>::epsilon()) &&
        (std::fabs(P1.get_oZ()) <= std::numeric_limits<double>::epsilon())) {
      not_on_origin = false;
    } else {
      not_on_origin = true;
    }
    if (not_on_origin) {
      it_tmp = it_i;
      ++it_tmp; // j = i+1
      for (it_j = it_tmp; it_j != listP.end(); ++it_j) {
        if (degenerate == false) {
          // std::cout << "Found a non degenerate configuration" << std::endl;
          break;
        }
        P2 = *it_j;
        if ((std::fabs(P2.get_oX()) <= std::numeric_limits<double>::epsilon()) &&
            (std::fabs(P2.get_oY()) <= std::numeric_limits<double>::epsilon()) &&
            (std::fabs(P2.get_oZ()) <= std::numeric_limits<double>::epsilon())) {
          not_on_origin = false;
        } else {
          not_on_origin = true;
        }
        if (not_on_origin) {
          it_tmp = it_j;
          ++it_tmp; // k = j+1
          for (it_k = it_tmp; it_k != listP.end(); ++it_k) {
            P3 = *it_k;
            if ((std::fabs(P3.get_oX()) <= std::numeric_limits<double>::epsilon()) &&
                (std::fabs(P3.get_oY()) <= std::numeric_limits<double>::epsilon()) &&
                (std::fabs(P3.get_oZ()) <= std::numeric_limits<double>::epsilon())) {
              not_on_origin = false;
            } else {
              not_on_origin = true;
            }
            if (not_on_origin) {
              x1 = P1.get_oX();
              x2 = P2.get_oX();
              x3 = P3.get_oX();

              y1 = P1.get_oY();
              y2 = P2.get_oY();
              y3 = P3.get_oY();

              z1 = P1.get_oZ();
              z2 = P2.get_oZ();
              z3 = P3.get_oZ();

              vpColVector a_b(3), b_c(3), cross_prod;
              a_b[0] = x1 - x2;
              a_b[1] = y1 - y2;
              a_b[2] = z1 - z2;
              b_c[0] = x2 - x3;
              b_c[1] = y2 - y3;
              b_c[2] = z2 - z3;

              cross_prod = vpColVector::crossProd(a_b, b_c);
              if (cross_prod.sumSquare() <= std::numeric_limits<double>::epsilon())
                degenerate = true; // points are collinear
              else
                degenerate = false;
            }
            if (degenerate == false)
              break;
          }
        }
      }
    }
  }

  if (degenerate) {
    coplanar_plane_type = 4; // points are collinear
    return true;
  }

  double a = y1 * z2 - y1 * z3 - y2 * z1 + y2 * z3 + y3 * z1 - y3 * z2;
  double b = -x1 * z2 + x1 * z3 + x2 * z1 - x2 * z3 - x3 * z1 + x3 * z2;
  double c = x1 * y2 - x1 * y3 - x2 * y1 + x2 * y3 + x3 * y1 - x3 * y2;
  double d = -x1 * y2 * z3 + x1 * y3 * z2 + x2 * y1 * z3 - x2 * y3 * z1 - x3 * y1 * z2 + x3 * y2 * z1;

  // std::cout << "a=" << a << " b=" << b << " c=" << c << " d=" << d <<
  // std::endl;
  if (std::fabs(b) <= std::numeric_limits<double>::epsilon() &&
      std::fabs(c) <= std::numeric_limits<double>::epsilon()) {
    coplanar_plane_type = 1; // ax=d
  } else if (std::fabs(a) <= std::numeric_limits<double>::epsilon() &&
             std::fabs(c) <= std::numeric_limits<double>::epsilon()) {
    coplanar_plane_type = 2; // by=d
  } else if (std::fabs(a) <= std::numeric_limits<double>::epsilon() &&
             std::fabs(b) <= std::numeric_limits<double>::epsilon()) {
    coplanar_plane_type = 3; // cz=d
  }

  double D = sqrt(vpMath::sqr(a) + vpMath::sqr(b) + vpMath::sqr(c));

  for (it = listP.begin(); it != listP.end(); ++it) {
    P1 = *it;
    double dist = (a * P1.get_oX() + b * P1.get_oY() + c * P1.get_oZ() + d) / D;
    // std::cout << "dist= " << dist << std::endl;

    if (fabs(dist) > distanceToPlaneForCoplanarityTest) {
      vpDEBUG_TRACE(10, " points are not coplanar ");
      //	TRACE(" points are not coplanar ") ;
      return false;
    }
  }

  vpDEBUG_TRACE(10, " points are  coplanar ");
  //  vpTRACE(" points are  coplanar ") ;

  return true;
}

/*!
  \brief Compute and return the sum of squared residuals expressed in meter^2 for
  the pose matrix \e cMo.

  \param cMo : Input pose. The matrix that defines the pose to be tested.

  \return The value of the sum of squared residuals in meter^2.

*/
double vpPose::computeResidual(const vpHomogeneousMatrix &cMo) const
{
  double squared_error = 0;
  vpPoint P;
  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it) {
    P = *it;
    double x = P.get_x();
    double y = P.get_y();

    P.track(cMo);

    squared_error += vpMath::sqr(x - P.get_x()) + vpMath::sqr(y - P.get_y());
  }
  return (squared_error);
}

/*!
  Compute the pose according to the desired method which are:
  - vpPose::LAGRANGE: Linear Lagrange approach (test is done to switch between
  planar and non planar algorithm)
  - vpPose::DEMENTHON: Linear Dementhon approach (test is done to switch
  between planar and non planar algorithm)
  - vpPose::LOWE: Lowe aproach based on a Levenberg Marquartd non linear
  minimization scheme that needs an initialization from Lagrange or Dementhon
  aproach
  - vpPose::LAGRANGE_LOWE: Non linear Lowe aproach initialized by Lagrange
  approach
  - vpPose::DEMENTHON_LOWE: Non linear Lowe aproach initialized by Dementhon
  approach
  - vpPose::VIRTUAL_VS: Non linear virtual visual servoing approach that needs
  an initialization from Lagrange or Dementhon aproach
  - vpPose::DEMENTHON_VIRTUAL_VS: Non linear virtual visual servoing approach
  initialized by Dementhon approach
  - vpPose::LAGRANGE_VIRTUAL_VS: Non linear virtual visual servoing approach
  initialized by Lagrange approach
  - vpPose::RANSAC: Robust Ransac aproach (doesn't need an initialization)

*/
bool vpPose::computePose(vpPoseMethodType method, vpHomogeneousMatrix &cMo, bool (*func)(const vpHomogeneousMatrix &))
{
  if (npt < 4) {
    throw(vpPoseException(vpPoseException::notEnoughPointError,
                          "Not enough point (%d) to compute the pose  ", npt));
  }

  switch (method) {
  case DEMENTHON:
  case DEMENTHON_VIRTUAL_VS:
  case DEMENTHON_LOWE: {
    if (npt < 4) {
      throw(vpPoseException(vpPoseException::notEnoughPointError,
                            "Dementhon method cannot be used in that case "
                            "(at least 4 points are required)"
                            "Not enough point (%d) to compute the pose  ", npt));
    }

    // test si les point 3D sont coplanaires
    int coplanar_plane_type = 0;
    bool plan = coplanar(coplanar_plane_type);
    if (plan == true) {
      poseDementhonPlan(cMo);
    } else {
      poseDementhonNonPlan(cMo);
    }
  } break;
  case LAGRANGE:
  case LAGRANGE_VIRTUAL_VS:
  case LAGRANGE_LOWE: {
    // test si les point 3D sont coplanaires
    int coplanar_plane_type;
    bool plan = coplanar(coplanar_plane_type);

    if (plan == true)
    {

      if (coplanar_plane_type == 4) {
        throw(vpPoseException(vpPoseException::notEnoughPointError,
                              "Lagrange method cannot be used in that case "
                              "(points are collinear)"));
      }
      if (npt < 4) {
        throw(vpPoseException(vpPoseException::notEnoughPointError,
                              "Lagrange method cannot be used in that case "
                              "(at least 4 points are required). "
                              "Not enough point (%d) to compute the pose  ", npt));
      }
      poseLagrangePlan(cMo);
    } else {
      if (npt < 6) {
        throw(vpPoseException(vpPoseException::notEnoughPointError,
                              "Lagrange method cannot be used in that case "
                              "(at least 6 points are required when 3D points are non coplanar). "
                              "Not enough point (%d) to compute the pose  ", npt));
      }
      poseLagrangeNonPlan(cMo);
    }
  } break;
  case RANSAC:
    if (npt < 4) {
      throw(vpPoseException(vpPoseException::notEnoughPointError,
                            "Ransac method cannot be used in that case "
                            "(at least 4 points are required). "
                            "Not enough point (%d) to compute the pose  ", npt));
    }
      return poseRansac(cMo, func);
    break;
  case LOWE:
  case VIRTUAL_VS:
    break;
  }

  switch (method) {
  case LAGRANGE:
  case DEMENTHON:
  case RANSAC:
    break;
  case VIRTUAL_VS:
  case LAGRANGE_VIRTUAL_VS:
  case DEMENTHON_VIRTUAL_VS: {
    poseVirtualVS(cMo);
  } break;
  case LOWE:
  case LAGRANGE_LOWE:
  case DEMENTHON_LOWE: {
    poseLowe(cMo);
  } break;
  }

  // If here, there was no exception thrown so return true
  return true;
}

void vpPose::printPoint()
{
  vpPoint P;
  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it) {
    P = *it;

    std::cout << "3D oP " << P.oP.t();
    std::cout << "3D cP " << P.cP.t();
    std::cout << "2D    " << P.p.t();
  }
}

/*!
   Display in the image \e I the pose represented by its homogenous
   transformation \e cMo as a 3 axis frame. \param I: Image where the pose is
   displayed in overlay. \param cMo: Considered pose to display. \param cam:
   Camera parameters associated to image \e I. \param size: length in meter of
   the axis that will be displayed \param col: Color used to display the 3
   axis. If vpColor::none, red, green and blue will represent x-axiw, y-axis
   and z-axis respectively.
 */
void vpPose::display(vpImage<unsigned char> &I, vpHomogeneousMatrix &cMo, vpCameraParameters &cam, double size,
                     vpColor col)
{
  vpDisplay::displayFrame(I, cMo, cam, size, col);
}

/*!
   Display in the image \e I the pose represented by its homogenous
   transformation \e cMo as a 3 axis frame. \param I: Image where the pose is
   displayed in overlay. \param cMo: Considered pose to display. \param cam:
   Camera parameters associated to image \e I. \param size: length in meter of
   the axis that will be displayed \param col: Color used to display the 3
   axis. If vpColor::none, red, green and blue will represent x-axiw, y-axis
   and z-axis respectively.
 */
void vpPose::display(vpImage<vpRGBa> &I, vpHomogeneousMatrix &cMo, vpCameraParameters &cam, double size, vpColor col)
{
  vpDisplay::displayFrame(I, cMo, cam, size, col);
}

/*!
  Display the coordinates of the points in the image plane that are used to
  compute the pose in image I.
*/
void vpPose::displayModel(vpImage<unsigned char> &I, vpCameraParameters &cam, vpColor col)
{
  vpPoint P;
  vpImagePoint ip;
  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it) {
    P = *it;
    vpMeterPixelConversion::convertPoint(cam, P.p[0], P.p[1], ip);
    vpDisplay::displayCross(I, ip, 5, col);
    //  std::cout << "3D oP " << P.oP.t() ;
    //  std::cout << "3D cP " << P.cP.t() ;
    //  std::cout << "2D    " << P.p.t() ;
  }
}

/*!
  Display the coordinates of the points in the image plane that are used to
  compute the pose in image I.
*/
void vpPose::displayModel(vpImage<vpRGBa> &I, vpCameraParameters &cam, vpColor col)
{
  vpPoint P;
  vpImagePoint ip;
  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it) {
    P = *it;
    vpMeterPixelConversion::convertPoint(cam, P.p[0], P.p[1], ip);
    vpDisplay::displayCross(I, ip, 5, col);
    //  std::cout << "3D oP " << P.oP.t() ;
    //  std::cout << "3D cP " << P.cP.t() ;
    //  std::cout << "2D    " << P.p.t() ;
  }
}

/*!
\brief Carries out the camera pose the image of a rectangle and
the intrinsec parameters, the length on x axis is known but the
proprtion of the rectangle are unknown.

This method is taken from "Markerless Tracking using Planar Structures
in the Scene" by Gilles Simon. The idea is to compute the homography H
giving the image point of the rectangle by associating them with the
coordinates (0,0)(1,0)(1,1/s)(0,1/s) (the rectangle is on the Z=0 plane).
If K is the intrinsec parameters matrix, we have  s = ||Kh1||/ ||Kh2||. s
gives us the proportion of the rectangle

\param p1,p2,p3,p4: the image of the corners of the rectangle
(respectively the image of  (0,0),(lx,0),(lx,lx/s) and (0,lx/s)) (input)
\param cam: the camera used (input)
\param lx: the rectangle size on the x axis (input)
\param cMo: the camera pose (output)
\return int : OK if no pb occurs
*/
double vpPose::poseFromRectangle(vpPoint &p1, vpPoint &p2, vpPoint &p3, vpPoint &p4, double lx, vpCameraParameters &cam,
                                 vpHomogeneousMatrix &cMo)
{

  std::vector<double> rectx(4);
  std::vector<double> recty(4);
  rectx[0] = 0;
  recty[0] = 0;
  rectx[1] = 1;
  recty[1] = 0;
  rectx[2] = 1;
  recty[2] = 1;
  rectx[3] = 0;
  recty[3] = 1;
  std::vector<double> irectx(4);
  std::vector<double> irecty(4);
  irectx[0] = (p1.get_x());
  irecty[0] = (p1.get_y());
  irectx[1] = (p2.get_x());
  irecty[1] = (p2.get_y());
  irectx[2] = (p3.get_x());
  irecty[2] = (p3.get_y());
  irectx[3] = (p4.get_x());
  irecty[3] = (p4.get_y());

  // calcul de l'homographie
  vpMatrix H(3, 3);
  vpHomography hom;

  //  vpHomography::HartleyDLT(rectx,recty,irectx,irecty,hom);
  vpHomography::HLM(rectx, recty, irectx, irecty, 1, hom);
  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++)
      H[i][j] = hom[i][j];
  // calcul de s =  ||Kh1||/ ||Kh2|| =ratio (length on x axis/ length on y
  // axis)
  vpColVector kh1(3);
  vpColVector kh2(3);
  vpMatrix K(3, 3);
  K = cam.get_K();
  K.eye();
  vpMatrix Kinv = K.pseudoInverse();

  vpMatrix KinvH = Kinv * H;
  kh1 = KinvH.getCol(0);
  kh2 = KinvH.getCol(1);

  double s = sqrt(kh1.sumSquare()) / sqrt(kh2.sumSquare());

  vpMatrix D(3, 3);
  D.eye();
  D[1][1] = 1 / s;
  vpMatrix cHo = H * D;

  // Calcul de la rotation et de la translation
  //  PoseFromRectangle(p1,p2,p3,p4,1/s,lx,cam,cMo );
  p1.setWorldCoordinates(0, 0, 0);
  p2.setWorldCoordinates(lx, 0, 0);
  p3.setWorldCoordinates(lx, lx / s, 0);
  p4.setWorldCoordinates(0, lx / s, 0);

  vpPose P;
  P.addPoint(p1);
  P.addPoint(p2);
  P.addPoint(p3);
  P.addPoint(p4);

  P.computePose(vpPose::DEMENTHON_LOWE, cMo);
  return lx / s;
}
