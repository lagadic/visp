/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 */

/*!
  \file vpPose.cpp
  \brief File containing the vpPose class, everything needed to calculate poses using different methods.
*/

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpUniRand.h>
#include <visp3/vision/vpPose.h>
#include <visp3/vision/vpPoseException.h>
#ifdef VISP_HAVE_HOMOGRAPHY
#include <visp3/vision/vpHomography.h>
#endif

#include <cmath>  // std::fabs
#include <limits> // numeric_limits

BEGIN_VISP_NAMESPACE

#ifndef DOXYGEN_SHOULD_SKIP_THIS
namespace
{
const int def_vvsIterMax = 200;
const unsigned int def_ransacNbInlier = 4;
const int def_ransacMaxTrials = 1000;
}
#endif // DOXYGEN_SHOULD_SKIP_THIS

vpPose::vpPose()
  : npt(0), listP(), residual(0), m_lambda(0.9), m_dementhonSvThresh(1e-6), vvsIterMax(def_vvsIterMax), c3d(),
  computeCovariance(false), covarianceMatrix(),
  ransacNbInlierConsensus(def_ransacNbInlier), ransacMaxTrials(def_ransacMaxTrials), ransacInliers(), ransacInlierIndex(), ransacThreshold(0.0001),
  distToPlaneForCoplanarityTest(0.001), ransacFlag(vpPose::NO_FILTER), listOfPoints(), useParallelRansac(false),
  nbParallelRansacThreads(0), // 0 means that we use C++11 (if available) to get the number of threads
  vvsEpsilon(1e-8)
{ }

vpPose::vpPose(const std::vector<vpPoint> &lP)
  : npt(static_cast<unsigned int>(lP.size())), listP(lP.begin(), lP.end()), residual(0), m_lambda(0.9),
  m_dementhonSvThresh(1e-6), vvsIterMax(def_vvsIterMax),
  c3d(), computeCovariance(false), covarianceMatrix(), ransacNbInlierConsensus(def_ransacNbInlier), ransacMaxTrials(def_ransacMaxTrials),
  ransacInliers(), ransacInlierIndex(), ransacThreshold(0.0001), distToPlaneForCoplanarityTest(0.001),
  ransacFlag(vpPose::NO_FILTER), listOfPoints(lP), useParallelRansac(false),
  nbParallelRansacThreads(0), // 0 means that we use C++11 (if available) to get the number of threads
  vvsEpsilon(1e-8)
{ }

vpPose::~vpPose()
{
  listP.clear();
}

void vpPose::clearPoint()
{
  listP.clear();
  listOfPoints.clear();
  npt = 0;
}

void vpPose::addPoint(const vpPoint &newP)
{
  listP.push_back(newP);
  listOfPoints.push_back(newP);
  ++npt;
}

void vpPose::addPoints(const std::vector<vpPoint> &lP)
{
  listP.insert(listP.end(), lP.begin(), lP.end());
  listOfPoints.insert(listOfPoints.end(), lP.begin(), lP.end());
  npt = static_cast<unsigned int>(listP.size());
}

void vpPose::setDistToPlaneForCoplanTest(double d) { distToPlaneForCoplanarityTest = d; }

void vpPose::setDementhonSvThreshold(const double &svThresh)
{
  if (svThresh < 0) {
    throw vpException(vpException::badValue, "The svd threshold must be positive");
  }
  m_dementhonSvThresh = svThresh;
}

bool vpPose::coplanar(int &coplanar_plane_type, double *p_a, double *p_b, double *p_c, double *p_d)
{
  coplanar_plane_type = 0;
  const unsigned int nbMinPt = 2;
  if (npt < nbMinPt) {
    throw(vpPoseException(vpPoseException::notEnoughPointError, "Not enough point (%d) to compute the pose  ", npt));
  }

  const unsigned int nbPtPlan = 3;
  if (npt == nbPtPlan) {
    return true;
  }

  // Shuffling the points to limit the risk of using points close to each other
  std::vector<vpPoint> shuffled_listP = vpUniRand::shuffleVector<vpPoint>(listOfPoints);

  double x1 = 0, x2 = 0, x3 = 0, y1 = 0, y2 = 0, y3 = 0, z1 = 0, z2 = 0, z3 = 0;

  std::vector<vpPoint>::const_iterator it = shuffled_listP.begin();

  vpPoint P1, P2, P3;

  // Get three 3D points that are not collinear and that is not at origin
  bool degenerate = true;
  bool not_on_origin = true;
  std::vector<vpPoint>::const_iterator it_tmp;

  std::vector<vpPoint>::const_iterator it_i, it_j, it_k;
  std::vector<vpPoint>::const_iterator shuffled_listp_end = shuffled_listP.end();
  for (it_i = shuffled_listP.begin(); it_i != shuffled_listp_end; ++it_i) {
    if (degenerate == false) {
      // --comment: print "Found a non degenerate configuration"
      break;
    }
    P1 = *it_i;
    // Test if point is on origin
    if ((std::fabs(P1.get_oX()) <= std::numeric_limits<double>::epsilon()) &&
      (std::fabs(P1.get_oY()) <= std::numeric_limits<double>::epsilon()) &&
      (std::fabs(P1.get_oZ()) <= std::numeric_limits<double>::epsilon())) {
      not_on_origin = false;
    }
    else {
      not_on_origin = true;
    }
    if (not_on_origin) {
      it_tmp = it_i;
      ++it_tmp; // j = i+1
      std::vector<vpPoint>::const_iterator shuffled_listp_end_1 = shuffled_listP.end();
      for (it_j = it_tmp; it_j != shuffled_listp_end_1; ++it_j) {
        if (degenerate == false) {
          // --comment: cout "Found a non degenerate configuration"
          break;
        }
        P2 = *it_j;
        if ((std::fabs(P2.get_oX()) <= std::numeric_limits<double>::epsilon()) &&
          (std::fabs(P2.get_oY()) <= std::numeric_limits<double>::epsilon()) &&
          (std::fabs(P2.get_oZ()) <= std::numeric_limits<double>::epsilon())) {
          not_on_origin = false;
        }
        else {
          not_on_origin = true;
        }
        if (not_on_origin) {
          it_tmp = it_j;
          ++it_tmp; // k = j+1
          std::vector<vpPoint>::const_iterator shuffled_listp_end_2 = shuffled_listP.end();
          for (it_k = it_tmp; it_k != shuffled_listp_end_2; ++it_k) {
            P3 = *it_k;
            if ((std::fabs(P3.get_oX()) <= std::numeric_limits<double>::epsilon()) &&
              (std::fabs(P3.get_oY()) <= std::numeric_limits<double>::epsilon()) &&
              (std::fabs(P3.get_oZ()) <= std::numeric_limits<double>::epsilon())) {
              not_on_origin = false;
            }
            else {
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
              if (cross_prod.sumSquare() <= std::numeric_limits<double>::epsilon()) {
                degenerate = true; // points are collinear
              }
              else {
                degenerate = false;
              }
            }
            if (degenerate == false) {
              break;
            }
          }
        }
      }
    }
  }

  if (degenerate) {
    const int typeCollinear = 4;
    coplanar_plane_type = typeCollinear; // points are collinear
    return true;
  }

  double a = (((y1 * z2) - (y1 * z3) - (y2 * z1)) + (y2 * z3) + (y3 * z1)) - (y3 * z2);
  double b = ((((-x1 * z2) + (x1 * z3) + (x2 * z1)) - (x2 * z3)) - (x3 * z1)) + (x3 * z2);
  double c = (((x1 * y2) - (x1 * y3) - (x2 * y1)) + (x2 * y3) + (x3 * y1)) - (x3 * y2);
  double d = (((-x1 * y2 * z3) + (x1 * y3 * z2) + (x2 * y1 * z3)) - (x2 * y3 * z1) - (x3 * y1 * z2)) + (x3 * y2 * z1);

  if ((std::fabs(b) <= std::numeric_limits<double>::epsilon()) &&
      (std::fabs(c) <= std::numeric_limits<double>::epsilon())) {
    const int typeAxD = 1;
    coplanar_plane_type = typeAxD; // ax=d
  }
  else if ((std::fabs(a) <= std::numeric_limits<double>::epsilon()) &&
            (std::fabs(c) <= std::numeric_limits<double>::epsilon())) {
    const int typeByD = 2;
    coplanar_plane_type = typeByD; // by=d
  }
  else if ((std::fabs(a) <= std::numeric_limits<double>::epsilon()) &&
            (std::fabs(b) <= std::numeric_limits<double>::epsilon())) {
    const int typeCzD = 3;
    coplanar_plane_type = typeCzD; // cz=d
  }

  double D = sqrt(vpMath::sqr(a) + vpMath::sqr(b) + vpMath::sqr(c));

  std::vector<vpPoint>::const_iterator shuffled_listp_end_decl2 = shuffled_listP.end();
  for (it = shuffled_listP.begin(); it != shuffled_listp_end_decl2; ++it) {
    P1 = *it;
    double dist = ((a * P1.get_oX()) + (b * P1.get_oY()) + (c * P1.get_oZ()) + d) / D;

    if (fabs(dist) > distToPlaneForCoplanarityTest) {
      // points are not coplanar
      return false;
    }
  }

  // points are  coplanar

  // If the points are coplanar and the input/output parameters are different from nullptr,
  // getting the values of the plan coefficient and storing in the input/output parameters
  if (p_a != nullptr) {
    *p_a = a;
  }

  if (p_b != nullptr) {
    *p_b = b;
  }

  if (p_c != nullptr) {
    *p_c = c;
  }

  if (p_d != nullptr) {
    *p_d = d;
  }

  return true;
}

double vpPose::computeResidual(const vpHomogeneousMatrix &cMo) const
{
  double squared_error = 0;
  vpPoint P;
  std::list<vpPoint>::const_iterator listp_end = listP.end();
  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listp_end; ++it) {
    P = *it;
    double x = P.get_x();
    double y = P.get_y();

    P.track(cMo);

    squared_error += vpMath::sqr(x - P.get_x()) + vpMath::sqr(y - P.get_y());
  }
  return squared_error;
}

double vpPose::computeResidual(const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam) const
{
  vpColVector residuals;
  return computeResidual(cMo, cam, residuals);
}

double vpPose::computeResidual(const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam, vpColVector &residuals) const
{
  double squared_error = 0;
  residuals.resize(static_cast<unsigned int>(listP.size()));
  vpPoint P;
  unsigned int i = 0;
  std::list<vpPoint>::const_iterator listp_end = listP.end();
  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listp_end; ++it) {
    P = *it;
    double x = P.get_x();
    double y = P.get_y();

    double u_initial = 0., v_initial = 0.;
    vpMeterPixelConversion::convertPoint(cam, x, y, u_initial, v_initial);

    P.track(cMo);

    double u_moved = 0., v_moved = 0.;
    vpMeterPixelConversion::convertPoint(cam, P.get_x(), P.get_y(), u_moved, v_moved);

    double squaredResidual = vpMath::sqr(u_moved - u_initial) + vpMath::sqr(v_moved - v_initial);
    residuals[i] = squaredResidual;
    ++i;
    squared_error += squaredResidual;
  }
  return squared_error;
}

void vpPose::callLagrangePose(vpHomogeneousMatrix &cMo)
{
  const int minNbPtLagrangePlan = 4;
  const int minNbPtLagrangeNoPlan = 6;
  // test if the 3D points are coplanar
  double a, b, c, d; // To get the plan coefficients if the points are coplanar
  int coplanar_plane_type = 0;
  bool plan = coplanar(coplanar_plane_type, &a, &b, &c, &d);

  if (plan == true) {
    const int typeCollinear = 4;
    if (coplanar_plane_type == typeCollinear) {
      throw(vpPoseException(vpPoseException::notEnoughPointError, "Lagrange method cannot be used in that case "
                            "(points are collinear)"));
    }
    if (npt < minNbPtLagrangePlan) {
      throw(vpPoseException(vpPoseException::notEnoughPointError,
                            "Lagrange method cannot be used in that case "
                            "(at least 4 points are required). "
                            "Not enough point (%d) to compute the pose  ",
                            npt));
    }
    poseLagrangePlan(cMo, &plan, &a, &b, &c, &d);
  }
  else {
    if (npt < minNbPtLagrangeNoPlan) {
      throw(vpPoseException(vpPoseException::notEnoughPointError,
                            "Lagrange method cannot be used in that case "
                            "(at least 6 points are required when 3D points are non coplanar). "
                            "Not enough point (%d) to compute the pose  ",
                            npt));
    }
    poseLagrangeNonPlan(cMo);
  }
}

bool vpPose::computePose(vpPoseMethodType method, vpHomogeneousMatrix &cMo, funcCheckValidityPose func)
{
  const int minNbPtDementhon = 4;
  const int minNbPtRansac = 4;
  std::stringstream errMsgDementhon;
  errMsgDementhon << "Dementhon method cannot be used in that case "
    << "(at least " << minNbPtDementhon << " points are required)"
    << "Not enough point (" << npt << ") to compute the pose  ";

  switch (method) {
  case DEMENTHON:
  case DEMENTHON_VIRTUAL_VS:
  case DEMENTHON_LOWE: {
    if (npt < minNbPtDementhon) {
      throw(vpPoseException(vpPoseException::notEnoughPointError, errMsgDementhon.str()));
    }
    // test if the 3D points are coplanar
    int coplanar_plane_type = 0;
    bool plan = coplanar(coplanar_plane_type);
    plan ? poseDementhonPlan(cMo) : poseDementhonNonPlan(cMo);
    break;
  }
  case LAGRANGE:
  case LAGRANGE_VIRTUAL_VS:
  case LAGRANGE_LOWE: {
    callLagrangePose(cMo);
    break;
  }
  case RANSAC: {
    if (npt < minNbPtRansac) {
      throw(vpPoseException(vpPoseException::notEnoughPointError,
                            "Ransac method cannot be used in that case "
                            "(at least 4 points are required). "
                            "Not enough point (%d) to compute the pose  ",
                            npt));
    }
    return poseRansac(cMo, func);
  }
  case LOWE:
  case VIRTUAL_VS:
    break;
  case DEMENTHON_LAGRANGE_VIRTUAL_VS: {
    return computePoseDementhonLagrangeVVS(cMo);
  }
  default:
  {
    std::cout << "method not identified" << std::endl;
  }
  }

  switch (method) {
  case LAGRANGE:
  case DEMENTHON:
  case DEMENTHON_LAGRANGE_VIRTUAL_VS:
  case RANSAC:
    break;
  case VIRTUAL_VS:
  case LAGRANGE_VIRTUAL_VS:
  case DEMENTHON_VIRTUAL_VS: {
    poseVirtualVS(cMo);
    break;
  }
  case LOWE:
  case LAGRANGE_LOWE:
  case DEMENTHON_LOWE:
  {
    poseLowe(cMo);
  }
  break;
  default:
  {
    std::cout << "method not identified" << std::endl;
  }
  }

  return true;
}

bool vpPose::computePoseDementhonLagrangeVVS(vpHomogeneousMatrix &cMo)
{
  vpHomogeneousMatrix cMo_dementhon, cMo_lagrange;
  double r_dementhon = std::numeric_limits<double>::max(), r_lagrange = std::numeric_limits<double>::max();
  // test if the 3D points are coplanar
  double a, b, c, d; // To get the plan coefficients if the points are coplanar
  int coplanar_plane_type = 0;
  bool plan = coplanar(coplanar_plane_type, &a, &b, &c, &d);
  bool hasDementhonSucceeded(false), hasLagrangeSucceeded(false);
  try {
    if (plan) {
      poseDementhonPlan(cMo_dementhon);
    }
    else {
      poseDementhonNonPlan(cMo_dementhon);
    }

    r_dementhon = computeResidual(cMo_dementhon);
    hasDementhonSucceeded = true; // We reached this point => no exception was thrown = method succeeded
  }
  catch (...) {
    // An exception was thrown using the original assumption, trying we the other one
    try {
      if (plan) {
        // Already tested poseDementhonPlan, now trying poseDementhonNonPlan
        poseDementhonNonPlan(cMo_dementhon);
      }
      else {
        // Already tested poseDementhonNonPlan, now trying poseDementhonPlan
        poseDementhonPlan(cMo_dementhon);
      }

      r_dementhon = computeResidual(cMo_dementhon);
      hasDementhonSucceeded = true; // We reached this point => no exception was thrown = method succeeded
    }
    catch (...) {
      // The Dementhon method failed both with the planar and non-planar assumptions.
      hasDementhonSucceeded = false;
    }
  }

  try {
    if (plan) {
      // If plan is true, then a, b, c, d will have been set when we called coplanar.
      poseLagrangePlan(cMo_lagrange, &plan, &a, &b, &c, &d);
    }
    else {
      poseLagrangeNonPlan(cMo_lagrange);
    }

    r_lagrange = computeResidual(cMo_lagrange);
    hasLagrangeSucceeded = true; // We reached this point => no exception was thrown = method succeeded
  }
  catch (...) {
    // An exception was thrown using the original assumption, trying we the other one
    try {
      if (plan) {
        // Already tested poseLagrangePlan, now trying poseLagrangeNonPlan
        poseLagrangeNonPlan(cMo_lagrange);
      }
      else {
        // Already tested poseLagrangeNonPlan, now trying poseLagrangePlan
        // Because plan is false, then a, b, c, d will not have
        // been initialized when calling coplanar
        // We are expecting that the call to poseLagrangePlan will throw an exception
        // because coplanar return false.
        poseLagrangePlan(cMo_lagrange, &plan, &a, &b, &c, &d);
      }

      r_lagrange = computeResidual(cMo_lagrange);
      hasLagrangeSucceeded = true; // We reached this point => no exception was thrown = method succeeded
    }
    catch (...) {
      // The Lagrange method both failed with the planar and non-planar assumptions.
      hasLagrangeSucceeded = false;
    }
  }

  if (hasDementhonSucceeded || hasLagrangeSucceeded) {
    // At least one of the linear methods managed to compute an initial pose.
    // We initialize cMo with the method that had the lowest residual
    cMo = (r_dementhon < r_lagrange) ? cMo_dementhon : cMo_lagrange;
    // We now use the non-linear Virtual Visual Servoing method to improve the estimated cMo
    return computePose(vpPose::VIRTUAL_VS, cMo);
  }
  else {
    // None of the linear methods manage to compute an initial pose
    return false;
  }
}

void vpPose::printPoint()
{
  vpPoint P;
  std::list<vpPoint>::const_iterator listp_end = listP.end();
  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listp_end; ++it) {
    P = *it;

    std::cout << "3D oP " << P.oP.t();
    std::cout << "3D cP " << P.cP.t();
    std::cout << "2D    " << P.p.t();
  }
}

void vpPose::display(vpImage<unsigned char> &I, vpHomogeneousMatrix &cMo, vpCameraParameters &cam, double size,
                     vpColor col)
{
  vpDisplay::displayFrame(I, cMo, cam, size, col);
}

void vpPose::display(vpImage<vpRGBa> &I, vpHomogeneousMatrix &cMo, vpCameraParameters &cam, double size, vpColor col)
{
  vpDisplay::displayFrame(I, cMo, cam, size, col);
}

void vpPose::displayModel(vpImage<unsigned char> &I, vpCameraParameters &cam, vpColor col)
{
  vpPoint P;
  vpImagePoint ip;
  std::list<vpPoint>::const_iterator listp_end = listP.end();
  const unsigned int sizeCross = 5;
  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listp_end; ++it) {
    P = *it;
    vpMeterPixelConversion::convertPoint(cam, P.p[0], P.p[1], ip);
    vpDisplay::displayCross(I, ip, sizeCross, col);
  }
}

void vpPose::displayModel(vpImage<vpRGBa> &I, vpCameraParameters &cam, vpColor col)
{
  vpPoint P;
  vpImagePoint ip;
  std::list<vpPoint>::const_iterator listp_end = listP.end();
  const unsigned int sizeCross = 5;

  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listp_end; ++it) {
    P = *it;
    vpMeterPixelConversion::convertPoint(cam, P.p[0], P.p[1], ip);
    vpDisplay::displayCross(I, ip, sizeCross, col);
  }
}

#ifdef VISP_HAVE_HOMOGRAPHY
double vpPose::poseFromRectangle(vpPoint &p1, vpPoint &p2, vpPoint &p3, vpPoint &p4, double lx, vpCameraParameters &cam,
                                 vpHomogeneousMatrix &cMo)
{
  const unsigned int id0 = 0, id1 = 1, id2 = 2, id3 = 3;
  std::vector<double> rectx(4);
  std::vector<double> recty(4);
  rectx[id0] = 0;
  recty[id0] = 0;
  rectx[id1] = 1;
  recty[id1] = 0;
  rectx[id2] = 1;
  recty[id2] = 1;
  rectx[id3] = 0;
  recty[id3] = 1;
  std::vector<double> irectx(4);
  std::vector<double> irecty(4);
  irectx[id0] = (p1.get_x());
  irecty[id0] = (p1.get_y());
  irectx[id1] = (p2.get_x());
  irecty[id1] = (p2.get_y());
  irectx[id2] = (p3.get_x());
  irecty[id2] = (p3.get_y());
  irectx[id3] = (p4.get_x());
  irecty[id3] = (p4.get_y());

  // calcul de l'homographie
  vpMatrix H(3, 3);
  vpHomography hom;

  vpHomography::HLM(rectx, recty, irectx, irecty, 1, hom);
  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    for (unsigned int j = 0; j < val_3; ++j) {
      H[i][j] = hom[i][j];
    }
  }
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
#endif

END_VISP_NAMESPACE
