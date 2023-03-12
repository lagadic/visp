/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2022 by Inria. All rights reserved.
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
 *
 *****************************************************************************/

/*!
  \file vpPoseVirtualVisualServoing.cpp
  \brief Compute the pose using virtual visual servoing approach
*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpRobust.h>
#include <visp3/vision/vpPose.h>

/*!
  \brief Compute the pose using virtual visual servoing approach

  This approach is described in \cite Marchand02c.

*/
void vpPose::poseVirtualVS(vpHomogeneousMatrix &cMo)
{
  try {

    double residu_1 = 1e8;
    double r = 1e8 - 1;

    // we stop the minimization when the error is bellow 1e-8

    int iter = 0;

    unsigned int nb = (unsigned int)listP.size();
    vpMatrix L(2 * nb, 6);
    vpColVector err(2 * nb);
    vpColVector sd(2 * nb), s(2 * nb);
    vpColVector v;

    vpPoint P;
    std::list<vpPoint> lP;

    // create sd
    unsigned int k = 0;
    for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it) {
      P = *it;
      sd[2 * k] = P.get_x();
      sd[2 * k + 1] = P.get_y();
      lP.push_back(P);
      k++;
    }

    vpHomogeneousMatrix cMoPrev = cMo;
    // while((int)((residu_1 - r)*1e12) !=0)
    //    while(std::fabs((residu_1 - r)*1e12) >
    //    std::numeric_limits<double>::epsilon())
    while (std::fabs(residu_1 - r) > vvsEpsilon) {
      residu_1 = r;

      // Compute the interaction matrix and the error
      k = 0;
      for (std::list<vpPoint>::const_iterator it = lP.begin(); it != lP.end(); ++it) {
        P = *it;
        // forward projection of the 3D model for a given pose
        // change frame coordinates
        // perspective projection
        P.track(cMo);

        double x = s[2 * k] = P.get_x(); /* point projected from cMo */
        double y = s[2 * k + 1] = P.get_y();
        double Z = P.get_Z();
        L[2 * k][0] = -1 / Z;
        L[2 * k][1] = 0;
        L[2 * k][2] = x / Z;
        L[2 * k][3] = x * y;
        L[2 * k][4] = -(1 + x * x);
        L[2 * k][5] = y;

        L[2 * k + 1][0] = 0;
        L[2 * k + 1][1] = -1 / Z;
        L[2 * k + 1][2] = y / Z;
        L[2 * k + 1][3] = 1 + y * y;
        L[2 * k + 1][4] = -x * y;
        L[2 * k + 1][5] = -x;

        k += 1;
      }
      err = s - sd;

      // compute the residual
      r = err.sumSquare();

      // compute the pseudo inverse of the interaction matrix
      vpMatrix Lp;
      L.pseudoInverse(Lp, 1e-16);

      // compute the VVS control law
      v = -lambda * Lp * err;

      // std::cout << "r=" << r <<std::endl ;
      // update the pose

      cMoPrev = cMo;
      cMo = vpExponentialMap::direct(v).inverse() * cMo;

      if (iter++ > vvsIterMax) {
        break;
      }
    }

    if (computeCovariance)
      covarianceMatrix = vpMatrix::computeCovarianceMatrixVVS(cMoPrev, err, L);
  }

  catch (...) {
    vpERROR_TRACE(" ");
    throw;
  }
}

/*!
  \brief Compute the pose using virtual visual servoing approach and
  a robust control law

  This approach is described in \cite Comport06b.

*/
void vpPose::poseVirtualVSrobust(vpHomogeneousMatrix &cMo)
{
  try {

    double residu_1 = 1e8;
    double r = 1e8 - 1;

    // we stop the minimization when the error is bellow 1e-8
    vpMatrix W;
    vpRobust robust;
    robust.setMinMedianAbsoluteDeviation(0.00001);
    vpColVector w, res;

    unsigned int nb = (unsigned int)listP.size();
    vpMatrix L(2 * nb, 6);
    vpColVector error(2 * nb);
    vpColVector sd(2 * nb), s(2 * nb);
    vpColVector v;

    vpPoint P;
    std::list<vpPoint> lP;

    // create sd
    unsigned int k_ = 0;
    for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it) {
      P = *it;
      sd[2 * k_] = P.get_x();
      sd[2 * k_ + 1] = P.get_y();
      lP.push_back(P);
      k_++;
    }
    int iter = 0;
    res.resize(s.getRows() / 2);
    w.resize(s.getRows() / 2);
    W.resize(s.getRows(), s.getRows());
    w = 1;

    // while((int)((residu_1 - r)*1e12) !=0)
    while (std::fabs((residu_1 - r) * 1e12) > std::numeric_limits<double>::epsilon()) {
      residu_1 = r;

      // Compute the interaction matrix and the error
      k_ = 0;
      for (std::list<vpPoint>::const_iterator it = lP.begin(); it != lP.end(); ++it) {
        P = *it;
        // forward projection of the 3D model for a given pose
        // change frame coordinates
        // perspective projection
        P.track(cMo);

        double x = s[2 * k_] = P.get_x(); // point projected from cMo
        double y = s[2 * k_ + 1] = P.get_y();
        double Z = P.get_Z();
        L[2 * k_][0] = -1 / Z;
        L[2 * k_][1] = 0;
        L[2 * k_][2] = x / Z;
        L[2 * k_][3] = x * y;
        L[2 * k_][4] = -(1 + x * x);
        L[2 * k_][5] = y;

        L[2 * k_ + 1][0] = 0;
        L[2 * k_ + 1][1] = -1 / Z;
        L[2 * k_ + 1][2] = y / Z;
        L[2 * k_ + 1][3] = 1 + y * y;
        L[2 * k_ + 1][4] = -x * y;
        L[2 * k_ + 1][5] = -x;

        k_++;
      }
      error = s - sd;

      // compute the residual
      r = error.sumSquare();

      for (unsigned int k = 0; k < error.getRows() / 2; k++) {
        res[k] = vpMath::sqr(error[2 * k]) + vpMath::sqr(error[2 * k + 1]);
      }
      robust.MEstimator(vpRobust::TUKEY, res, w);

      // compute the pseudo inverse of the interaction matrix
      for (unsigned int k = 0; k < error.getRows() / 2; k++) {
        W[2 * k][2 * k] = w[k];
        W[2 * k + 1][2 * k + 1] = w[k];
      }
      // compute the pseudo inverse of the interaction matrix
      vpMatrix Lp;
      (W * L).pseudoInverse(Lp, 1e-6);

      // compute the VVS control law
      v = -lambda * Lp * W * error;

      cMo = vpExponentialMap::direct(v).inverse() * cMo;
      if (iter++ > vvsIterMax)
        break;
    }

    if (computeCovariance)
      covarianceMatrix =
          vpMatrix::computeCovarianceMatrix(L, v, -lambda * error, W * W); // Remark: W*W = W*W.t() since the
                                                                           // matrix is diagonale, but using W*W
                                                                           // is more efficient.
  } catch (...) {
    vpERROR_TRACE(" ");
    throw;
  }
}

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17) &&                                                                     \
    (!defined(_MSC_VER) || ((VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17) && (_MSC_VER >= 1911)))
/*!
 * Compute the pose by virtual visual servoing using x,y and Z point coordinates as visual features.
 * We recall that x,y are the coordinates of a point in the image plane which are obtained by perspective
 * projection, while Z is the 3D coordinate of the point along the camera frame Z-axis.
 *
 * \param[in] cMo : Pose initial value used to initialize the non linear pose estimation algorithm.
 * \param[in] points : A vector of points with [x,y,Z] values used as visual features.
 * \return Estimated pose when the minimization converged, of std::nullopt when it failed.
 */
std::optional<vpHomogeneousMatrix> vpPose::poseVirtualVSWithDepth(const std::vector<vpPoint> &points, const vpHomogeneousMatrix &cMo)
{
  auto residu_1{1e8}, r{1e8 - 1};
  const auto lambda{0.9}, vvsEpsilon{1e-8};
  const unsigned int vvsIterMax{200};

  const unsigned int nb = static_cast<unsigned int>(points.size());
  vpMatrix L(3 * nb, 6);
  vpColVector err(3 * nb);
  vpColVector sd(3 * nb), s(3 * nb);

  // create sd
  for (auto i = 0u; i < points.size(); i++) {
    sd[3 * i] = points[i].get_x();
    sd[3 * i + 1] = points[i].get_y();
    sd[3 * i + 2] = points[i].get_Z();
  }

  auto cMoPrev = cMo;
  auto iter = 0u;
  while (std::fabs(residu_1 - r) > vvsEpsilon) {
    residu_1 = r;

    // Compute the interaction matrix and the error
    for (auto i = 0u; i < points.size(); i++) {
      // forward projection of the 3D model for a given pose
      // change frame coordinates
      // perspective projection
      vpColVector cP, p;
      points.at(i).changeFrame(cMo, cP);
      points.at(i).projection(cP, p);

      const auto x = s[3 * i] = p[0];
      const auto y = s[3 * i + 1] = p[1];
      const auto Z = s[3 * i + 2] = cP[2];
      L[3 * i][0] = -1 / Z;
      L[3 * i][1] = 0;
      L[3 * i][2] = x / Z;
      L[3 * i][3] = x * y;
      L[3 * i][4] = -(1 + vpMath::sqr(x));
      L[3 * i][5] = y;

      L[3 * i + 1][0] = 0;
      L[3 * i + 1][1] = -1 / Z;
      L[3 * i + 1][2] = y / Z;
      L[3 * i + 1][3] = 1 + vpMath::sqr(y);
      L[3 * i + 1][4] = -x * y;
      L[3 * i + 1][5] = -x;

      L[3 * i + 2][0] = 0;
      L[3 * i + 2][1] = 0;
      L[3 * i + 2][2] = -1;
      L[3 * i + 2][3] = -y * Z;
      L[3 * i + 2][4] = x * Z;
      L[3 * i + 2][5] = -0;
    }
    err = s - sd;

    // compute the residual
    r = err.sumSquare();

    // compute the pseudo inverse of the interaction matrix
    vpMatrix Lp;
    L.pseudoInverse(Lp, 1e-16);

    // compute the VVS control law
    const auto v = -lambda * Lp * err;

    // update the pose
    cMoPrev = vpExponentialMap::direct(v).inverse() * cMoPrev;

    if (iter++ > vvsIterMax) {
      return std::nullopt;
    }
  }
  return cMoPrev;
}

#endif
