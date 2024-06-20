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
  \file vpPoseVirtualVisualServoing.cpp
  \brief Compute the pose using virtual visual servoing approach
*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpRobust.h>
#include <visp3/vision/vpPose.h>

BEGIN_VISP_NAMESPACE

void vpPose::poseVirtualVS(vpHomogeneousMatrix &cMo)
{
  try {
    double residu_1 = 1e8;
    double r = 1e8 - 1;
    const unsigned int index_0 = 0;
    const unsigned int index_1 = 1;
    const unsigned int index_2 = 2;
    const unsigned int index_3 = 3;
    const unsigned int index_4 = 4;
    const unsigned int index_5 = 5;
    const unsigned int dim2DPoints = 2;

    // we stop the minimization when the error is bellow 1e-8

    int iter = 0;

    unsigned int nb = static_cast<unsigned int>(listP.size());
    const unsigned int nbColsL = 6;
    vpMatrix L(dim2DPoints * nb, nbColsL);
    vpColVector err(dim2DPoints * nb);
    vpColVector sd(dim2DPoints * nb), s(dim2DPoints * nb);
    vpColVector v;

    vpPoint P;
    std::list<vpPoint> lP;

    // create sd
    unsigned int k = 0;
    std::list<vpPoint>::const_iterator listp_end = listP.end();
    for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listp_end; ++it) {
      P = *it;
      sd[dim2DPoints * k] = P.get_x();
      sd[(dim2DPoints * k) + 1] = P.get_y();
      lP.push_back(P);
      ++k;
    }

    vpHomogeneousMatrix cMoPrev = cMo;
    /*
    // --comment: while((int)((residu_1 - r)*1e12) !=0)
    // --comment: while(std::fabs((residu_1 - r)*1e12) >
    // --comment: std::numeric_limits < double > :: epsilon())
    */
    bool iter_gt_vvsitermax = false;
    while ((std::fabs(residu_1 - r) > vvsEpsilon) && (iter_gt_vvsitermax == false)) {
      residu_1 = r;

      // Compute the interaction matrix and the error
      k = 0;
      std::list<vpPoint>::const_iterator lp_end = lP.end();
      for (std::list<vpPoint>::const_iterator it = lP.begin(); it != lp_end; ++it) {
        P = *it;
        // forward projection of the 3D model for a given pose
        // change frame coordinates
        // perspective projection
        P.track(cMo);

        double x = s[dim2DPoints * k] = P.get_x(); /* point projected from cMo */
        double y = s[(dim2DPoints * k) + index_1] = P.get_y();
        double Z = P.get_Z();
        L[dim2DPoints * k][index_0] = -1 / Z;
        L[dim2DPoints * k][index_1] = 0;
        L[dim2DPoints * k][index_2] = x / Z;
        L[dim2DPoints * k][index_3] = x * y;
        L[dim2DPoints * k][index_4] = -(1 + (x * x));
        L[dim2DPoints * k][index_5] = y;

        L[(dim2DPoints * k) + 1][index_0] = 0;
        L[(dim2DPoints * k) + 1][index_1] = -1 / Z;
        L[(dim2DPoints * k) + 1][index_2] = y / Z;
        L[(dim2DPoints * k) + 1][index_3] = 1 + (y * y);
        L[(dim2DPoints * k) + 1][index_4] = -x * y;
        L[(dim2DPoints * k) + 1][index_5] = -x;

        k += 1;
      }
      err = s - sd;

      // compute the residual
      r = err.sumSquare();

      // compute the pseudo inverse of the interaction matrix
      vpMatrix Lp;
      L.pseudoInverse(Lp, 1e-16);

      // compute the VVS control law
      v = -m_lambda * Lp * err;

      // update the pose

      cMoPrev = cMo;
      cMo = vpExponentialMap::direct(v).inverse() * cMo;

      if (iter> vvsIterMax) {
        iter_gt_vvsitermax = true;
        // break
      }
      else {
        ++iter;
      }
    }

    if (computeCovariance) {
      covarianceMatrix = vpMatrix::computeCovarianceMatrixVVS(cMoPrev, err, L);
    }
  }
  catch (...) {
    throw(vpException(vpException::fatalError, "poseVirtualVS did not succeed"));
  }
}

void vpPose::poseVirtualVSrobust(vpHomogeneousMatrix &cMo)
{
  double residu_1 = 1e8;
  double r = 1e8 - 1;
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;
  const unsigned int dim2DPoints = 2;

  // we stop the minimization when the error is bellow 1e-8
  vpMatrix W;
  vpRobust robust;
  robust.setMinMedianAbsoluteDeviation(0.00001);
  vpColVector w, res;

  unsigned int nb = static_cast<unsigned int>(listP.size());
  const unsigned int nbColsL = 6;
  vpMatrix L(dim2DPoints * nb, nbColsL);
  vpColVector error(dim2DPoints * nb);
  vpColVector sd(dim2DPoints * nb), s(dim2DPoints * nb);
  vpColVector v;

  vpPoint P;
  std::list<vpPoint> lP;

  // create sd
  unsigned int k_ = 0;
  std::list<vpPoint>::const_iterator listp_end = listP.end();
  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listp_end; ++it) {
    P = *it;
    sd[dim2DPoints * k_] = P.get_x();
    sd[(dim2DPoints * k_) + 1] = P.get_y();
    lP.push_back(P);
    ++k_;
  }
  int iter = 0;
  res.resize(s.getRows() / dim2DPoints);
  w.resize(s.getRows() / dim2DPoints);
  W.resize(s.getRows(), s.getRows());
  w = 1;

  //  --comment: while (residu_1 - r) times 1e12 diff 0
  bool iter_gt_vvsitermax = false;
  while ((std::fabs((residu_1 - r) * 1e12) > std::numeric_limits<double>::epsilon()) && (iter_gt_vvsitermax == false)) {
    residu_1 = r;

    // Compute the interaction matrix and the error
    k_ = 0;
    std::list<vpPoint>::const_iterator lp_end = lP.end();
    for (std::list<vpPoint>::const_iterator it = lP.begin(); it != lp_end; ++it) {
      P = *it;
      // forward projection of the 3D model for a given pose
      // change frame coordinates
      // perspective projection
      P.track(cMo);

      double x = s[dim2DPoints * k_] = P.get_x(); // point projected from cMo
      double y = s[(dim2DPoints * k_) + index_1] = P.get_y();
      double Z = P.get_Z();
      L[dim2DPoints * k_][index_0] = -1 / Z;
      L[dim2DPoints * k_][index_1] = 0;
      L[dim2DPoints * k_][index_2] = x / Z;
      L[dim2DPoints * k_][index_3] = x * y;
      L[dim2DPoints * k_][index_4] = -(1 + (x * x));
      L[dim2DPoints * k_][index_5] = y;

      L[(dim2DPoints * k_) + index_1][index_0] = 0;
      L[(dim2DPoints * k_) + index_1][index_1] = -1 / Z;
      L[(dim2DPoints * k_) + index_1][index_2] = y / Z;
      L[(dim2DPoints * k_) + index_1][index_3] = 1 + (y * y);
      L[(dim2DPoints * k_) + index_1][index_4] = -x * y;
      L[(dim2DPoints * k_) + index_1][index_5] = -x;

      ++k_;
    }
    error = s - sd;

    // compute the residual
    r = error.sumSquare();

    unsigned int v_error_rows = error.getRows();
    const unsigned int nbPts = v_error_rows / dim2DPoints;
    for (unsigned int k = 0; k < nbPts; ++k) {
      res[k] = vpMath::sqr(error[dim2DPoints * k]) + vpMath::sqr(error[(dim2DPoints * k) + 1]);
    }
    robust.MEstimator(vpRobust::TUKEY, res, w);

    // compute the pseudo inverse of the interaction matrix
    unsigned int error_rows = error.getRows();
    unsigned int nbErrors = error_rows / dim2DPoints;
    for (unsigned int k = 0; k < nbErrors; ++k) {
      W[dim2DPoints * k][dim2DPoints * k] = w[k];
      W[(dim2DPoints * k) + 1][(dim2DPoints * k) + 1] = w[k];
    }
    // compute the pseudo inverse of the interaction matrix
    vpMatrix Lp;
    (W * L).pseudoInverse(Lp, 1e-6);

    // compute the VVS control law
    v = -m_lambda * Lp * W * error;

    cMo = vpExponentialMap::direct(v).inverse() * cMo;
    if (iter > vvsIterMax) {
      iter_gt_vvsitermax = true;
      // break
    }
    else {
      ++iter;
    }
  }

  if (computeCovariance) {
    covarianceMatrix =
      vpMatrix::computeCovarianceMatrix(L, v, -m_lambda * error, W * W); // Remark: W*W = W*W.t() since the
                                                                       // matrix is diagonale, but using W*W
                                                                       // is more efficient.
  }
}

// Check if std:c++17 or higher
#if ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))
std::optional<vpHomogeneousMatrix> vpPose::poseVirtualVSWithDepth(const std::vector<vpPoint> &points, const vpHomogeneousMatrix &cMo)
{
  auto residu_1 { 1e8 }, r { 1e8 - 1 };
  const auto lambda { 0.9 }, vvsEpsilon { 1e-8 };
  const unsigned int vvsIterMax { 200 };
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;

  const unsigned int nb = static_cast<unsigned int>(points.size());
  const unsigned int sizePoints = 3;
  const unsigned int nbColsL = 6;
  vpMatrix L(sizePoints * nb, nbColsL);
  vpColVector err(sizePoints * nb);
  vpColVector sd(sizePoints * nb), s(sizePoints * nb);

  // create sd
  auto v_points_size = points.size();
  for (auto i = 0u; i < v_points_size; ++i) {
    sd[sizePoints * i] = points[i].get_x();
    sd[(sizePoints * i) + index_1] = points[i].get_y();
    sd[(sizePoints * i) + index_2] = points[i].get_Z();
  }

  auto cMoPrev = cMo;
  auto iter = 0u;
  while (std::fabs(residu_1 - r) > vvsEpsilon) {
    residu_1 = r;

    // Compute the interaction matrix and the error
    auto points_size = points.size();
    for (auto i = 0u; i < points_size; ++i) {
      // forward projection of the 3D model for a given pose
      // change frame coordinates
      // perspective projection
      vpColVector cP, p;
      points.at(i).changeFrame(cMo, cP);
      points.at(i).projection(cP, p);

      const auto x = s[sizePoints * i] = p[index_0];
      const auto y = s[(sizePoints * i) + index_1] = p[index_1];
      const auto Z = s[(sizePoints * i) + index_2] = cP[index_2];
      L[sizePoints * i][index_0] = -1. / Z;
      L[sizePoints * i][index_1] = 0;
      L[sizePoints * i][index_2] = x / Z;
      L[sizePoints * i][index_3] = x * y;
      L[sizePoints * i][index_4] = -(1. + vpMath::sqr(x));
      L[sizePoints * i][index_5] = y;

      L[(sizePoints * i) + index_1][index_0] = 0;
      L[(sizePoints * i) + index_1][index_1] = -1. / Z;
      L[(sizePoints * i) + index_1][index_2] = y / Z;
      L[(sizePoints * i) + index_1][index_3] = 1. + vpMath::sqr(y);
      L[(sizePoints * i) + index_1][index_4] = -x * y;
      L[(sizePoints * i) + index_1][index_5] = -x;

      L[(sizePoints * i) + index_2][index_0] = 0;
      L[(sizePoints * i) + index_2][index_1] = 0;
      L[(sizePoints * i) + index_2][index_2] = -1.;
      L[(sizePoints * i) + index_2][index_3] = -y * Z;
      L[(sizePoints * i) + index_2][index_4] = x * Z;
      L[(sizePoints * i) + index_2][index_5] = -0;
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

    if (iter > vvsIterMax) {
      return std::nullopt;
    }
    else {
      ++iter;
    }
  }
  return cMoPrev;
}

#endif

END_VISP_NAMESPACE
