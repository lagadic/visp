/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 */

#include <visp3/rbt/vpRBVisualOdometryUtils.h>

#include <visp3/core/vpMatrix.h>
#include <visp3/rbt/vpRBFeatureTrackerInput.h>
#include <vector>
#include <utility>

#include <visp3/core/vpRobust.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpHomogeneousMatrix.h>

BEGIN_VISP_NAMESPACE

std::pair<std::vector<unsigned int>, std::vector<unsigned int>>
vpRBVisualOdometryUtils::computeIndicesObjectAndEnvironment(const vpMatrix &keypoints,
                                                            const vpRBFeatureTrackerInput &frame,
                                                            double minMaskConfidence, double minEdgeDistObject,
                                                            double minEdgeDistEnv)
{
  std::vector<unsigned int> objIndices, envIndices;
  const vpImage<float> &renderDepth = frame.renders.depth;
  const unsigned int h = renderDepth.getHeight(), w = renderDepth.getWidth();
  const bool useMask = frame.hasMask() && minMaskConfidence > 0.0;
  const double thresholdObject = vpMath::sqr(minEdgeDistObject), thresholdEnv = vpMath::sqr(minEdgeDistEnv);

  std::vector<std::pair<double, double>> actualSilhouettePoints;
  actualSilhouettePoints.reserve(frame.silhouettePoints.size());
  for (const vpRBSilhouettePoint &p: frame.silhouettePoints) {
    if (p.isSilhouette) {
      actualSilhouettePoints.push_back(std::make_pair(
        static_cast<double>(p.i), static_cast<double>(p.j)
      ));
    }
  }

  const auto testDistanceEdge = [&actualSilhouettePoints](double u, double v, double threshold) -> bool {
    for (const std::pair<double, double> &p: actualSilhouettePoints) {
      double dist2 = vpMath::sqr(p.first - v) + vpMath::sqr(p.second - u);
      if (dist2 < threshold) {
        return false;
      }
    }
    return true;
    };

  for (unsigned int i = 0; i < keypoints.getRows(); ++i) {
    double u = keypoints[i][0], v = keypoints[i][1];
    if (u < 0.0 || v < 0.0 || u >= w || v >= h) {
      continue;
    }
    const unsigned int ui = static_cast<unsigned int>(u), vi = static_cast<unsigned int>(v);
    const double Z = static_cast<double>(renderDepth[vi][ui]);

    if (Z > 0.0) { // Potential object candidate
      if (!useMask || frame.mask[vi][ui] > minMaskConfidence) {
        bool notTooCloseToEdge = testDistanceEdge(u, v, thresholdObject);
        if (notTooCloseToEdge) {
          objIndices.push_back(i);
        }
      }
    }
    else { // Env candidate
      bool notTooCloseToEdge = testDistanceEdge(u, v, thresholdEnv);
      if (notTooCloseToEdge) {
        envIndices.push_back(i);
      }
    }
  }
  return std::make_pair(objIndices, envIndices);
}

void vpRBVisualOdometryUtils::levenbergMarquardtKeypoints2D(const vpMatrix &points3d, const vpMatrix &observations,
                                                            const vpLevenbergMarquardtParameters &parameters,
                                                            vpHomogeneousMatrix &cTw)
{
  vpMatrix L(points3d.getRows() * 2, 6);
  vpMatrix Lt(6, points3d.getRows());
  vpColVector e(points3d.getRows() * 2);
  vpColVector weights(points3d.getRows() * 2);
  vpColVector weighted_error(points3d.getRows() * 2);

  if (points3d.getRows() != observations.getRows()) {
    throw vpException(vpException::dimensionError, "Expected number of 3D points and 2D observations to be the same");
  }
  vpMatrix points3dTranspose = points3d.t();
  vpMatrix cXt(points3d.getRows(), points3d.getCols());
  vpMatrix Id(6, 6);
  Id.eye();
  vpRobust robust;
  vpMatrix H(6, 6);
  double mu = parameters.muInit;
  double errorNormPrev = std::numeric_limits<double>::max();
  for (unsigned int iter = 0; iter < parameters.maxNumIters; ++iter) {
    const vpTranslationVector t = cTw.getTranslationVector();
    const vpRotationMatrix cRw = cTw.getRotationMatrix();
    vpMatrix::mult2Matrices(cRw, points3dTranspose, cXt);
    // Project 3D points and compute interaction matrix
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < static_cast<int>(points3d.getRows()); ++i) {
      const double X = cXt[0][i] + t[0], Y = cXt[1][i] + t[1], Z = cXt[2][i] + t[2];
      const double x = X / Z, y = Y / Z;
      e[i * 2] = x - observations[i][0];
      e[i * 2 + 1] = y - observations[i][1];

      L[i * 2][0] = -1.0 / Z; L[i * 2][1] = 0.0; L[i * 2][2] = x / Z;
      L[i * 2][3] = x * y; L[i * 2][4] = -(1.0 + x * x); L[i * 2][5] = y;

      L[i * 2 + 1][0] = 0.0; L[i * 2 + 1][1] = -1.0 / Z; L[i * 2 + 1][2] = y / Z;
      L[i * 2 + 1][3] = 1.0 + y * y; L[i * 2 + 1][4] = -(x * y); L[i * 2 + 1][5] = -x;
    }

    robust.MEstimator(vpRobust::TUKEY, e, weights);
    for (unsigned int i = 0; i < e.getRows(); ++i) {
      weighted_error[i] = e[i] * weights[i];
      for (unsigned int j = 0; j < 6; ++j) {
        L[i][j] *= weights[i];
      }
    }

    L.transpose(Lt);
    L.AtA(H);
    vpColVector Lte = Lt * weighted_error;
    vpColVector v = -parameters.gain * ((H + Id * mu).pseudoInverse() * Lte);
    cTw = vpExponentialMap::direct(v).inverse() * cTw;
    double errorNormCurr = weighted_error.frobeniusNorm();
    double improvementFactor = errorNormCurr / errorNormPrev;
    if (iter > 0) {
      if (improvementFactor < 1.0 && improvementFactor >(1.0 - parameters.minImprovementFactor)) {
        break;
      }
    }
    // if (improvementFactor < 1.0) {
    mu *= parameters.muIterFactor;
  // }
  // else {
  //   mu /= parameters.muIterFactor;
  // }
    errorNormPrev = errorNormCurr;
  }
}


void vpRBVisualOdometryUtils::levenbergMarquardtKeypoints3D(const vpMatrix &points3d, const vpMatrix &observations,
                                                            const vpLevenbergMarquardtParameters &parameters,
                                                            vpHomogeneousMatrix &cTw)
{
  vpMatrix L(points3d.getRows() * 3, 6, 0.0);
  vpMatrix Lt(6, points3d.getRows());
  vpColVector e(points3d.getRows() * 3);
  vpColVector weights(points3d.getRows() * 3);
  vpColVector weighted_error(points3d.getRows() * 3);

  if (points3d.getRows() != observations.getRows()) {
    throw vpException(vpException::dimensionError, "Expected number of 3D points and 2D observations to be the same");
  }
  vpMatrix points3dTranspose = points3d.t();
  vpMatrix cXt(points3d.getRows(), points3d.getCols());
  vpMatrix Id(6, 6);
  Id.eye();
  vpRobust robust;
  vpMatrix H(6, 6);
  double mu = parameters.muInit;
  double errorNormPrev = std::numeric_limits<double>::max();
  for (unsigned int iter = 0; iter < parameters.maxNumIters; ++iter) {
    const vpTranslationVector t = cTw.getTranslationVector();
    const vpRotationMatrix cRw = cTw.getRotationMatrix();
    vpMatrix::mult2Matrices(cRw, points3dTranspose, cXt);
    // Project 3D points and compute interaction matrix
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < static_cast<int>(points3d.getRows()); ++i) {
      const double X = cXt[0][i] + t[0], Y = cXt[1][i] + t[1], Z = cXt[2][i] + t[2];

      e[i * 3] = X - observations[i][0];
      e[i * 3 + 1] = Y - observations[i][1];
      e[i * 3 + 2] = Z - observations[i][2];

      L[i * 3][0] = -1;
      L[i * 3 + 1][1] = -1;
      L[i * 3 + 2][2] = -1;

      L[i * 3][4] = -Z;
      L[i * 3][5] = Y;

      L[i * 3 + 1][3] = Z;
      L[i * 3 + 1][5] = -X;

      L[i * 3 + 2][3] = -Y;
      L[i * 3 + 2][4] = X;
    }

    robust.MEstimator(vpRobust::TUKEY, e, weights);
    for (unsigned int i = 0; i < e.getRows(); ++i) {
      weighted_error[i] = e[i] * weights[i];
      for (unsigned int j = 0; j < 6; ++j) {
        L[i][j] *= weights[i];
      }
    }

    L.transpose(Lt);
    L.AtA(H);
    vpColVector Lte = Lt * weighted_error;
    vpColVector v = -parameters.gain * ((H + Id * mu).pseudoInverse() * Lte);
    cTw = vpExponentialMap::direct(v).inverse() * cTw;
    double errorNormCurr = weighted_error.frobeniusNorm();
    double improvementFactor = errorNormCurr / errorNormPrev;
    if (iter > 0) {
      if (improvementFactor < 1.0 && improvementFactor >(1.0 - parameters.minImprovementFactor)) {
        break;
      }
    }
    // if (improvementFactor < 1.0) {
    mu *= parameters.muIterFactor;
  // }
  // else {
  //   mu /= parameters.muIterFactor;
  // }
    errorNormPrev = errorNormCurr;
  }
}

END_VISP_NAMESPACE
