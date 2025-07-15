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
 */

#include <visp3/rbt/vpRBPhotometricTracker.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpDisplay.h>

#ifdef VISP_HAVE_OPENMP
#include <omp.h>
#endif
BEGIN_VISP_NAMESPACE

// #define VISP_DEBUG_RB_DEPTH_DENSE_TRACKER 1

void fastProjectionSurfacePoint(const vpHomogeneousMatrix &oTc, double X, double Y, double Z, std::array<double, 3> &p)
{
  const double *T = oTc.data;
  p[0] = (T[0] * X + T[1] * Y + T[2] * Z + T[3]);
  p[1] = (T[4] * X + T[5] * Y + T[6] * Z + T[7]);
  p[2] = (T[8] * X + T[9] * Y + T[10] * Z + T[11]);
}

void vpRBPhotometricTracker::extractFeatures(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &previousFrame, const vpHomogeneousMatrix &/*cMo*/)
{
  const vpImage<float> &depthMap = frame.depth;
  const vpImage<float> &renderDepth = frame.renders.depth;
  const vpRect bb = frame.renders.boundingBox;
  const vpHomogeneousMatrix &cMo = frame.renders.cMo;
  const vpRotationMatrix cRo = cMo.getRotationMatrix();
  const vpHomogeneousMatrix oMc = cMo.inverse();
  const vpTranslationVector co = oMc.getTranslationVector(); // Position of the camera in object frame
  const bool useMask = m_useMask && frame.hasMask();
  m_surfacePoints.clear();
  m_surfacePoints.reserve(static_cast<size_t>(bb.getArea() / (m_step * m_step * 2)));

#ifdef VISP_HAVE_OPENMP
#pragma omp parallel
#endif
  {
    vpSurfacePoint point;
#ifdef VISP_HAVE_OPENMP
    std::vector<vpSurfacePoint> localPoints;
    localPoints.reserve(m_surfacePoints.capacity() / omp_get_num_threads());
#else
    // Just reference the global vector, no need to add to it later on
    std::vector<vpSurfacePoint> &localPoints = m_surfacePoints;
#endif

#ifdef VISP_HAVE_OPENMP
#pragma omp for nowait
#endif
    for (auto i = static_cast<int>(bb.getTop()); i < static_cast<int>(bb.getBottom()); i += m_step) {
      for (auto j = static_cast<int>(bb.getLeft()); j < static_cast<int>(bb.getRight()); j += m_step) {
        const double Z = renderDepth[i][j];
        if (Z > 0.f) {
          if (useMask && frame.mask[i][j] < m_minMaskConfidence) {
            continue;
          }
          double x = 0.0, y = 0.0;
          vpPixelMeterConversion::convertPointWithoutDistortion(frame.cam, j, i, x, y);
          //vpColVector objectNormal({ frame.renders.normals[i][j].R, frame.renders.normals[i][j].G, frame.renders.normals[i][j].B });

          fastProjectionSurfacePoint(oMc, x * Z, y * Z, Z, point.oX);
          unsigned char luminance = previousFrame.I[i][j];
          point.targetLuminance = luminance;
          localPoints.push_back(point);
        }
      }
    }

// If we use openmp, add to the global vector. If we're not using openmp, no need to do so
#ifdef VISP_HAVE_OPENMP
#pragma omp critical
    {
      m_surfacePoints.insert(m_surfacePoints.end(), localPoints.begin(), localPoints.end());
    }
#endif
  }
  m_surfacePointsSet.build(m_surfacePoints);

  if (m_surfacePoints.size() > 0) {
    m_error.resize(m_surfacePoints.size(), false);
    m_weights.resize(m_surfacePoints.size(), false);
    m_weighted_error.resize(m_surfacePoints.size(), false);
    m_L.resize(m_surfacePoints.size(), 6, false, false);
    m_cov.resize(6, 6, false, false);
    m_covWeightDiag.resize(m_surfacePoints.size(), false);
    m_numFeatures = m_L.getRows();
  }
  else {
    m_numFeatures = 0;
  }
}

void vpRBPhotometricTracker::computeVVSIter(const vpRBFeatureTrackerInput &frame, const vpHomogeneousMatrix &cMo, unsigned int /*iteration*/)
{
  std::cerr << "Entering vvs iter" << std::endl;
  if (m_numFeatures == 0) {
    m_LTL = 0;
    m_LTR = 0;
    m_error = 0;
    m_weights = 1.0;
    m_weighted_error = 0.0;
    m_cov = 0.0;
    m_covWeightDiag = 0.0;
    return;
  }
  m_surfacePointsSet.update(cMo, frame.I, frame.cam);
  m_surfacePointsSet.errorAndInteraction(m_error, m_L);
  //m_weights = 0.0;
  m_robust.setMinMedianAbsoluteDeviation(1);
  m_robust.MEstimator(vpRobust::TUKEY, m_error, m_weights);

  for (unsigned int i = 0; i < m_surfacePoints.size(); ++i) {
    m_weighted_error[i] = m_error[i] * m_weights[i];
    m_covWeightDiag[i] = m_weights[i] * m_weights[i];
    for (unsigned int dof = 0; dof < 6; ++dof) {
      m_L[i][dof] *= m_weights[i];
    }
  }

  m_LTL = m_L.AtA();
  computeJTR(m_L, m_weighted_error, m_LTR);
  m_vvsConverged = false;
}

void vpRBPhotometricTracker::display(const vpCameraParameters &/*cam*/, const vpImage<unsigned char> &/*I*/,
                                    const vpImage<vpRGBa> &/*IRGB*/, const vpImage<unsigned char> &depth) const
{
  // switch (m_displayType) {
  // case DT_SIMPLE:
  // {
  //   for (unsigned int i = 0; i < m_surfacePoints.size(); ++i) {
  //     vpColor c(0, 255, 0);
  //     vpDisplay::displayPoint(depth, m_surfacePoints[i].pixelPos[0], m_surfacePoints[i].pixelPos[1], c, 2);
  //   }
  //   break;
  // }
  // case DT_WEIGHT:
  // {
  //   for (unsigned int i = 0; i < m_surfacePoints.size(); ++i) {
  //     vpColor c(0, static_cast<unsigned char>(m_weights[i] * 255), 0);
  //     vpDisplay::displayPoint(depth, m_surfacePoints[i].pixelPos[0], m_surfacePoints[i].pixelPos[1], c, 2);
  //   }
  //   break;
  // }
  // case DT_ERROR:
  // {
  //   double maxError = m_error.getMaxValue();
  //   for (unsigned int i = 0; i < m_surfacePoints.size(); ++i) {
  //     vpColor c(static_cast<unsigned int>((m_error[i] / maxError) * 255), 0, 0);
  //     vpDisplay::displayPoint(depth, m_surfacePoints[i].pixelPos[0], m_surfacePoints[i].pixelPos[1], c, 2);
  //   }
  //   break;
  // }
  // case DT_WEIGHT_AND_ERROR:
  // {
  //   double maxError = m_error.getMaxValue();
  //   for (unsigned int i = 0; i < m_surfacePoints.size(); ++i) {
  //     vpColor c(static_cast<unsigned int>((m_error[i] / maxError) * 255.0), static_cast<unsigned char>(m_weights[i] * 255), 0);
  //     vpDisplay::displayPoint(depth, m_surfacePoints[i].pixelPos[0], m_surfacePoints[i].pixelPos[1], c, 2);
  //   }
  //   break;
  // }
  // default:
  //   throw vpException(vpException::notImplementedError, "Depth tracker display type is invalid or not implemented");
  // }
}

END_VISP_NAMESPACE
