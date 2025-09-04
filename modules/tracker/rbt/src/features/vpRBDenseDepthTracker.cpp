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

#include <visp3/rbt/vpRBDenseDepthTracker.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpDisplay.h>

#ifdef VISP_HAVE_OPENMP
#include <omp.h>
#endif
BEGIN_VISP_NAMESPACE

// #define VISP_DEBUG_RB_DEPTH_DENSE_TRACKER 1


void fastProjection(const vpHomogeneousMatrix &oTc, double X, double Y, double Z, std::array<double, 3> &p)
{
  const double *T = oTc.data;
  p[0] = (T[0] * X + T[1] * Y + T[2] * Z + T[3]);
  p[1] = (T[4] * X + T[5] * Y + T[6] * Z + T[7]);
  p[2] = (T[8] * X + T[9] * Y + T[10] * Z + T[11]);
}

double dotProd3(const vpColVector &a, const std::array<double, 3> &b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}


void vpRBDenseDepthTracker::extractFeatures(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &/*previousFrame*/, const vpHomogeneousMatrix &/*cMo*/)
{
  const vpImage<float> &depthMap = frame.depth;
  const vpImage<float> &renderDepth = frame.renders.depth;
  const vpRect bb = frame.renders.boundingBox;
  const vpHomogeneousMatrix &cMo = frame.renders.cMo;
  const vpRotationMatrix cRo = cMo.getRotationMatrix();
  const vpHomogeneousMatrix oMc = cMo.inverse();
  const vpTranslationVector co = oMc.getTranslationVector(); // Position of the camera in object frame
  const bool useMask = m_useMask && frame.hasMask();
  m_depthPoints.clear();
  m_depthPoints.reserve(static_cast<size_t>(bb.getArea() / (m_step * m_step * 2)));

  std::vector<std::vector<vpDepthPoint>> pointsPerThread;

#ifdef VISP_HAVE_OPENMP
#pragma omp parallel
#endif
  {
#ifdef VISP_HAVE_OPENMP
#pragma omp single
    {
      unsigned int numThreads = omp_get_num_threads();
      pointsPerThread.resize(numThreads);
    }
#else
    {
      pointsPerThread.resize(1);
    }
#endif

#ifdef VISP_HAVE_OPENMP
    unsigned int threadIdx = omp_get_thread_num();
#else
    unsigned int threadIdx = 0;
#endif
    vpDepthPoint point;
    vpColVector cameraRay(3);
#ifdef VISP_HAVE_OPENMP
    std::vector<vpDepthPoint> localPoints;
    localPoints.reserve(m_depthPoints.capacity() / omp_get_num_threads());
#else
    // Just reference the global vector, no need to add to it later on
    std::vector<vpDepthPoint> &localPoints = m_depthPoints;
#endif

#ifdef VISP_HAVE_OPENMP
#pragma omp for nowait
#endif
    for (auto i = static_cast<int>(bb.getTop()); i < static_cast<int>(bb.getBottom()); i += m_step) {
      for (auto j = static_cast<int>(bb.getLeft()); j < static_cast<int>(bb.getRight()); j += m_step) {
        const double Z = renderDepth[i][j];
        const double currZ = depthMap[i][j];

        if (Z > 0.f && currZ > 0.f) {
          if (useMask && frame.mask[i][j] < m_minMaskConfidence) {
            continue;
          }
          double x = 0.0, y = 0.0;
          vpPixelMeterConversion::convertPointWithoutDistortion(frame.cam, j, i, x, y);
          //vpColVector objectNormal({ frame.renders.normals[i][j].R, frame.renders.normals[i][j].G, frame.renders.normals[i][j].B });
          point.objectNormal[0] = frame.renders.normals[i][j].R;
          point.objectNormal[1] = frame.renders.normals[i][j].G;
          point.objectNormal[2] = frame.renders.normals[i][j].B;

          bool invalidNormal = false;
          for (unsigned int oi = 0; oi < 3; ++oi) {
            if (!vpMath::isFinite(point.objectNormal[oi]) || abs(point.objectNormal[oi]) > 1.0) {
              invalidNormal = true;
            }
          }

          if (invalidNormal) {
            continue;
          }

          fastProjection(oMc, x * Z, y * Z, Z, point.oX);

          cameraRay = { co[0] - point.oX[0], co[1] - point.oX[1], co[2] - point.oX[2] };
          cameraRay.normalize();

          if (acos(dotProd3(cameraRay, point.objectNormal)) > vpMath::rad(85.0)) {
            continue;
          }

          point.pixelPos[0] = i;
          point.pixelPos[1] = j;

          point.observation[0] = x * currZ;
          point.observation[1] = y * currZ;
          point.observation[2] = currZ;

          localPoints.push_back(point);
        }
      }
    }

    pointsPerThread[threadIdx] = std::move(localPoints);
  }
  for (const std::vector<vpDepthPoint> &points: pointsPerThread) {
    m_depthPoints.insert(m_depthPoints.end(), std::make_move_iterator(points.begin()), std::make_move_iterator(points.end()));
  }
  m_depthPointSet.build(m_depthPoints);

  if (m_depthPoints.size() > 0) {
    m_error.resize(m_depthPoints.size(), false);
    m_weights.resize(m_depthPoints.size(), false);
    m_weighted_error.resize(m_depthPoints.size(), false);
    m_L.resize(m_depthPoints.size(), 6, false, false);
    m_cov.resize(6, 6, false, false);
    m_covWeightDiag.resize(m_depthPoints.size(), false);
    m_numFeatures = m_L.getRows();
  }
  else {
    m_numFeatures = 0;
  }
}

void vpRBDenseDepthTracker::computeVVSIter(const vpRBFeatureTrackerInput &frame, const vpHomogeneousMatrix &cMo, unsigned int /*iteration*/)
{
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

  m_depthPointSet.updateAndErrorAndInteractionMatrix(frame.cam, cMo, frame.depth, m_error, m_L);

  m_robust.setMinMedianAbsoluteDeviation((frame.renders.zFar - frame.renders.zNear) * 0.01);
  m_robust.MEstimator(vpRobust::TUKEY, m_error, m_weights);



  for (unsigned int i = 0; i < m_depthPoints.size(); ++i) {
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

void vpRBDenseDepthTracker::display(const vpCameraParameters &/*cam*/, const vpImage<unsigned char> &/*I*/,
                                    const vpImage<vpRGBa> &/*IRGB*/, const vpImage<unsigned char> &depth) const
{
  switch (m_displayType) {
  case DT_SIMPLE:
  {
    for (unsigned int i = 0; i < m_depthPoints.size(); ++i) {
      vpColor c(0, 255, 0);
      vpDisplay::displayPoint(depth, m_depthPoints[i].pixelPos[0], m_depthPoints[i].pixelPos[1], c, 2);
    }
    break;
  }
  case DT_WEIGHT:
  {
    for (unsigned int i = 0; i < m_depthPoints.size(); ++i) {
      vpColor c(0, static_cast<unsigned char>(m_weights[i] * 255), 0);
      vpDisplay::displayPoint(depth, m_depthPoints[i].pixelPos[0], m_depthPoints[i].pixelPos[1], c, 2);
    }
    break;
  }
  case DT_ERROR:
  {
    double maxError = m_error.getMaxValue();
    for (unsigned int i = 0; i < m_depthPoints.size(); ++i) {
      vpColor c(static_cast<unsigned int>((m_error[i] / maxError) * 255), 0, 0);
      vpDisplay::displayPoint(depth, m_depthPoints[i].pixelPos[0], m_depthPoints[i].pixelPos[1], c, 2);
    }
    break;
  }
  case DT_WEIGHT_AND_ERROR:
  {
    double maxError = m_error.getMaxValue();
    for (unsigned int i = 0; i < m_depthPoints.size(); ++i) {
      vpColor c(static_cast<unsigned int>((m_error[i] / maxError) * 255.0), static_cast<unsigned char>(m_weights[i] * 255), 0);
      vpDisplay::displayPoint(depth, m_depthPoints[i].pixelPos[0], m_depthPoints[i].pixelPos[1], c, 2);
    }
    break;
  }
  default:
    throw vpException(vpException::notImplementedError, "Depth tracker display type is invalid or not implemented");
  }
}

END_VISP_NAMESPACE
