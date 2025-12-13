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

#include <visp3/rbt/vpRBDenseDepthTracker.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpDisplay.h>

#include "../core/private/vpSIMDUtils.h"

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
    m_error.resize(static_cast<unsigned int>(m_depthPoints.size()), false);
    m_weights.resize(static_cast<unsigned int>(m_depthPoints.size()), false);
    m_weighted_error.resize(static_cast<unsigned int>(m_depthPoints.size()), false);
    m_L.resize(static_cast<unsigned int>(m_depthPoints.size()), 6, false, false);
    m_cov.resize(6, 6, false, false);
    m_covWeightDiag.resize(static_cast<unsigned int>(m_depthPoints.size()), false);
    m_numFeatures = m_L.getRows();
  }
  else {
    m_numFeatures = 0;
  }
}

void vpRBDenseDepthTracker::initVVS(const vpRBFeatureTrackerInput &/*frame*/, const vpRBFeatureTrackerInput &/*previousFrame*/, const vpHomogeneousMatrix &/*cMo*/)
{
  m_LTL.resize(6, 6, false, false);
  m_LTR.resize(6, false);
  m_L.resize(m_numFeatures, 6, false, false);
  m_Lt.resize(6, m_numFeatures, false, false);
  m_error.resize(m_numFeatures, false);
  m_weighted_error.resize(m_numFeatures, false);
  m_weights.resize(m_numFeatures, false);
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

  m_depthPointSet.updateAndErrorAndInteractionMatrix(frame.cam, cMo, frame.depth, m_error, m_Lt);
  m_Lt.transpose(m_L);

  m_robust.setMinMedianAbsoluteDeviation((frame.renders.zFar - frame.renders.zNear) * 0.01);
  m_robust.MEstimator(vpRobust::TUKEY, m_error, m_weights);

// #if defined(VISP_HAVE_OPENMP)
// #pragma omp parallel for
// #endif
  for (unsigned int i = 0; i < m_depthPoints.size(); ++i) {
    m_weighted_error[i] = m_error[i] * m_weights[i];
    m_covWeightDiag[i] = m_weights[i] * m_weights[i];
    for (unsigned int dof = 0; dof < 6; ++dof) {
      m_L[i][dof] *= m_weights[i];
    }
  }
  m_LTL.resize(6, 6);
  m_L.AtA(m_LTL);
  computeJTR(m_L, m_weighted_error, m_LTR);
  m_vvsConverged = false;
}

void errorAndInteractionMatrixBase(const vpMatrix &cXt, const vpMatrix &cNt, const vpMatrix &obsT, vpColVector &e, vpMatrix &Lt, unsigned int start, unsigned int end)
{
#if defined(VISP_HAVE_OPENMP)
#pragma omp parallel for
#endif
  for (unsigned int i = start; i < end; ++i) {
    const double X = cXt[0][i], Y = cXt[1][i], Z = cXt[2][i];
    const double nX = cNt[0][i], nY = cNt[1][i], nZ = cNt[2][i];

    const double D = -((nX * X) + (nY  * Y) + (nZ * Z));
    const double projNormal = nX * obsT[0][i] + nY  * obsT[1][i] + nZ * obsT[2][i];

    e[i] = D + projNormal;

    Lt[0][i] = nX;
    Lt[1][i] = nY;
    Lt[2][i] = nZ;
    Lt[3][i] = nZ * Y - nY * Z;
    Lt[4][i] = nX * Z - nZ * X;
    Lt[5][i] = nY * X - nX * Y;
  }
}

#if defined(VISP_HAVE_AVX2) || defined(VISP_HAVE_AVX) || defined(VISP_HAVE_SSE2)
void errorAndInteractionMatrixSIMD(const vpMatrix &cXt, const  vpMatrix &cNt, const  vpMatrix &obsT, vpColVector &e, vpMatrix &Lt)
{
#if defined(VISP_HAVE_OPENMP)
#pragma omp parallel for schedule(static, 1024)
#endif
  for (int i = 0; i <= static_cast<int>(cXt.getCols()) - vpSIMD::numLanes; i += vpSIMD::numLanes) {
    const vpSIMD::Register X = vpSIMD::loadu(cXt[0] + i), Y = vpSIMD::loadu(cXt[1] + i), Z = vpSIMD::loadu(cXt[2] + i);
    const vpSIMD::Register nX = vpSIMD::loadu(cNt[0] + i), nY = vpSIMD::loadu(cNt[1] + i), nZ = vpSIMD::loadu(cNt[2] + i);
    const vpSIMD::Register obsX = vpSIMD::loadu(obsT[0] + i), obsY = vpSIMD::loadu(obsT[1] + i), obsZ = vpSIMD::loadu(obsT[2] + i);

    vpSIMD::Register D = vpSIMD::mul(nX, X);
    vpSIMD::Register projNormal = vpSIMD::mul(obsX, X);
    D = vpSIMD::fma(nY, Y, D);
    D = vpSIMD::fma(nZ, Z, D);

    projNormal = vpSIMD::fma(nY, obsY, projNormal);
    projNormal = vpSIMD::fma(nZ, obsZ, projNormal);

    vpSIMD::storeu(e.data + i, vpSIMD::sub(projNormal, D));

    vpSIMD::storeu(Lt[0] + i, nX);
    vpSIMD::storeu(Lt[1] + i, nY);
    vpSIMD::storeu(Lt[2] + i, nZ);

    vpSIMD::storeu(Lt[3] + i, vpSIMD::sub(vpSIMD::mul(nZ, Y), vpSIMD::mul(nY, Z)));
    vpSIMD::storeu(Lt[4] + i, vpSIMD::sub(vpSIMD::mul(nX, Z), vpSIMD::mul(nZ, X)));
    vpSIMD::storeu(Lt[5] + i, vpSIMD::sub(vpSIMD::mul(nY, X), vpSIMD::mul(nX, Y)));
  }
  errorAndInteractionMatrixBase(cXt, cNt, obsT, e, Lt, (cXt.getCols() / vpSIMD::numLanes) * vpSIMD::numLanes, cXt.getCols());
}
#endif

void vpRBDenseDepthTracker::vpDepthPointSet::updateAndErrorAndInteractionMatrix(
  const vpCameraParameters &cam, const vpHomogeneousMatrix &cMo,
  const vpImage<float> &depth, vpColVector &e, vpMatrix &Lt)
{
  const vpRotationMatrix cRo = cMo.getRotationMatrix();
  cMo.project(m_oXt, m_cXt, false);
  cRo.rotateVectors(m_oNt, m_cNt, false);
  m_invalid.clear();

  const unsigned int numPoints = m_oXt.getCols();
  e.resize(numPoints, false);
  Lt.resize(6, numPoints, false, false);
#if defined(VISP_HAVE_OPENMP)
#pragma omp parallel
#endif
  {
    std::vector<unsigned int> localInvalid;
#if defined(VISP_HAVE_OPENMP)
#pragma omp for
#endif

    for (int i = 0; i < static_cast<int>(numPoints); ++i) {
      //Step 1: update and filter out points that are no longer valid
      {
        // Plane points away from the camera: this surface is no longer visible due to rotation
        if (m_cNt[2][i] >= 0.0) {
          localInvalid.push_back(i);
          continue;
        }
        double x, y, u, v;
        x = m_cXt[0][i] / m_cXt[2][i];
        y = m_cXt[1][i] / m_cXt[2][i];
        vpMeterPixelConversion::convertPointWithoutDistortion(cam, x, y, u, v);
        // Point is no longer in image: depth value cannot be sampled
        if (u < 0 || v < 0 || u >= depth.getWidth() || v >= depth.getHeight()) {
          localInvalid.push_back(i);
          continue;
        }
        const double Z = depth[static_cast<unsigned int>(v)][static_cast<unsigned int>(u)];
        // Z value in the depth image from the camera is invalid
        if (Z <= 0.0) {
          localInvalid.push_back(i);
          continue;
        }
        m_observations[0][i] = x * Z;
        m_observations[1][i] = y * Z;
        m_observations[2][i] = Z;
      }
    }
#if defined(VISP_HAVE_OPENMP)
#pragma omp critical
    {
      m_invalid.insert(m_invalid.end(), localInvalid.begin(), localInvalid.end());
    }
#else
    m_invalid = std::move(localInvalid);
#endif
  }

#if defined(VISP_HAVE_AVX2) || defined(VISP_HAVE_AVX) || defined(VISP_HAVE_SSE2)
  errorAndInteractionMatrixSIMD(m_cXt, m_cNt, m_observations, e, Lt);
#else
  errorAndInteractionMatrixBase(m_cXt, m_cNt, m_observations, e, Lt, 0, numPoints);
#endif
  // Disable invalid points
  for (unsigned int i : m_invalid) {
    e[i] = 0.0;

    Lt[0][i] = 0;
    Lt[1][i] = 0;
    Lt[2][i] = 0;
    Lt[3][i] = 0;
    Lt[4][i] = 0;
    Lt[5][i] = 0;
  }
}

void vpRBDenseDepthTracker::display(const vpCameraParameters &/*cam*/, const vpImage<unsigned char> &/*I*/,
                                    const vpImage<vpRGBa> &/*IRGB*/, const vpImage<unsigned char> &depth) const
{
  switch (m_displayType) {
  case DT_SIMPLE:
  {
    for (unsigned int i = 0; i < m_depthPoints.size(); ++i) {
      vpColor c(0, 255, 0);
      vpDisplay::displayPoint(depth, m_depthPoints[i].pixelPos[0], m_depthPoints[i].pixelPos[1], c, 1);
    }
    break;
  }
  case DT_WEIGHT:
  {
    for (unsigned int i = 0; i < m_depthPoints.size(); ++i) {
      vpColor c(0, static_cast<unsigned char>(m_weights[i] * 255), 0);
      vpDisplay::displayPoint(depth, m_depthPoints[i].pixelPos[0], m_depthPoints[i].pixelPos[1], c, 1);
    }
    break;
  }
  case DT_ERROR:
  {
    double maxError = m_error.getMaxValue();
    for (unsigned int i = 0; i < m_depthPoints.size(); ++i) {
      vpColor c(static_cast<unsigned int>((m_error[i] / maxError) * 255), 0, 0);
      vpDisplay::displayPoint(depth, m_depthPoints[i].pixelPos[0], m_depthPoints[i].pixelPos[1], c, 1);
    }
    break;
  }
  case DT_WEIGHT_AND_ERROR:
  {
    double maxError = m_error.getMaxValue();
    for (unsigned int i = 0; i < m_depthPoints.size(); ++i) {
      vpColor c(static_cast<unsigned int>((m_error[i] / maxError) * 255.0), static_cast<unsigned char>(m_weights[i] * 255), 0);
      vpDisplay::displayPoint(depth, m_depthPoints[i].pixelPos[0], m_depthPoints[i].pixelPos[1], c, 1);
    }
    break;
  }
  default:
    throw vpException(vpException::notImplementedError, "Depth tracker display type is invalid or not implemented");
  }
}

END_VISP_NAMESPACE
