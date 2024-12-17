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
BEGIN_VISP_NAMESPACE

void fastRotationMatmul(const vpRotationMatrix &cRo, const vpRGBf &v, vpColVector &res)
{
  res.resize(3, false);
  const double r = static_cast<double>(v.R), g = static_cast<double>(v.G), b = static_cast<double>(v.B);
  const double *R = cRo.data;
  res[0] = R[0] * r + R[1] * g + R[2] * b;
  res[1] = R[3] * r + R[4] * g + R[5] * b;
  res[2] = R[6] * r + R[7] * g + R[8] * b;
}

void fastProjection(const vpHomogeneousMatrix &oTc, double X, double Y, double Z, vpPoint &p)
{
  const double *T = oTc.data;
  p.set_oX(T[0] * X + T[1] * Y + T[2] * Z + T[3]);
  p.set_oY(T[4] * X + T[5] * Y + T[6] * Z + T[7]);
  p.set_oZ(T[8] * X + T[9] * Y + T[10] * Z + T[11]);
  p.set_oW(1.0);
}

void vpRBDenseDepthTracker::extractFeatures(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &/*previousFrame*/, const vpHomogeneousMatrix &/*cMo*/)
{
  const vpImage<float> &depthMap = frame.depth;
  const vpImage<float> &renderDepth = frame.renders.depth;
  vpRect bb = frame.renders.boundingBox;
  const vpHomogeneousMatrix &cMo = frame.renders.cMo;
  vpHomogeneousMatrix oMc = cMo.inverse();
  vpRotationMatrix cRo = cMo.getRotationMatrix();
  bool useMask = m_useMask && frame.hasMask();
  m_depthPoints.clear();
  m_depthPoints.reserve(static_cast<size_t>(bb.getArea() / (m_step * m_step * 2)));
  vpDepthPoint point;
  for (unsigned int i = static_cast<unsigned int>(bb.getTop()); i < static_cast<unsigned int>(bb.getBottom()); i += m_step) {
    for (unsigned int j = static_cast<unsigned int>(bb.getLeft()); j < static_cast<unsigned int>(bb.getRight()); j += m_step) {
      // if (renderDepth[i][j] > frame.renders.zNear && renderDepth[i][j] < frame.renders.zFar && depthMap[i][j] > frame.renders.zNear * 0.33 && depthMap[i][j] < frame.renders.zFar * 3.0) {
      double Z = renderDepth[i][j];
      double currZ = depthMap[i][j];
      if (Z > 0.f && currZ > 0.f) {
        if (useMask && frame.mask[i][j] < m_minMaskConfidence) {
          continue;
        }
        double x = 0.0, y = 0.0;
        vpPixelMeterConversion::convertPoint(frame.cam, j, i, x, y);
        //vpColVector objectNormal({ frame.renders.normals[i][j].R, frame.renders.normals[i][j].G, frame.renders.normals[i][j].B });
        point.objectNormal[0] = frame.renders.normals[i][j].R;
        point.objectNormal[1] = frame.renders.normals[i][j].G;
        point.objectNormal[2] = frame.renders.normals[i][j].B;

        //fastRotationMatmul(cRo, frame.renders.normals[i][j], point.objectNormal);
        //vpColVector cameraNormal = cRo * objectNormal;
        // if (acos(cameraNormal * vpColVector({ 0.0, 0.0, -1.0 })) > vpMath::rad(70.0)) {
        //   continue;
        // }
        // vpColVector cp({ x * Z, y * Z, Z, 1 });
        // vpColVector oP = oMc * cp;
        fastProjection(oMc, x * Z, y * Z, Z, point.oP);
        // point.oP = vpPoint(oP);
        point.pixelPos.set_ij(i, j);
        point.currentPoint[0] = x * currZ;
        point.currentPoint[1] = y * currZ;
        point.currentPoint[2] = currZ;

        m_depthPoints.push_back(point);
      }
    }
  }
  if (m_depthPoints.size() > 0) {
    m_error.resize(m_depthPoints.size(), false);
    m_weights.resize(m_depthPoints.size(), false);
    m_weighted_error.resize(m_depthPoints.size(), false);
    m_L.resize(m_depthPoints.size(), 6, false, false);
    m_numFeatures = m_L.getRows();
    m_cov.resize(6, 6, false, false);
    m_covWeightDiag.resize(m_depthPoints.size(), false);
  }
  else {
    m_numFeatures = 0;
  }
}

void vpRBDenseDepthTracker::computeVVSIter(const vpRBFeatureTrackerInput &/*frame*/, const vpHomogeneousMatrix &cMo, unsigned int /*iteration*/)
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
  vpRotationMatrix cRo = cMo.getRotationMatrix();
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
  for (unsigned int i = 0; i < m_depthPoints.size(); ++i) {
    vpDepthPoint &depthPoint = m_depthPoints[i];
    depthPoint.update(cMo, cRo);
    depthPoint.error(m_error, i);
    depthPoint.interaction(m_L, i);
  }

  //m_weights = 0.0;
  //m_robust.setMinMedianAbsoluteDeviation(1e-3);
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
                                    const vpImage<vpRGBa> &/*IRGB*/, const vpImage<unsigned char> &depth,
                                    const vpRBFeatureDisplayType /*type*/) const
{
  for (unsigned int i = 0; i < m_depthPoints.size(); ++i) {
    const vpDepthPoint &p = m_depthPoints[i];
    vpColor c(0, static_cast<unsigned char>(m_weights[i] * 255), 0);
    vpDisplay::displayPoint(depth, p.pixelPos, c, 2);
  }

}

END_VISP_NAMESPACE
