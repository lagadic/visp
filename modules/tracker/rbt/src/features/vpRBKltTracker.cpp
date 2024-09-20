/****************************************************************************
 *
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
*****************************************************************************/

#include <visp3/rbt/vpRBKltTracker.h>

#if defined(VP_HAVE_RB_KLT_TRACKER)

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpDisplay.h>

BEGIN_VISP_NAMESPACE

inline bool isTooCloseToBorder(unsigned int i, unsigned int j, unsigned int h, unsigned w, unsigned int border)
{
  return i < border || j < border || i >(h - border) || j >(w - border);
}

inline void vpRBKltTracker::tryAddNewPoint(const vpRBFeatureTrackerInput &frame, std::map<long, vpRBKltTracker::vpTrackedKltPoint> &points,
long id, const float u, const float v,
const vpHomogeneousMatrix &cMo, const vpHomogeneousMatrix &oMc)
{
  unsigned int uu = static_cast<unsigned int>(round(u)), uv = static_cast<unsigned int>(round(v));
  if (isTooCloseToBorder(uv, uu, frame.renders.depth.getRows(), frame.renders.depth.getCols(), m_border)) {
    return;
  }

  float Z = frame.renders.depth[uv][uu];
  if (Z <= 0.f || (frame.hasDepth() && frame.depth[uv][uu] > 0.f && fabs(frame.depth[uv][uu] - Z) > 1e-1)) {
    return;
  }
  vpRBKltTracker::vpTrackedKltPoint p;
  p.cTo0 = cMo;
  vpRGBf normalRGB = frame.renders.normals[uv][uu];
  p.normal = vpColVector({ normalRGB.R, normalRGB.G, normalRGB.B });
  double x = 0.0, y = 0.0;
  vpPixelMeterConversion::convertPoint(frame.cam, static_cast<double>(u), static_cast<double>(v), x, y);
  vpColVector oC({ x * Z, y * Z, Z, 1.0 });
  vpColVector oX = oMc * oC;
  oX /= oX[3];
  p.oX = vpPoint(oX[0], oX[1], oX[2]);
  p.currentPos = vpImagePoint(y, x);
  points[id] = p;

}

vpRBKltTracker::vpRBKltTracker() :
  vpRBFeatureTracker(), m_numPointsReinit(20), m_newPointsDistanceThreshold(5.0), m_border(5),
  m_maxErrorOutliersPixels(10.0), m_useMask(false), m_minMaskConfidence(0.0)
{

}

void vpRBKltTracker::extractFeatures(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput & /*previousFrame*/, const vpHomogeneousMatrix &cMo)
{
  m_Iprev = m_I;
  vpImageConvert::convert(frame.I, m_I);
  const vpHomogeneousMatrix oMc = cMo.inverse();
  if (m_maxErrorOutliersPixels > 0.0) {
    const double distanceThresholdPxSquare = vpMath::sqr(m_maxErrorOutliersPixels);
    const unsigned int nbFeatures = static_cast<unsigned int>(m_klt.getNbFeatures());
    // Detect outliers
    for (unsigned int i = 0; i < nbFeatures; ++i) {
      long id = 0;
      float u = 0.f, v = 0.f;

      m_klt.getFeature(i, id, u, v);
      if (m_points.find(id) != m_points.end()) {
        unsigned int uu = static_cast<unsigned int>(round(u)), uv = static_cast<unsigned int>(round(v));
        if (isTooCloseToBorder(uv, uu, frame.renders.depth.getRows(), frame.renders.depth.getCols(), m_border)) {
          return;
        }

        float Z = frame.renders.depth[uv][uu];
        if (Z > 0.f) {
          vpTrackedKltPoint &p = m_points[id];
          double x = 0.0, y = 0.0;
          vpPixelMeterConversion::convertPoint(frame.cam, static_cast<double>(u), static_cast<double>(v), x, y);
          vpColVector oXn = oMc * vpColVector({ x * Z, y * Z, Z, 1.0 });
          oXn /= oXn[3];
          p.update(cMo);
          double x1 = p.oX.get_x(), y1 = p.oX.get_y();
          double u1 = 0.0, v1 = 0.0;
          vpMeterPixelConversion::convertPoint(frame.cam, x1, y1, u1, v1);
          double distancePx = vpMath::sqr(u1 - u) + vpMath::sqr(v1 - v);

          vpColVector oX = p.oX.get_oP();
          if (distancePx > distanceThresholdPxSquare) {
            m_points.erase(id);
            m_klt.suppressFeature(i);
          }
        }
      }
    }
  }

  cv::Mat mask = cv::Mat::zeros(m_I.rows, m_I.cols, CV_8U);
  vpRect bb = frame.renders.boundingBox;
  for (unsigned int i = static_cast<unsigned int>(bb.getTop()); i < static_cast<unsigned int>(bb.getBottom()); ++i) {
    for (unsigned int j = static_cast<unsigned int>(bb.getLeft()); j < static_cast<unsigned int>(bb.getRight()); ++j) {
      mask.at<unsigned char>(i, j) = (frame.renders.depth[i][j] > 0.f) * 255;
    }
  }

  if (m_Iprev.rows > 0) {
    // Consider that there are not enough points: reinit KLT tracking
    if (m_points.size() < m_numPointsReinit) {
      m_klt.initTracking(m_Iprev, mask);
      const unsigned int nbFeatures = static_cast<unsigned int>(m_klt.getNbFeatures());
      m_points.clear();
      for (unsigned int i = 0; i < nbFeatures; ++i) {
        long id;
        float u, v;
        m_klt.getFeature(i, id, u, v);
        tryAddNewPoint(frame, m_points, id, u, v, cMo, oMc);
      }
    }
    else { // Otherwise, try and get new points
      vpKltOpencv kltTemp;
      kltTemp.setMaxFeatures(m_klt.getMaxFeatures());
      kltTemp.setWindowSize(m_klt.getWindowSize());
      kltTemp.setQuality(m_klt.getQuality());
      kltTemp.setMinDistance(m_klt.getMinDistance());
      kltTemp.setHarrisFreeParameter(m_klt.getHarrisFreeParameter());
      kltTemp.setBlockSize(m_klt.getBlockSize());
      kltTemp.setPyramidLevels(m_klt.getPyramidLevels());
      kltTemp.initTracking(m_Iprev, mask);
      const unsigned int nbFeaturesTemp = static_cast<unsigned int>(kltTemp.getNbFeatures());
      const unsigned int nbFeatures = static_cast<unsigned int>(m_klt.getNbFeatures());
      for (unsigned int i = 0; i < nbFeaturesTemp; ++i) {
        double threshold = vpMath::sqr(m_newPointsDistanceThreshold); // distance threshold, in squared pixels
        double tooClose = false;
        float u, v;
        long id;
        kltTemp.getFeature(i, id, u, v);
        for (unsigned int j = 0; j < nbFeatures; ++j) {
          float uj, vj;
          long idj;
          m_klt.getFeature(j, idj, uj, vj);
          if (vpMath::sqr(uj - u) + vpMath::sqr(vj - v) < threshold) {
            tooClose = true;
            break;
          }
        }
        if (tooClose) {
          continue;
        }

        m_klt.addFeature(u, v);
        const std::vector<long> &ids = m_klt.getFeaturesId();
        id = ids[ids.size() - 1];
        tryAddNewPoint(frame, m_points, id, u, v, cMo, oMc);

      }
    }
  }
  else {
    m_klt.initTracking(m_I, mask);
    m_points.clear();
    const unsigned int nbFeatures = static_cast<unsigned int>(m_klt.getNbFeatures());
    for (unsigned int i = 0; i < nbFeatures; ++i) {
      long id;
      float u, v;
      m_klt.getFeature(i, id, u, v);
      tryAddNewPoint(frame, m_points, id, u, v, cMo, oMc);
    }
  }
}

void vpRBKltTracker::trackFeatures(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &/*previousFrame*/, const vpHomogeneousMatrix &cMo)
{
  unsigned int nbKltFeatures = static_cast<unsigned int>(m_klt.getNbFeatures());
  if (nbKltFeatures > 0) {
    m_klt.track(m_I);
  }
  std::map<long, vpTrackedKltPoint> newPoints;
  const vpHomogeneousMatrix oMc = cMo.inverse();

  bool testMask = m_useMask && frame.hasMask();
  nbKltFeatures = static_cast<unsigned int>(m_klt.getNbFeatures());

  for (unsigned int i = 0; i < nbKltFeatures; ++i) {
    long id = 0;
    float u = 0.f, v = 0.f;
    double x = 0.0, y = 0.0;
    m_klt.getFeature(i, id, u, v);
    unsigned int uu = static_cast<unsigned int>(round(u)), uv = static_cast<unsigned int>(round(v));
    // Filter points that are too close to image borders and cannot be reliably tracked
    if (isTooCloseToBorder(uv, uu, frame.renders.depth.getRows(), frame.renders.depth.getCols(), m_border)) {
      continue;
    }
    float Z = frame.renders.depth[uv][uu];
    if (Z <= 0.f) {
      continue;
    }

    if (testMask && frame.mask[uv][uu] < m_minMaskConfidence) {
      continue;
    }

    if (m_points.find(id) != m_points.end()) {
      vpTrackedKltPoint &p = m_points[id];
      if (p.rotationDifferenceToInitial(oMc) > vpMath::rad(45.0) && p.normalDotProd(cMo) < cos(vpMath::rad(70))) {
        continue;
      }
      vpPixelMeterConversion::convertPoint(frame.cam, static_cast<double>(u), static_cast<double>(v), x, y);
      p.currentPos = vpImagePoint(y, x);
      newPoints[id] = p;
    }
    // float Z = frame.renders.depth[uv][uu];
    // if (Z > 0.f && m_points.find(id) != m_points.end()) {
    //   vpTrackedKltPoint &p = m_points[id];
    //   if (p.rotationDifferenceToInitial(oMc) > vpMath::rad(30.0)) {
    //     continue;
    //   }
    //   vpPixelMeterConversion::convertPoint(frame.cam, static_cast<double>(u), static_cast<double>(v), x, y);
    //   p.currentPos = vpImagePoint(y, x);
    //   newPoints[id] = p;
    // }
  }

  m_points = newPoints;
  m_numFeatures = m_points.size() * 2;
}

void vpRBKltTracker::initVVS(const vpRBFeatureTrackerInput &/*frame*/, const vpRBFeatureTrackerInput &/*previousFrame*/, const vpHomogeneousMatrix & /*cMo*/)
{
  if (m_numFeatures < m_numPointsReinit * 2) {
    m_numFeatures = 0;
    return;
  }
  m_L.resize(m_numFeatures, 6, false, false);
  m_error.resize(m_numFeatures, false);
  m_weighted_error.resize(m_numFeatures, false);
  m_weights.resize(m_numFeatures, false);
  m_LTL.resize(6, 6, false, false);
  m_LTR.resize(6, false);
  m_cov.resize(6, 6, false, false);
  m_covWeightDiag.resize(m_numFeatures, false);
  m_error = 0;
}

void vpRBKltTracker::computeVVSIter(const vpRBFeatureTrackerInput &/*frame*/, const vpHomogeneousMatrix &cMo, unsigned int /*iteration*/)
{
  if (m_numFeatures < m_numPointsReinit * 2) {
    m_LTL = 0;
    m_LTR = 0;
    m_error = 0;
    return;
  }
  unsigned int pointIndex = 0;

  for (std::pair<const long, vpTrackedKltPoint> &p : m_points) {
    p.second.update(cMo);
    p.second.interaction(m_L, pointIndex);
    p.second.error(m_error, pointIndex);
    ++pointIndex;
  }

  //m_robust.setMinMedianAbsoluteDeviation(2.0 / frame.cam.get_px());
  m_robust.MEstimator(vpRobust::TUKEY, m_error, m_weights);
  for (unsigned int i = 0; i < m_error.getRows(); ++i) {
    m_weighted_error[i] = m_error[i] * m_weights[i];
    m_covWeightDiag[i] = m_weights[i] * m_weights[i];
    for (unsigned int dof = 0; dof < 6; ++dof) {
      m_L[i][dof] *= m_weights[i];
    }
  }

  m_LTL = m_L.AtA();
  computeJTR(m_L, m_weighted_error, m_LTR);
}


void vpRBKltTracker::display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpImage<vpRGBa> &/*IRGB*/, const vpImage<unsigned char> &/*depth*/, const vpRBFeatureDisplayType /*type*/) const
{
  for (const std::pair<const long, vpTrackedKltPoint> &p : m_points) {
    double u = 0.0, v = 0.0;
    vpMeterPixelConversion::convertPoint(cam, p.second.currentPos.get_j(), p.second.currentPos.get_i(), u, v);
    vpDisplay::displayPoint(I, v, u, vpColor::red, 2);
  }
}

#endif

END_VISP_NAMESPACE
