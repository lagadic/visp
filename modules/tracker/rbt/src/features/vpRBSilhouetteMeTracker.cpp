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

#include <visp3/rbt/vpRBSilhouetteMeTracker.h>

#define VISP_DEBUG_ME_TRACKER 0

BEGIN_VISP_NAMESPACE

/**
 * @brief Extract the geometric features from the list of collected silhouette points
*/
void vpRBSilhouetteMeTracker::extractFeatures(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &previousFrame, const vpHomogeneousMatrix &/*cMo*/)
{
  m_controlPoints.clear();
  m_controlPoints.reserve(frame.silhouettePoints.size());
  const vpHomogeneousMatrix &cMo = frame.renders.cMo;
  const vpHomogeneousMatrix oMc = cMo.inverse();
  const vpColVector oC = oMc.getRotationMatrix() * vpColVector({ 0.0, 0.0, -1.0 });
  const vpImage<unsigned char> &initImage = previousFrame.I.getSize() == frame.I.getSize() ? previousFrame.I : frame.I;

#ifdef VISP_HAVE_OPENMP
#pragma omp parallel
#endif
  {
    std::vector<vpRBSilhouetteControlPoint> localPoints;
#ifdef VISP_HAVE_OPENMP
#pragma omp for nowait
#endif
    for (int i = 0; i < static_cast<int>(frame.silhouettePoints.size()); ++i) {
      const vpRBSilhouettePoint &sp = frame.silhouettePoints[i];
      // float angle = vpMath::deg(acos(sp.normal * oC));
      // if (angle > 89.0) {
      //   continue;
      // }
      // std::cout <<  angle << std::endl;
#if VISP_DEBUG_ME_TRACKER
      if (sp.Z == 0) {
        throw vpException(vpException::badValue, "Got a point with Z == 0");
      }
      if (std::isnan(sp.orientation)) {
        throw vpException(vpException::badValue, "Got a point with theta nan");
      }
#endif
      vpRBSilhouetteControlPoint p;
      p.buildPoint(static_cast<int>(sp.i), static_cast<int>(sp.j), sp.Z, sp.orientation, sp.normal, cMo, oMc, frame.cam, m_me, sp.isSilhouette);
      if (p.tooCloseToBorder(frame.I.getHeight(), frame.I.getWidth(), m_me.getRange())) {
        continue;
      }
      if (m_useMask && frame.hasMask()) {
        double maxMaskGradient;
        if (p.isSilhouette()) { // If it is a silhouette point, we check that the mask actually considers it an object border
          maxMaskGradient = p.getMaxMaskGradientAlongLine(frame.mask, m_me.getRange());
        }
        else { // Otherwise, we just check that the site is considered as belonging to the object
          maxMaskGradient = frame.mask[sp.i][sp.j];
        }
        if (maxMaskGradient < m_minMaskConfidence) {
          continue;
        }
      }

      p.initControlPoint(initImage, 0);
      p.setNumCandidates(m_numCandidates);
      localPoints.push_back(std::move(p));
    }

#ifdef VISP_HAVE_OPENMP
#pragma omp critical
#endif
    {
      m_controlPoints.insert(m_controlPoints.end(), localPoints.begin(), localPoints.end());
    }
  }

  m_numFeatures = static_cast<unsigned int>(m_controlPoints.size());

  m_robust.setMinMedianAbsoluteDeviation(m_robustMadMin / frame.cam.get_px());
}

void vpRBSilhouetteMeTracker::trackFeatures(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &/*previousFrame*/, const vpHomogeneousMatrix &/*cMo*/)
{
  if (m_numCandidates <= 1) {
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < static_cast<int>(m_controlPoints.size()); ++i) {
      m_controlPoints[i].track(frame.I);
    }
  }
  else {
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < static_cast<int>(m_controlPoints.size()); ++i) {
      m_controlPoints[i].trackMultipleHypotheses(frame.I);
    }
  }
}

void vpRBSilhouetteMeTracker::initVVS(const vpRBFeatureTrackerInput & /*frame*/, const vpRBFeatureTrackerInput & /*previousFrame*/, const vpHomogeneousMatrix & /*cMo*/)
{
  if (m_numFeatures == 0) {
    return;
  }

  m_weighted_error.resize(m_numFeatures, false);
  m_weights.resize(m_numFeatures, false);
  m_weights = 0;
  m_L.resize(m_numFeatures, 6, false, false);
  m_covWeightDiag.resize(m_numFeatures, false);
  m_vvsConverged = false;
  m_error.resize(m_numFeatures, false);
}

void vpRBSilhouetteMeTracker::computeVVSIter(const vpRBFeatureTrackerInput &frame, const vpHomogeneousMatrix &cMo, unsigned int /*iteration*/)
{
  vpColVector factor(m_numFeatures, 1.0);
  const double threshold = m_singlePointConvergedThresholdPixels / frame.cam.get_px(); //Transformation limite pixel en limite metre.

  unsigned count = 0;
  unsigned countValidSites = 0;

  for (unsigned int k = 0; k < m_controlPoints.size(); k++) {
    vpRBSilhouetteControlPoint &p = m_controlPoints[k];
    //p.update(cMo);
    if (m_numCandidates <= 1) {
      p.computeMeInteractionMatrixError(cMo, k, m_L, m_error);
    }
    else {
      p.computeMeInteractionMatrixErrorMH(cMo, k, m_L, m_error);
    }

    m_weights[k] = 1;
    if (!p.siteIsValid() || !p.isValid()) {
      factor[k] = 0.0;
      for (unsigned int j = 0; j < 6; j++) {
        m_L[k][j] = 0;
      }
    }
    else {
      countValidSites++;
      if (m_error[k] <= threshold) {
        ++count;
      }
    }
  }

  if (countValidSites == 0) {
    m_vvsConverged = false;
  }
  else {
    const double percentageConverged = static_cast<double>(count) / static_cast<double>(countValidSites);
    if (percentageConverged < m_globalVVSConvergenceThreshold) {
      m_vvsConverged = false;
    }
    else {
      m_vvsConverged = true;
    }
  }

  if (m_numFeatures == 0) {
    return;
  }

  m_robust.MEstimator(vpRobust::TUKEY, m_error, m_weights);

  for (unsigned int i = 0; i < m_error.size(); i++) {
    const double wi = m_weights[i] * factor[i];
    const double eri = m_error[i];
    m_covWeightDiag[i] = wi * wi;
    m_weighted_error[i] = wi * eri;
    for (unsigned int j = 0; j < 6; j++) {
      m_L[i][j] = wi * m_L[i][j];
    }
  }

  m_LTL = m_L.AtA();
  computeJTR(m_L, m_weighted_error, m_LTR);

#if VISP_DEBUG_ME_TRACKER
  for (unsigned int i = 0; i < 6; ++i) {
    if (std::isnan(m_LTR[i])) {
      std::cerr << m_L << std::endl;
      throw vpException(vpException::badValue, "Some components were nan in ME tracker computation");
    }
  }
#endif
}

void vpRBSilhouetteMeTracker::display(const vpCameraParameters &/*cam*/, const vpImage<unsigned char> &I,
                                      const vpImage<vpRGBa> &/*IRGB*/, const vpImage<unsigned char> &/*depth*/) const
{

  for (const vpRBSilhouetteControlPoint &p: m_controlPoints) {
    const vpMeSite &s = p.getSite();
    s.display(I);
  }

}

END_VISP_NAMESPACE
