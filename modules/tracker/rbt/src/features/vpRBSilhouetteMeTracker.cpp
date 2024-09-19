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

#include <visp3/rbt/vpRBSilhouetteMeTracker.h>

#define VISP_DEBUG_ME_TRACKER 0


BEGIN_VISP_NAMESPACE


/**
 * @brief Extract the geometric features from the list of collected silhouette points
*/
void vpRBSilhouetteMeTracker::extractFeatures(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &previousFrame, const vpHomogeneousMatrix &cMo)
{
  m_controlPoints.clear();
  m_controlPoints.reserve(frame.silhouettePoints.size());
  const vpHomogeneousMatrix oMc = cMo.inverse();
  vpColVector oC = oMc.getRotationMatrix() * vpColVector({ 0.0, 0.0, -1.0 });
  for (const vpRBSilhouettePoint &sp: frame.silhouettePoints) {
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

    if (m_useMask && frame.hasMask()) {
      float confidence = frame.mask[sp.i][sp.j];
      if (confidence < m_minMaskConfidence) {
        continue;
      }
    }

    vpRBSilhouetteControlPoint p;
    p.setCameraParameters(&frame.cam);
    p.setMovingEdge(&m_me);
    p.buildPoint((int)sp.i, (int)sp.j, sp.Z, sp.orientation, sp.normal, cMo, oMc);
    if (previousFrame.I.getSize() == frame.I.getSize()) {
      p.initControlPoint(previousFrame.I, 0);
    }
    else {
      p.initControlPoint(frame.I, 0);
    }

    p.setNumCandidates(m_numCandidates);
    m_controlPoints.push_back(p);
  }
  m_numFeatures = m_controlPoints.size();

  m_robust.setMinMedianAbsoluteDeviation(std::max(m_singlePointConvergedThresholdPixels * 0.5, 1.0) / frame.cam.get_px());
}


void vpRBSilhouetteMeTracker::trackFeatures(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &/*previousFrame*/, const vpHomogeneousMatrix &/*cMo*/)
{
  if (m_numCandidates <= 1) {
    for (vpRBSilhouetteControlPoint &p: m_controlPoints) {
      p.track(frame.I);
    }
  }
  else {
    for (vpRBSilhouetteControlPoint &p: m_controlPoints) {
      p.trackMultipleHypotheses(frame.I);
    }
  }
}

void vpRBSilhouetteMeTracker::initVVS(const vpRBFeatureTrackerInput & /*frame*/, const vpRBFeatureTrackerInput & /*previousFrame*/, const vpHomogeneousMatrix & /*cMo*/)
{
  if (m_numFeatures == 0) {
    return;
  }

  for (unsigned int k = 0; k < m_controlPoints.size(); k++) {
    m_controlPoints[k].initInteractionMatrixError();
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
      p.computeInteractionMatrixError(cMo);
    }
    else {
      p.computeInteractionMatrixErrorMH(cMo);
    }


    m_error[k] = p.error; //On remplit la matrice d'erreur

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
      for (unsigned int j = 0; j < 6; j++) {
        m_L[k][j] = p.L[j];
      }
    }
  }

  if (countValidSites == 0) {
    m_vvsConverged = false;
  }
  else {
    const double percentageConverged = (double)count / (double)countValidSites;
    if (percentageConverged < m_globalVVSConvergenceThreshold) {
      m_vvsConverged = false;
    }
    else {
      m_vvsConverged = true;
    }

  }

  m_robust.MEstimator(vpRobust::TUKEY, m_error, m_weights);

  for (unsigned int i = 0; i < m_error.size(); i++) {
    const double wi = m_weights[i] * factor[i];
    const double eri = m_error[i];
    m_covWeightDiag[i] = wi * wi;
    m_weighted_error[i] = wi * eri;
  }

  for (unsigned int i = 0; i < m_error.size(); i++) {
    for (unsigned int j = 0; j < 6; j++) {
      m_L[i][j] = m_weights[i] * m_L[i][j];
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

void vpRBSilhouetteMeTracker::display(const vpCameraParameters &/*cam*/, const vpImage<unsigned char> &I, const vpImage<vpRGBa> &/*IRGB*/, const vpImage<unsigned char> &/*depth*/, const vpRBFeatureDisplayType type) const
{

  if (type == vpRBFeatureDisplayType::SIMPLE) {
    for (const vpRBSilhouetteControlPoint &p: m_controlPoints) {
      const vpMeSite &s = p.getSite();
      s.display(I);
      // vpImagePoint diff(p.nys * m_me.getRange(), p.nxs * m_me.getRange());
      // vpImagePoint ip2 = p.icpoint + diff;
      // vpDisplay::displayLine(I, p.icpoint, ip2, vpColor::lightBlue, 2);
      // vpDisplay::displayPoint(I, p.icpoint, vpColor::red, 2);

    }
    // vpColor cs[6] = {
    //   vpColor::red,
    //   vpColor::blue,
    //   vpColor::green,
    //   vpColor::purple,
    //   vpColor::cyan,
    //   vpColor::darkGreen

    // };
    // unsigned colorIndex = 0;
    // for (const vpTrackedSilhouetteLine &line: m_lines) {
    //   if (line.getPoints().size() > 10) {
    //     for (const vpRBSilhouetteControlPoint *p: line.getPoints()) {
    //       vpDisplay::displayCross(I, p->getSite().m_i, p->getSite().m_j, 3, cs[colorIndex]);
    //     }
    //     colorIndex = (colorIndex + 1) % 6;
    //   }
    //   //line.getLine().display(I, cam);
    // }
  }
  else if (type == vpRBFeatureDisplayType::IMPORTANCE) {
    const double maxWeight = m_weights.getMaxValue();
    unsigned idx = 0;
    const vpColor bestColor = vpColor::green;
    for (const vpRBSilhouetteControlPoint &p: m_controlPoints) {
      const vpMeSite &s = p.getSite();

      if (s.getState() == vpMeSite::NO_SUPPRESSION) {
        double weight = m_weights[idx] / maxWeight;
        vpColor c((unsigned char)((double)(bestColor.R) * weight), (unsigned char)((double)(bestColor.G) * weight), (unsigned char)((double)(bestColor.B) * weight));
        vpDisplay::displayCross(I, s.get_i(), s.get_j(), 3, c, 1);
      }
      else {
        s.display(I);
      }
      ++idx;
    }
  }
  else if (type == vpRBFeatureDisplayType::ERROR) {
    unsigned idx = 0;
    const vpColor bestColor = vpColor::green;
    double maxError = m_error.getMaxValue();

    for (const vpRBSilhouetteControlPoint &p: m_controlPoints) {
      const vpMeSite &s = p.getSite();

      if (s.getState() == vpMeSite::NO_SUPPRESSION) {
        double weight = m_error[idx] / maxError;
        vpColor c((unsigned char)((double)(bestColor.R) * weight), (unsigned char)((double)(bestColor.G) * weight), (unsigned char)((double)(bestColor.B) * weight));
        vpDisplay::displayCross(I, s.get_i(), s.get_j(), 3, c, 1);
      }
      else {
        s.display(I);
      }
      ++idx;
    }
  }
  else {
    throw vpException(vpException::notImplementedError, "Display not implemented for unknown type");
  }
}

END_VISP_NAMESPACE
