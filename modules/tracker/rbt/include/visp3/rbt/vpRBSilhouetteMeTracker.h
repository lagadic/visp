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

/*!
  \file vpRBSilhouetteMeTracker.h
  \brief Moving edge tracking for depth-extracted object contours
*/
#ifndef VP_RB_SILHOUETTE_ME_TRACKER_H
#define VP_RB_SILHOUETTE_ME_TRACKER_H

#include <visp3/rbt/vpRBFeatureTracker.h>
#include <visp3/rbt/vpRBSilhouetteControlPoint.h>
#include <visp3/core/vpRobust.h>

BEGIN_VISP_NAMESPACE
/**
 * \brief Moving edge feature tracking from depth-extracted object contours
 *
 * \ingroup group_rbt_trackers
*/
class VISP_EXPORT vpRBSilhouetteMeTracker : public vpRBFeatureTracker
{
public:

  vpRBSilhouetteMeTracker() :
    vpRBFeatureTracker(), m_me(), m_numCandidates(1), m_robustMadMin(1.0), m_globalVVSConvergenceThreshold(1.0),
    m_singlePointConvergedThresholdPixels(3), m_useMask(false), m_minMaskConfidence(0.f)
  { }

  virtual ~vpRBSilhouetteMeTracker() = default;

  bool requiresRGB() const VP_OVERRIDE { return false; }

  bool requiresDepth() const VP_OVERRIDE { return false; }

  bool requiresSilhouetteCandidates() const VP_OVERRIDE { return true; }

  void setMovingEdge(const vpMe &me) { m_me = me; }

  void onTrackingIterStart() VP_OVERRIDE
  {
    m_controlPoints.clear();
  }

  void onTrackingIterEnd() VP_OVERRIDE { }

  /**
   * @brief Extract the geometric features from the list of collected silhouette points
   */
  void extractFeatures(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &previousFrame, const vpHomogeneousMatrix &cMo) VP_OVERRIDE;

  void trackFeatures(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &previousFrame, const vpHomogeneousMatrix &cMo) VP_OVERRIDE;

  void initVVS(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &previousFrame, const vpHomogeneousMatrix &cMo) VP_OVERRIDE;

  void computeVVSIter(const vpRBFeatureTrackerInput &frame, const vpHomogeneousMatrix &cMo, unsigned int iteration) VP_OVERRIDE;

  void display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB, const vpImage<unsigned char> &depth, const vpRBFeatureDisplayType type) const VP_OVERRIDE;

  /**
   * \name Settings
   * @{
   */
  const vpMe &getMe() const { return m_me; }
  vpMe &getMe() { return m_me; }

  unsigned int getNumCandidates() const { return m_numCandidates; }
  void setNumCandidates(unsigned int candidates)
  {
    if (candidates == 0) {
      throw vpException(vpException::badValue, "Cannot set a number of candidates equal to zero");
    }
    m_numCandidates = candidates;
  }

  double getMinRobustThreshold() const { return m_robustMadMin; }
  void setMinRobustThreshold(double threshold)
  {
    if (threshold < 0) {
      throw vpException(vpException::badValue, "Robust M estimator min threshold should be greater or equal to 0.");
    }
    m_robustMadMin = threshold;
  }

  /**
   * \brief Returns whether the tracking algorithm should filter out points that are unlikely to be on the object according to the mask.
   * If the mask is not computed beforehand, then it has no effect
   */
  bool shouldUseMask() const { return m_useMask; }
  void setShouldUseMask(bool useMask) { m_useMask = useMask; }

  /**
   * \brief Returns the minimum mask confidence that a pixel linked to depth point should have if it should be kept during tracking.
   *
   * This value is between 0 and 1
   */
  float getMinimumMaskConfidence() const { return m_minMaskConfidence; }
  void setMinimumMaskConfidence(float confidence)
  {
    if (confidence > 1.f || confidence < 0.f) {
      throw vpException(vpException::badValue, "Mask confidence should be between 0 and 1");
    }
    m_minMaskConfidence = confidence;
  }

  double getSinglePointConvergenceThreshold() const { return m_singlePointConvergedThresholdPixels; }
  void setSinglePointConvergenceThreshold(double threshold)
  {
    if (threshold < 0.0) {
      throw vpException(vpException::badValue, "Convergence threshold should be null or positive");
    }
    m_singlePointConvergedThresholdPixels = threshold;
  }

  double getGlobalConvergenceMinimumRatio() const { return m_globalVVSConvergenceThreshold; }
  void setGlobalConvergenceMinimumRatio(double threshold)
  {
    if (threshold < 0.0 || threshold > 1.0) {
      throw vpException(vpException::badValue, "Minimum converged ratio be between 0 and 1");
    }
    m_globalVVSConvergenceThreshold = threshold;
  }

#if defined(VISP_HAVE_NLOHMANN_JSON)
  virtual void loadJsonConfiguration(const nlohmann::json &j) VP_OVERRIDE
  {
    vpRBFeatureTracker::loadJsonConfiguration(j);
    setNumCandidates(j.value("numCandidates", m_numCandidates));
    setSinglePointConvergenceThreshold(j.value("convergencePixelThreshold", m_singlePointConvergedThresholdPixels));
    setGlobalConvergenceMinimumRatio(j.value("convergenceRatio", m_globalVVSConvergenceThreshold));
    m_me = j.value("movingEdge", m_me);
    setShouldUseMask(j.value("useMask", m_useMask));
    setMinimumMaskConfidence(j.value("minMaskConfidence", m_minMaskConfidence));
    // m_me.setThresholdMarginRatio(-1.0);
    // m_me.setMinThreshold(-1.0);
  }
#endif

  /**
   * \name Settings
   * @}
   */

private:

  std::vector<vpRBSilhouetteControlPoint> m_controlPoints;
  vpMe m_me; //! Moving edge settings
  unsigned int m_numCandidates; //! Number of best candidates kept when finding correspondence points
  vpRobust m_robust; //! M-Estimator to filter outliers
  double m_robustMadMin;
  double m_globalVVSConvergenceThreshold; //! Percentage of control points that should have converged to consider VVS as successful
  double m_singlePointConvergedThresholdPixels; //! Whether a single Control point is considered as converged
  bool m_useMask;
  float m_minMaskConfidence;
};

END_VISP_NAMESPACE

#endif
