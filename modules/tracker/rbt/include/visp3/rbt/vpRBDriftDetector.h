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
  \file vpRBDriftDetector.h
  \brief Base class for drift/divergence detection algorithms for the render-based tracker
*/
#ifndef VP_RB_DRIFT_DETECTOR_H
#define VP_RB_DRIFT_DETECTOR_H

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include VISP_NLOHMANN_JSON(json_fwd.hpp)
#endif

BEGIN_VISP_NAMESPACE

class vpRBFeatureTrackerInput;
class vpHomogeneousMatrix;
class vpRGBa;
template <typename T> class vpImage;

/**
 * \brief Base interface for algorithms that should detect tracking drift for the render-based tracker.
 *
 * In the tracking loop, these algorithms should be used as follows:
 * - Perform a tracking step, estimating a new object pose
 * - Call vpRBDriftDetector::update to update the drift detection parameters.
 * - use vpRBDriftDetector::hasDiverged to detect the drift, or vpRBDriftDetector::getScore to use the estimated tracking reliability.
 *
 * \ingroup group_rbt_drift
*/
class VISP_EXPORT vpRBDriftDetector
{
public:
  vpRBDriftDetector() = default;

  virtual ~vpRBDriftDetector() = default;

  /**
   * \brief Update the algorithm after a new tracking step.
   *
   * \param previousFrame The previous frame data: contains the input images at t-1 (linked to cprevTo) and the renders at t-2. May be empty for the first iteration
   * \param frame The current frame data: contains the input images at time t (linked to the newly estimated cTo) and the renders at t-1 (linked to cprevTo)
   * \param cTo the newly estimated object pose in the camera frame
   * \param cprevTo the previously estimated object pose in the camera frame
   */
  virtual void update(const vpRBFeatureTrackerInput &previousFrame, const vpRBFeatureTrackerInput &frame, const vpHomogeneousMatrix &cTo, const vpHomogeneousMatrix &cprevTo) = 0;

  virtual double score(const vpRBFeatureTrackerInput &frame, const vpHomogeneousMatrix &cTo) = 0;

  /**
   * \brief Get the estimated tracking reliability.
   * A high score should mean that the tracking is reliable.
   * Different algorithms may use different value ranges.
   *
   * \return The estimated tracking accuracy
   */
  virtual double getScore() const = 0;

  /**
   * \brief Returns whether the tracking has diverged and should be reinitialized.
   * This function should be called after update.
   *
   */
  virtual bool hasDiverged() const = 0;

  /**
   * \brief Displays the information used for drift detection.
   *
   * \param I the image in which to display the information
   */
  virtual void display(const vpImage<vpRGBa> &I) = 0;

#if defined(VISP_HAVE_NLOHMANN_JSON)
  virtual void loadJsonConfiguration(const nlohmann::json &) = 0;
#endif

};

END_VISP_NAMESPACE

#endif
