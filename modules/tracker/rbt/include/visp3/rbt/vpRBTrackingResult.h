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
  \file vpRBKltTracker.h
  \brief KLT features in the context of render based tracking
*/
#ifndef VP_RB_TRACKING_RESULT_H
#define VP_RB_TRACKING_RESULT_H

#include <visp3/core/vpConfig.h>


#include <vector>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>


#include <visp3/rbt/vpRBTrackingTimings.h>
#include <visp3/rbt/vpRBFeatureTracker.h>

BEGIN_VISP_NAMESPACE

class VISP_EXPORT vpRBFeatureResult
{

public:
  vpRBFeatureResult() = default;
  void onIter(vpRBFeatureTracker &tracker)
  {
    m_numFeatures.push_back(tracker.getNumFeatures());
    m_overallWeight.push_back(tracker.getVVSTrackerWeight());
    m_error.push_back(tracker.getWeightedError());
    m_JTJ.push_back(tracker.getLTL());
    m_JTR.push_back(tracker.getLTR());
  }

#ifdef VISP_HAVE_NLOHMANN_JSON

#endif




private:
  std::vector<unsigned int> m_numFeatures;
  std::vector<double> m_overallWeight;

  std::vector<vpColVector> m_error;
  std::vector<vpMatrix> m_JTJ;
  std::vector<vpColVector> m_JTR;
};


enum vpRBTrackingStoppingReason
{
  MAX_ITERS = 0,
  CONVERGENCE_CRITERION = 1,
  OBJECT_NOT_IN_IMAGE = 2,
  EXCEPTION = 3,
  INVALID_REASON = 4
};


class VISP_EXPORT vpRBTrackingResult
{
public:

  vpRBTrackingResult()
  {
    m_odometryMotion = vpHomogeneousMatrix();
  }

  unsigned int numIterations() const { return m_cMos.size(); }
  const std::vector<vpHomogeneousMatrix> &getPoses() const { return m_cMos; }


  vpRBTrackingTimings &timings() { return m_timings; }

  void setOdometryMotion(const vpHomogeneousMatrix &cMw)
  {
    m_odometryMotion = cMw;
  }

private:
  vpRBTrackingStoppingReason m_stopReason;
  vpRBTrackingTimings m_timings;
  std::vector<vpHomogeneousMatrix> m_cMos;
  vpHomogeneousMatrix m_odometryMotion;
  std::vector<vpRBFeatureResult> m_featureData;
};

END_VISP_NAMESPACE

#endif
