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
 */

/**
*
* \file vpStatisticalTestEWMA.cpp
*
* \brief Definition of the vpStatisticalTestEWMA class that implements Exponentially Weighted Moving Average
* mean drift test.
*/

#include <visp3/core/vpStatisticalTestEWMA.h>

BEGIN_VISP_NAMESPACE
void vpStatisticalTestEWMA::computeDeltaAndLimits()
{
  float delta = 3.f * m_stdev * std::sqrt(m_alpha / (2.f - m_alpha));
  m_limitDown = m_mean - delta;
  m_limitUp = m_mean + delta;
}

vpStatisticalTestAbstract::vpMeanDriftType vpStatisticalTestEWMA::detectDownwardMeanDrift()
{
  if (m_wt <= m_limitDown) {
    return MEAN_DRIFT_DOWNWARD;
  }
  else {
    return MEAN_DRIFT_NONE;
  }
}

vpStatisticalTestAbstract::vpMeanDriftType vpStatisticalTestEWMA::detectUpwardMeanDrift()
{
  if (m_wt >= m_limitUp) {
    return MEAN_DRIFT_UPWARD;
  }
  else {
    return MEAN_DRIFT_NONE;
  }
}

bool vpStatisticalTestEWMA::updateStatistics(const float &signal)
{
  bool areStatsReady = vpStatisticalTestAbstract::updateStatistics(signal);
  if (areStatsReady) {
    // Computation of the limits
    computeDeltaAndLimits();

    // Initialize first value
    m_wt = m_mean;
  }
  return areStatsReady;
}

void vpStatisticalTestEWMA::updateTestSignals(const float &signal)
{
  // Update last value
  m_wtprev = m_wt;
// w(t) = alpha * s(t) + (1 - alpha) * w(t- 1);
  m_wt = m_wtprev +  m_alpha * (signal - m_wtprev);
}

vpStatisticalTestEWMA::vpStatisticalTestEWMA(const float &alpha)
  : vpStatisticalTestAbstract()
  , m_alpha(0.f)
  , m_wt(0.f)
  , m_wtprev(0.f)
{
  init(alpha);
}

void vpStatisticalTestEWMA::init(const float &alpha)
{
  vpStatisticalTestAbstract::init();
  m_alpha = alpha;
  unsigned int nbRequiredSamples = static_cast<unsigned int>(std::ceil(3.f / m_alpha));
  setNbSamplesForStat(nbRequiredSamples);
  m_wt = 0.f;
  m_wtprev = 0.f;
}

void vpStatisticalTestEWMA::init(const float &alpha, const float &mean, const float &stdev)
{
  vpStatisticalTestAbstract::init();
  m_alpha = alpha;
  m_mean = mean;
  unsigned int nbRequiredSamples = static_cast<unsigned int>(std::ceil(3.f / m_alpha));
  setNbSamplesForStat(nbRequiredSamples);
  m_stdev = stdev;
  m_wt = mean;
  m_wtprev = 0.f;

  // Computation of the limits
  computeDeltaAndLimits();
  m_areStatisticsComputed = true;
}

void vpStatisticalTestEWMA::setAlpha(const float &alpha)
{
  init(alpha);
}
END_VISP_NAMESPACE
