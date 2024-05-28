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
* \file vpStatisticalTestMeanAdjustedCUSUM.cpp
*
* \brief Definition of the vpStatisticalTestMeanAdjustedCUSUM class that implements mean adjusted Cumulative Sum
* mean drift test.
*/

#include <visp3/core/vpStatisticalTestMeanAdjustedCUSUM.h>

BEGIN_VISP_NAMESPACE
void vpStatisticalTestMeanAdjustedCUSUM::computeDeltaAndLimits()
{
  setDelta(m_k * m_stdev);
  float limitDown = m_h * m_stdev;
  float limitUp = limitDown;
  setLimits(limitDown, limitUp);
}

vpStatisticalTestAbstract::vpMeanDriftType vpStatisticalTestMeanAdjustedCUSUM::detectDownwardMeanDrift()
{
  if (m_sminus >= m_limitDown) {
    return MEAN_DRIFT_DOWNWARD;
  }
  else {
    return MEAN_DRIFT_NONE;
  }
}

vpStatisticalTestAbstract::vpMeanDriftType vpStatisticalTestMeanAdjustedCUSUM::detectUpwardMeanDrift()
{
  if (m_splus >= m_limitUp) {
    return MEAN_DRIFT_UPWARD;
  }
  else {
    return MEAN_DRIFT_NONE;
  }
}

bool vpStatisticalTestMeanAdjustedCUSUM::updateStatistics(const float &signal)
{
  bool areStatsAvailable = vpStatisticalTestAbstract::updateStatistics(signal);
  if (areStatsAvailable) {
    // Computation of the limits
    if ((m_limitDown < 0.f) && (m_limitUp < 0.f)) {
      computeDeltaAndLimits();
    }

    // Initialize first values
    m_sminus = 0.f;
    m_splus = 0.f;
  }
  return areStatsAvailable;
}

void vpStatisticalTestMeanAdjustedCUSUM::updateTestSignals(const float &signal)
{
  m_sminus = std::max(0.f, m_sminus - (signal - m_mean) - m_half_delta);
  m_splus = std::max(0.f, m_splus + (signal - m_mean) - m_half_delta);
}

vpStatisticalTestMeanAdjustedCUSUM::vpStatisticalTestMeanAdjustedCUSUM(const float &h, const float &k, const unsigned int &nbPtsForStats)
  : vpStatisticalTestAbstract()
  , m_delta(-1.f)
  , m_h(h)
  , m_half_delta(-1.f)
  , m_k(k)
  , m_sminus(0.f)
  , m_splus(0.f)
{
  init(h, k, nbPtsForStats);
}

void vpStatisticalTestMeanAdjustedCUSUM::init(const float &h, const float &k, const unsigned int &nbPtsForStats)
{
  vpStatisticalTestAbstract::init();
  setNbSamplesForStat(nbPtsForStats);
  m_delta = -1.f;
  m_half_delta = -1.f;
  setLimits(-1.f, -1.f); // To compute automatically the limits once the signal statistics are available.
  m_h = h;
  m_k = k;
  m_mean = 0.f;
  m_sminus = 0.f;
  m_splus = 0.f;
  m_stdev = 0.f;
}

void vpStatisticalTestMeanAdjustedCUSUM::init(const float &delta, const float &limitDown, const float &limitUp, const unsigned int &nbPtsForStats)
{
  vpStatisticalTestAbstract::init();
  setDelta(delta);
  setLimits(limitDown, limitUp);
  setNbSamplesForStat(nbPtsForStats);
  m_sminus = 0.f;
  m_splus = 0.f;
}

void vpStatisticalTestMeanAdjustedCUSUM::init(const float &h, const float &k, const float &mean, const float &stdev)
{
  vpStatisticalTestAbstract::init();
  setLimits(-1.f, -1.f); // To compute automatically the limits once the signal statistics are available.
  m_h = h;
  m_k = k;
  m_mean = mean;
  m_sminus = 0.f;
  m_splus = 0.f;
  m_stdev = stdev;
  // Compute delta and limits from m_h, m_k and m_stdev
  computeDeltaAndLimits();
  m_areStatisticsComputed = true;
}

void vpStatisticalTestMeanAdjustedCUSUM::init(const float &delta, const float &limitDown, const float &limitUp, const float &mean, const float &stdev)
{
  vpStatisticalTestAbstract::init();
  setDelta(delta);
  setLimits(limitDown, limitUp);
  m_mean = mean;
  m_sminus = 0.f;
  m_splus = 0.f;
  m_stdev = stdev;
  m_sumForMean = 0.f;
  m_areStatisticsComputed = true;
}
END_VISP_NAMESPACE
