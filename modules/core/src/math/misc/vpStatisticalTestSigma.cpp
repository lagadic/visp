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
* \file vpStatisticalTestSigma.cpp
*
* \brief Definition of the vpStatisticalTestSigma class that implements sigma
* mean drift test.
*/

#include <visp3/core/vpStatisticalTestSigma.h>

BEGIN_VISP_NAMESPACE
void vpStatisticalTestSigma::computeLimits()
{
  float delta = m_h * m_stdev;
  m_limitDown = m_mean - delta;
  m_limitUp = m_mean + delta;
}

vpStatisticalTestAbstract::vpMeanDriftType vpStatisticalTestSigma::detectDownwardMeanDrift()
{
  if (m_s <= m_limitDown) {
    return vpStatisticalTestAbstract::MEAN_DRIFT_DOWNWARD;
  }
  else {
    return vpStatisticalTestAbstract::MEAN_DRIFT_NONE;
  }
}

vpStatisticalTestAbstract::vpMeanDriftType vpStatisticalTestSigma::detectUpwardMeanDrift()
{
  if (m_s >= m_limitUp) {
    return vpStatisticalTestAbstract::MEAN_DRIFT_UPWARD;
  }
  else {
    return vpStatisticalTestAbstract::MEAN_DRIFT_NONE;
  }
}

bool vpStatisticalTestSigma::updateStatistics(const float &signal)
{
  bool areStatsAvailable = vpStatisticalTestAbstract::updateStatistics(signal);
  if (areStatsAvailable) {
    computeLimits();
  }
  return areStatsAvailable;
}

void vpStatisticalTestSigma::updateTestSignals(const float &signal)
{
  m_s = signal;
}

vpStatisticalTestSigma::vpStatisticalTestSigma(const float &h, const unsigned int &nbSamplesForStats)
  : vpStatisticalTestAbstract()
  , m_h(h)
{
  init(h, nbSamplesForStats);
}

vpStatisticalTestSigma::vpStatisticalTestSigma(const float &h, const float &mean, const float &stdev)
  : vpStatisticalTestAbstract()
  , m_h(h)
{
  init(h, mean, stdev);
}

void vpStatisticalTestSigma::init(const float &h, const unsigned int &nbSamplesForStats)
{
  vpStatisticalTestAbstract::init();
  m_h = h;
  setNbSamplesForStat(nbSamplesForStats);
  m_s = 0;
}

void vpStatisticalTestSigma::init(const float &h, const float &mean, const float &stdev)
{
  vpStatisticalTestAbstract::init();
  m_h = h;
  m_mean = mean;
  m_s = 0;
  m_stdev = stdev;
  computeLimits();
  m_areStatisticsComputed = true;
}
END_VISP_NAMESPACE
