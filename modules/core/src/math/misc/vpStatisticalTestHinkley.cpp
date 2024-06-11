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
* \file vpStatisticalTestHinkley.cpp
*
* \brief Definition of the vpStatisticalTestHinkley class corresponding to the Hinkley's
* cumulative sum test.
*/

#include <visp3/core/vpStatisticalTestHinkley.h>
#include <visp3/core/vpMath.h>

#include <cmath> // std::fabs
#include <iostream>
#include <limits> // numeric_limits
#include <stdio.h>
#include <stdlib.h>

BEGIN_VISP_NAMESPACE
vpStatisticalTestHinkley::vpStatisticalTestHinkley()
  : vpStatisticalTestAbstract()
  , m_dmin2(0.1f)
  , m_alpha(0.2f)
  , m_Sk(0.f)
  , m_Mk(0.f)
  , m_Tk(0.f)
  , m_Nk(0.f)
  , m_computeDeltaAndAlpha(false)
  , m_h(4.76f)
  , m_k(1.f)
{
  init();
}

vpStatisticalTestHinkley::vpStatisticalTestHinkley(const float &alpha, const float &delta_val, const unsigned int &nbSamplesForInit)
  : vpStatisticalTestAbstract()
  , m_dmin2(delta_val / 2.f)
  , m_alpha(alpha)
  , m_Sk(0.f)
  , m_Mk(0.f)
  , m_Tk(0.f)
  , m_Nk(0.f)
  , m_computeDeltaAndAlpha(false)
  , m_h(4.76f)
  , m_k(1.f)
{
  init(alpha, delta_val, nbSamplesForInit);
}

vpStatisticalTestHinkley::vpStatisticalTestHinkley(const float &h, const float &k, const bool &computeAlphaDeltaFromStdev, const unsigned int &nbSamplesForInit)
  : vpStatisticalTestAbstract()
{
  init(h, k, computeAlphaDeltaFromStdev, nbSamplesForInit);
}

vpStatisticalTestHinkley::vpStatisticalTestHinkley(const float &h, const float &k, const float &mean, const float &stdev)
  : vpStatisticalTestAbstract()
{
  init(h, k, mean, stdev);
}

void vpStatisticalTestHinkley::init()
{
  vpStatisticalTestAbstract::init();
  setNbSamplesForStat(30);
  setAlpha(m_alpha);

  m_Sk = 0.f;
  m_Mk = 0.f;

  m_Tk = 0.f;
  m_Nk = 0.f;

  m_computeDeltaAndAlpha = false;
}

void vpStatisticalTestHinkley::init(const float &alpha, const float &delta_val, const unsigned int &nbSamplesForInit)
{
  init();
  setNbSamplesForStat(nbSamplesForInit);
  setAlpha(alpha);
  setDelta(delta_val);
  m_computeDeltaAndAlpha = false;
}

void vpStatisticalTestHinkley::init(const float &alpha, const float &delta_val, const float &mean)
{
  init();
  setAlpha(alpha);
  setDelta(delta_val);
  m_mean = mean;
  m_computeDeltaAndAlpha = false;
  m_areStatisticsComputed = true;
}

void vpStatisticalTestHinkley::init(const float &h, const float &k, const bool &computeAlphaDeltaFromStdev, const unsigned int &nbSamples)
{
  if (!computeAlphaDeltaFromStdev) {
    throw(vpException(vpException::badValue, "computeAlphaDeltaFromStdev must be true, or use another init function"));
  }
  init();
  setNbSamplesForStat(nbSamples);
  m_h = h;
  m_k = k;
  m_computeDeltaAndAlpha = true;
}

void vpStatisticalTestHinkley::init(const float &h, const float &k, const float &mean, const float &stdev)
{
  init();
  m_mean = mean;
  m_stdev = stdev;
  m_h = h;
  m_k = k;
  m_computeDeltaAndAlpha = true;
  computeAlphaDelta();
  m_areStatisticsComputed = true;
}

vpStatisticalTestHinkley::~vpStatisticalTestHinkley() { }

void vpStatisticalTestHinkley::setDelta(const float &delta) { m_dmin2 = delta / 2.f; }

void vpStatisticalTestHinkley::setAlpha(const float &alpha)
{
  this->m_alpha = alpha;
  m_limitDown = m_alpha;
  m_limitUp = m_alpha;
}

void vpStatisticalTestHinkley::computeAlphaDelta()
{
  float delta = m_k * m_stdev;
  setDelta(delta);
  float alpha = m_h * m_stdev;
  setAlpha(alpha);
}

void vpStatisticalTestHinkley::computeMean(double signal)
{
  // When the mean slowly increases or decreases, especially after an abrupt change of the mean,
  // the means tends to drift. To reduce the drift of the mean
  // it is updated with the current value of the signal only if
  // a beginning of a mean drift is not detected,
  // i.e.  if ( ((m_Mk-m_Sk) == 0) && ((m_Tk-m_Nk) == 0) )
  if ((std::fabs(m_Mk - m_Sk) <= std::fabs(vpMath::maximum(m_Mk, m_Sk)) * std::numeric_limits<double>::epsilon()) &&
      (std::fabs(m_Tk - m_Nk) <= std::fabs(vpMath::maximum(m_Tk, m_Nk)) * std::numeric_limits<double>::epsilon())) {
    m_mean = (m_mean * (m_count - 1) + static_cast<float>(signal)) / (m_count);
  }
}

void vpStatisticalTestHinkley::computeSk(double signal)
{
  m_Sk += static_cast<float>(signal) - m_mean + m_dmin2;
}

void vpStatisticalTestHinkley::computeMk()
{
  if (m_Sk > m_Mk) {
    m_Mk = m_Sk;
  }
}

void vpStatisticalTestHinkley::computeTk(double signal)
{
  m_Tk += static_cast<float>(signal) - m_mean - m_dmin2;
}

void vpStatisticalTestHinkley::computeNk()
{
  if (m_Tk < m_Nk) {
    m_Nk = m_Tk;
  }
}

vpStatisticalTestAbstract::vpMeanDriftType vpStatisticalTestHinkley::detectDownwardMeanDrift()
{
  vpStatisticalTestAbstract::vpMeanDriftType shift = MEAN_DRIFT_NONE;
  if ((m_Mk - m_Sk) > m_alpha) {
    shift = MEAN_DRIFT_DOWNWARD;
  }
  return shift;
}

vpStatisticalTestAbstract::vpMeanDriftType vpStatisticalTestHinkley::detectUpwardMeanDrift()
{
  vpStatisticalTestAbstract::vpMeanDriftType shift = MEAN_DRIFT_NONE;
  if ((m_Tk - m_Nk) > m_alpha) {
    shift = MEAN_DRIFT_UPWARD;
  }
  return shift;
}

bool vpStatisticalTestHinkley::updateStatistics(const float &signal)
{
  bool updateStats = vpStatisticalTestAbstract::updateStatistics(signal);
  if (m_areStatisticsComputed) {
    // If needed, compute alpha and delta
    if (m_computeDeltaAndAlpha) {
      computeAlphaDelta();
    }
    // Initialize the test signals
    m_Mk = 0.f;
    m_Nk = 0.f;
    m_Sk = 0.f;
    m_Tk = 0.f;
  }
  return updateStats;
}

void vpStatisticalTestHinkley::updateTestSignals(const float &signal)
{
  computeSk(signal);
  computeTk(signal);

  computeMk();
  computeNk();

  ++m_count;
  computeMean(signal);
}
END_VISP_NAMESPACE
