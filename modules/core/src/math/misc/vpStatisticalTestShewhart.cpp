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
* \file vpStatisticalTestShewhart.cpp
*
* \brief Definition of the vpStatisticalTestShewhart class that implements Shewhart's
* mean drift test.
*/

#include <visp3/core/vpStatisticalTestShewhart.h>

#include<cstring>

#include <visp3/core/vpMath.h>

BEGIN_VISP_NAMESPACE
const int vpStatisticalTestShewhart::NB_DATA_SIGNAL;
const bool vpStatisticalTestShewhart::CONST_ALL_WECO_ACTIVATED[vpStatisticalTestShewhart::COUNT_WECO - 1] = { true, true, true, true };

std::string vpStatisticalTestShewhart::vpWecoRulesAlarmToString(const vpStatisticalTestShewhart::vpWecoRulesAlarm &alarm)
{
  std::string name;
  switch (alarm) {
  case THREE_SIGMA_WECO:
    name = "3-sigma alarm";
    break;
  case TWO_SIGMA_WECO:
    name = "2-sigma alarm";
    break;
  case ONE_SIGMA_WECO:
    name = "1-sigma alarm";
    break;
  case SAME_SIDE_WECO:
    name = "Same-side alarm";
    break;
  case NONE_WECO:
    name = "No alarm";
    break;
  default:
    name = "Unknown WECO alarm";
  }
  return name;
}

void vpStatisticalTestShewhart::computeLimits()
{
  float delta = 3.f * m_stdev;
  m_limitDown = m_mean - delta;
  m_limitUp = m_mean + delta;
  m_oneSigmaNegLim = m_mean - m_stdev;
  m_oneSigmaPosLim = m_mean + m_stdev;
  m_twoSigmaNegLim = m_mean - 2.f * m_stdev;
  m_twoSigmaPosLim = m_mean + 2.f * m_stdev;
}

vpStatisticalTestAbstract::vpMeanDriftType vpStatisticalTestShewhart::detectDownwardMeanDrift()
{
  if (m_nbDataInBuffer < NB_DATA_SIGNAL) {
    return vpStatisticalTestAbstract::MEAN_DRIFT_NONE;
  }
  if ((m_signal[m_idCurrentData] <= m_limitDown) && m_activatedWECOrules[THREE_SIGMA_WECO]) {
    m_alarm = THREE_SIGMA_WECO;
    return vpStatisticalTestAbstract::MEAN_DRIFT_DOWNWARD;
  }
  if (!m_activateWECOrules) {
    return vpStatisticalTestAbstract::MEAN_DRIFT_NONE;
  }
  vpStatisticalTestAbstract::vpMeanDriftType result = vpStatisticalTestAbstract::MEAN_DRIFT_NONE;
  int id = vpMath::modulo(m_idCurrentData - (NB_DATA_SIGNAL - 1), NB_DATA_SIGNAL);
  int i = 0;
  unsigned int nbAboveMean = 0;
  unsigned int nbAbove2SigmaLimit = 0;
  unsigned int nbAbove1SigmaLimit = 0;
  while (i < NB_DATA_SIGNAL) {
    // Reinit for next iteration
    nbAbove2SigmaLimit = 0;
    nbAbove1SigmaLimit = 0;
    if (m_signal[id] < m_mean  && m_activatedWECOrules[SAME_SIDE_WECO]) {
      // Single-side test
      ++nbAboveMean;
    }
    if (i > 3  && m_activatedWECOrules[TWO_SIGMA_WECO]) {
      // Two sigma test
      for (int idPrev = vpMath::modulo(id - 2, NB_DATA_SIGNAL); idPrev != id; idPrev = vpMath::modulo(idPrev + 1, NB_DATA_SIGNAL)) {
        if (m_signal[idPrev] <= m_twoSigmaNegLim) {
          ++nbAbove2SigmaLimit;
        }
      }
      if (m_signal[id] <= m_twoSigmaNegLim) {
        ++nbAbove2SigmaLimit;
      }
      if (nbAbove2SigmaLimit >= 2) {
        break;
      }
    }
    if (i > 5 && m_activatedWECOrules[ONE_SIGMA_WECO]) {
      // One sigma test
      for (int idPrev = vpMath::modulo(id - 4, NB_DATA_SIGNAL); idPrev != id; idPrev = vpMath::modulo(idPrev + 1, NB_DATA_SIGNAL)) {
        if (m_signal[idPrev] <= m_oneSigmaNegLim) {
          ++nbAbove1SigmaLimit;
        }
      }
      if (m_signal[id] <= m_oneSigmaNegLim) {
        ++nbAbove1SigmaLimit;
      }
      if (nbAbove1SigmaLimit >= 4) {
        break;
      }
    }
    id = vpMath::modulo(id  + 1, NB_DATA_SIGNAL);
    ++i;
  }
  if (nbAboveMean == NB_DATA_SIGNAL) {
    m_alarm = SAME_SIDE_WECO;
    result = MEAN_DRIFT_DOWNWARD;
  }
  else if (nbAbove2SigmaLimit >= 2) {
    m_alarm = TWO_SIGMA_WECO;
    result = MEAN_DRIFT_DOWNWARD;
  }
  else if (nbAbove1SigmaLimit >= 4) {
    m_alarm = ONE_SIGMA_WECO;
    result = MEAN_DRIFT_DOWNWARD;
  }
  return result;
}

vpStatisticalTestAbstract::vpMeanDriftType vpStatisticalTestShewhart::detectUpwardMeanDrift()
{
  if (m_nbDataInBuffer < NB_DATA_SIGNAL) {
    return vpStatisticalTestAbstract::MEAN_DRIFT_NONE;
  }
  if ((m_signal[m_idCurrentData] >= m_limitUp) && m_activatedWECOrules[THREE_SIGMA_WECO]) {
    m_alarm = THREE_SIGMA_WECO;
    return vpStatisticalTestAbstract::MEAN_DRIFT_UPWARD;
  }
  if (!m_activateWECOrules) {
    return vpStatisticalTestAbstract::MEAN_DRIFT_NONE;
  }
  vpStatisticalTestAbstract::vpMeanDriftType result = vpStatisticalTestAbstract::MEAN_DRIFT_NONE;
  int id = vpMath::modulo(m_idCurrentData - (NB_DATA_SIGNAL - 1), NB_DATA_SIGNAL);
  int i = 0;
  unsigned int nbAboveMean = 0;
  unsigned int nbAbove2SigmaLimit = 0;
  unsigned int nbAbove1SigmaLimit = 0;
  while (i < NB_DATA_SIGNAL) {
    // Reinit for next iteration
    nbAbove2SigmaLimit = 0;
    nbAbove1SigmaLimit = 0;
    if (m_signal[id] > m_mean && m_activatedWECOrules[SAME_SIDE_WECO]) {
      // Single-side test
      ++nbAboveMean;
    }
    if (i > 3 && m_activatedWECOrules[TWO_SIGMA_WECO]) {
      // Two sigma test
      for (int idPrev = vpMath::modulo(id - 2, NB_DATA_SIGNAL); idPrev != id; idPrev = vpMath::modulo(idPrev + 1, NB_DATA_SIGNAL)) {
        if (m_signal[idPrev] >= m_twoSigmaPosLim) {
          ++nbAbove2SigmaLimit;
        }
      }
      if (m_signal[id] >= m_twoSigmaPosLim) {
        ++nbAbove2SigmaLimit;
      }
      if (nbAbove2SigmaLimit >= 2) {
        break;
      }
    }
    if (i > 5 && m_activatedWECOrules[ONE_SIGMA_WECO]) {
      // One sigma test
      for (int idPrev = vpMath::modulo(id - 4, NB_DATA_SIGNAL); idPrev != id; idPrev = vpMath::modulo(idPrev + 1, NB_DATA_SIGNAL)) {
        if (m_signal[idPrev] >= m_oneSigmaPosLim) {
          ++nbAbove1SigmaLimit;
        }
      }
      if (m_signal[id] >= m_oneSigmaPosLim) {
        ++nbAbove1SigmaLimit;
      }
      if (nbAbove1SigmaLimit >= 4) {
        break;
      }
    }
    id = vpMath::modulo(id  + 1, NB_DATA_SIGNAL);
    ++i;
  }
  if (nbAboveMean == NB_DATA_SIGNAL) {
    m_alarm = SAME_SIDE_WECO;
    result = MEAN_DRIFT_UPWARD;
  }
  else if (nbAbove2SigmaLimit >= 2) {
    m_alarm = TWO_SIGMA_WECO;
    result = MEAN_DRIFT_UPWARD;
  }
  else if (nbAbove1SigmaLimit >= 4) {
    m_alarm = ONE_SIGMA_WECO;
    result = MEAN_DRIFT_UPWARD;
  }
  return result;
}

bool vpStatisticalTestShewhart::updateStatistics(const float &signal)
{
  bool areStatsAvailable = vpStatisticalTestAbstract::updateStatistics(signal);
  updateTestSignals(signal); // Store the signal in the circular buffer too.
  if (areStatsAvailable) {
    computeLimits();
  }
  return areStatsAvailable;
}

void vpStatisticalTestShewhart::updateTestSignals(const float &signal)
{
  m_idCurrentData = (m_idCurrentData + 1) % NB_DATA_SIGNAL;
  m_signal[m_idCurrentData] = signal;
  if (m_nbDataInBuffer < NB_DATA_SIGNAL) {
    ++m_nbDataInBuffer;
  }
}

vpStatisticalTestShewhart::vpStatisticalTestShewhart(const bool &activateWECOrules, const bool activatedRules[COUNT_WECO - 1], const unsigned int &nbSamplesForStats)
  : vpStatisticalTestSigma(3, nbSamplesForStats)
  , m_nbDataInBuffer(0)
  , m_activateWECOrules(activateWECOrules)
  , m_idCurrentData(0)
  , m_alarm(NONE_WECO)
  , m_oneSigmaNegLim(0.f)
  , m_oneSigmaPosLim(0.f)
  , m_twoSigmaNegLim(0.f)
  , m_twoSigmaPosLim(0.f)
{
  init(activateWECOrules, activatedRules, nbSamplesForStats);
}

vpStatisticalTestShewhart::vpStatisticalTestShewhart(const bool &activateWECOrules, const bool activatedRules[COUNT_WECO - 1], const float &mean, const float &stdev)
  : vpStatisticalTestSigma(3)
  , m_nbDataInBuffer(0)
  , m_activateWECOrules(activateWECOrules)
  , m_idCurrentData(0)
  , m_alarm(NONE_WECO)
  , m_oneSigmaNegLim(0.f)
  , m_oneSigmaPosLim(0.f)
  , m_twoSigmaNegLim(0.f)
  , m_twoSigmaPosLim(0.f)
{
  init(activateWECOrules, activatedRules, mean, stdev);
}

std::vector<float> vpStatisticalTestShewhart::getSignals() const
{
  std::vector<float> signals;
  for (int i = 0; i < NB_DATA_SIGNAL; ++i) {
    int id = vpMath::modulo(m_idCurrentData - (NB_DATA_SIGNAL - i - 1), NB_DATA_SIGNAL);
    signals.push_back(m_signal[id]);
  }
  return signals;
}

void vpStatisticalTestShewhart::init(const bool &activateWECOrules, const bool activatedRules[COUNT_WECO - 1], const unsigned int &nbSamplesForStats)
{
  vpStatisticalTestSigma::init(3.f, nbSamplesForStats);
  m_nbDataInBuffer = 0;
  memset(m_signal, 0, NB_DATA_SIGNAL * sizeof(float));
  m_activateWECOrules = activateWECOrules;
  std::memcpy(m_activatedWECOrules, activatedRules, (COUNT_WECO - 1) * sizeof(bool));
  m_idCurrentData = 0;
  m_alarm = NONE_WECO;
  m_oneSigmaNegLim = 0.f;
  m_oneSigmaPosLim = 0.f;
  m_twoSigmaNegLim = 0.f;
  m_twoSigmaPosLim = 0.f;
}

void vpStatisticalTestShewhart::init(const bool &activateWECOrules, const bool activatedRules[COUNT_WECO - 1], const float &mean, const float &stdev)
{
  vpStatisticalTestShewhart::init(activateWECOrules, activatedRules, 30);
  m_mean = mean;
  m_stdev = stdev;
  computeLimits();
  m_areStatisticsComputed = true;
}
END_VISP_NAMESPACE
