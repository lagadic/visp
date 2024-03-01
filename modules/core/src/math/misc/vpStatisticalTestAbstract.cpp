/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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

/**
*
* \file vpStatisticalTestAbstract.cpp
*
* \brief Definition of the vpStatisticalTestAbstract base class.
*/

#include <visp3/core/vpStatisticalTestAbstract.h>

std::string vpStatisticalTestAbstract::vpMeanDriftTypeToString(const vpStatisticalTestAbstract::vpMeanDriftType &type)
{
  std::string name;
  switch (type) {
  case NO_MEAN_DRIFT:
    name = " No jump";
    break;
  case MEAN_DRIFT_DOWNWARD:
    name = " Jump downward";
    break;
  case MEAN_DRIFT_UPWARD:
    name = " Jump upward";
    break;
  default:
    name = " Undefined jump";
    break;
  }
  return name;
}

void vpStatisticalTestAbstract::print(const vpStatisticalTestAbstract::vpMeanDriftType &type)
{
  std::cout << vpMeanDriftTypeToString(type) << " detected" << std::endl;
}

bool vpStatisticalTestAbstract::updateStatistics(const float &signal)
{
  m_s[static_cast<unsigned int>(m_count)] = signal;
  m_count += 1.f;
  m_sumForMean += signal;
  if (static_cast<unsigned int> (m_count) >= m_nbSamplesForStatistics) {
    // Computation of the mean
    m_mean = m_sumForMean / m_count;

    // Computation of the stdev
    float sumSquaredDiff = 0.f;
    unsigned int count = static_cast<unsigned int>(m_nbSamplesForStatistics);
    for (unsigned int i = 0; i < count; ++i) {
      sumSquaredDiff += (m_s[i] - m_mean) * (m_s[i] - m_mean);
    }
    float stdev = std::sqrt(sumSquaredDiff / m_count);
    if (m_stdevmin < 0) {
      m_stdev = stdev;
    }
    else {
      m_stdev = std::max<float>(m_stdev, m_stdevmin);
    }

    m_areStatisticsComputed = true;
  }
  return m_areStatisticsComputed;
}

vpStatisticalTestAbstract::vpStatisticalTestAbstract()
  : m_areStatisticsComputed(false)
  , m_count(0.f)
  , m_limitDown(0.f)
  , m_limitUp(0.f)
  , m_mean(0.f)
  , m_nbSamplesForStatistics(0)
  , m_s(nullptr)
  , m_stdev(0.f)
  , m_stdevmin(-1.f)
  , m_sumForMean(0.f)
{ }

vpStatisticalTestAbstract::vpStatisticalTestAbstract(const vpStatisticalTestAbstract &other)
{
  *this = other;
}

vpStatisticalTestAbstract::~vpStatisticalTestAbstract()
{
  if (m_s != nullptr) {
    delete m_s;
    m_s = nullptr;
  }
}

void vpStatisticalTestAbstract::init()
{
  m_areStatisticsComputed = false;
  m_count = 0.f;
  m_limitDown = 0.f;
  m_limitUp = 0.f;
  m_mean = 0.f;
  m_nbSamplesForStatistics = 0;
  if (m_s != nullptr) {
    delete m_s;
    m_s = nullptr;
  }
  m_stdev = 0.f;
  m_sumForMean = 0.f;
}

vpStatisticalTestAbstract &vpStatisticalTestAbstract::operator=(const vpStatisticalTestAbstract &other)
{
  m_areStatisticsComputed = other.m_areStatisticsComputed;
  m_count = other.m_count;
  m_limitDown = other.m_limitDown;
  m_limitUp = other.m_limitUp;
  m_mean = other.m_mean;
  if (other.m_nbSamplesForStatistics > 0) {
    setNbSamplesForStat(other.m_nbSamplesForStatistics);
  }
  else if (m_s != nullptr) {
    m_nbSamplesForStatistics = 0;
    delete m_s;
    m_s = nullptr;
  }
  m_stdev = 0.f;
  m_sumForMean = 0.f;
  return *this;
}

void vpStatisticalTestAbstract::setNbSamplesForStat(const unsigned int &nbSamples)
{
  m_nbSamplesForStatistics = nbSamples;
  if (m_s != nullptr) {
    delete m_s;
  }
  m_s = new float[nbSamples];
}

vpStatisticalTestAbstract::vpMeanDriftType vpStatisticalTestAbstract::testDownUpwardMeanShift(const float &signal)
{
  if (m_areStatisticsComputed) {
    updateTestSignals(signal);
    vpMeanDriftType jumpDown = detectDownwardMeanShift();
    vpMeanDriftType jumpUp = detectUpwardMeanShift();
    if (jumpDown != NO_MEAN_DRIFT) {
      return jumpDown;
    }
    else if (jumpUp != NO_MEAN_DRIFT) {
      return jumpUp;
    }
    else {
      return NO_MEAN_DRIFT;
    }
  }
  else {
    updateStatistics(signal);
    return NO_MEAN_DRIFT;
  }
}

vpStatisticalTestAbstract::vpMeanDriftType vpStatisticalTestAbstract::testDownwardMeanShift(const float &signal)
{
  if (m_areStatisticsComputed) {
    updateTestSignals(signal);
    return detectDownwardMeanShift();
  }
  else {
    updateStatistics(signal);
    return NO_MEAN_DRIFT;
  }
}

vpStatisticalTestAbstract::vpMeanDriftType vpStatisticalTestAbstract::testUpwardMeanShift(const float &signal)
{
  if (m_areStatisticsComputed) {
    updateTestSignals(signal);
    return detectUpwardMeanShift();
  }
  else {
    updateStatistics(signal);
    return NO_MEAN_DRIFT;
  }
}
