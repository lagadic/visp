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
/*!
 * \file vpStatisticalTestMeanAdjustedCUSUM.h
 * \brief Statistical Process Control mean adjusted CUSUM implementation.
 */

#ifndef _vpStatisticalTestMeanAdjustedCUSUM_h_
#define _vpStatisticalTestMeanAdjustedCUSUM_h_

#include <visp3/core/vpConfig.h>

#include <visp3/core/vpStatisticalTestAbstract.h>

/**
 * \ingroup group_core_math_tools
 * \brief Class that permits to perform a mean adjusted Cumulative Sum test.
 */
class VISP_EXPORT vpStatisticalTestMeanAdjustedCUSUM : public vpStatisticalTestAbstract
{
protected:
  float m_delta; /*!< Slack of the CUSUM test, i.e. amplitude of mean shift we want to be able to detect.*/
  float m_h; /*!< Alarm factor that permits to determine the limit telling when a mean shift occurs: limit = m_h * m_stdev .
                  To have an Average Run Lenght of ~374 samples for a detection of 1 stdev, set it to 4.76f*/
  float m_half_delta; /*!< Half of the amplitude we want to detect.*/
  float m_k; /*!< Detection factor that permits to determine the slack: m_delta = m_k * m_stdev .*/
  float m_sminus; /*!< Test signal for downward mean shift: S_-(i) = max{0, S_-(i-1) - (y_i - m_mean) - m_delta/2}.*/
  float m_splus; /*!< Test signal for upward mean shift: S_+(i) = max{0, S_+(i-1) + (y_i - m_mean) - m_delta/2}.*/

  /**
   * \brief Compute the upper and lower limits of the test signal.
   */
  virtual void computeDeltaAndLimits();

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  virtual vpMeanDriftType detectDownwardMeanShift() override;
#else
  virtual vpMeanDriftType detectDownwardMeanShift();
#endif

/**
 * \brief Detects if a upward jump occured on the mean.
 *
 * \return upwardJump if a upward jump occured, noJump otherwise.
 */
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  virtual vpMeanDriftType detectUpwardMeanShift() override;
#else
  virtual vpMeanDriftType detectUpwardMeanShift();
#endif

  /**
   * \brief Update m_s and if enough values are available, compute the mean, the standard
   * deviation and the limits.
   *
   * \param[in] signal The new value of the signal to monitor.
   */
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  virtual bool updateStatistics(const float &signal) override;
#else
  virtual bool updateStatistics(const float &signal);
#endif

  /**
   * \brief Update the test signals.
   *
   * \param[in] signal The new value of the signal to monitor.
   */
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  virtual void updateTestSignals(const float &signal) override;
#else
  virtual void updateTestSignals(const float &signal);
#endif
public:
  /**
   * \brief Construct a new vpStatisticalTestMeanAdjustedCUSUM object.
   *
   * \param[in] h The alarm factor that permits to determine when the process is out of control from the standard
   * deviation of the signal.
   * \param[in] k The detection factor that permits to determine the slack of the CUSUM test, i.e. the
   * minimum value of the jumps we want to detect, from the standard deviation of the signal.
   * \param[in] nbPtsForStats The number of samples to use to compute the mean and the standard deviation of the signal
   * to monitor.
   */
  vpStatisticalTestMeanAdjustedCUSUM(const float &h = 4.f, const float &k = 1.f, const unsigned int &nbPtsForStats = 30);

  /**
   * \brief Get the slack of the CUSUM test,
   * i.e. amplitude of mean shift we want to be able to detect.
   *
   * \return float The slack of the CUSUM test.
   */
  inline float getDelta() const
  {
    return m_delta;
  }

  /**
   * \brief Get the alarm factor.
   *
   * \return float The alarm factor.
   */
  inline float getH() const
  {
    return m_h;
  }

  /**
   * \brief Get the detection factor.
   *
   * \return float The detection factor.
   */
  inline float getK() const
  {
    return m_k;
  }

  /**
   * \brief Get the latest value of the test signal for downward jumps of the mean.
   *
   * \return float Its latest value.
   */
  inline float getTestSignalMinus() const
  {
    return m_sminus;
  }

  /**
   * \brief Get the latest value of the test signal for upward jumps of the mean.
   *
   * \return float Its latest value.
   */
  inline float getTestSignalPlus() const
  {
    return m_splus;
  }

  /**
   * \brief (Re)Initialize the mean adjusted CUSUM test.
   *
   * \param[in] h The alarm factor that permits to determine when the process is out of control from the standard
   * deviation of the signal.
   * \param[in] k The detection factor that permits to determine the slack of the CUSUM test, i.e. the
   * minimum value of the jumps we want to detect, from the standard deviation of the signal.
   * \param[in] nbPtsForStats The number of points to use to compute the mean and the standard deviation of the signal
   */
  void init(const float &h, const float &k, const unsigned int &nbPtsForStats);

  /**
   * \brief Initialize the mean adjusted CUSUM test.
   *
   * \param[in] delta The slack of the CUSUM test, i.e. the minimum value of the jumps we want to detect.
   * \param[in] limitDown The lower limit of the CUSUM test, for the downward jumps.
   * \param[in] limitUp The upper limit of the CUSUM test, for the upward jumps.
   * \param[in] nbPtsForStats The number of points to use to compute the mean and the standard deviation of the signal
   * to monitor.
   */
  void init(const float &delta, const float &limitDown, const float &limitUp, const unsigned int &nbPtsForStats);

  /**
   * \brief Initialize the mean adjusted CUSUM test.
   *
   * \param[in] h The alarm factor that permits to determine when the process is out of control from the standard
   * deviation of the signal.
   * \param[in] k The detection factor that permits to determine the slack of the CUSUM test, i.e. the
   * minimum value of the jumps we want to detect, from the standard deviation of the signal.
   * \param[in] mean The expected mean of the signal to monitor.
   * \param[in] stdev The expected standard deviation of the signal to monitor.
   */
  void init(const float &h, const float &k, const float &mean, const float &stdev);

  /**
   * \brief Initialize the mean adjusted CUSUM test.
   *
   * \param[in] delta The slack of the CUSUM test, i.e. the minimum value of the jumps we want to detect.
   * \param[in] limitDown The lower limit of the CUSUM test, for the downward jumps.
   * \param[in] limitUp The upper limit of the CUSUM test, for the upward jumps.
   * \param[in] mean The expected mean of the signal to monitor.
   * \param[in] stdev The expected standard deviation of the signal to monitor.
   */
  void init(const float &delta, const float &limitDown, const float &limitUp, const float &mean, const float &stdev);

  /**
   * \brief Set the slack of the CUSUM test, i.e. the minimum value of the jumps we want to detect.
   *
   * \param[in] delta The slack of the CUSUM test.
   */
  inline void setDelta(const float &delta)
  {
    m_delta = delta;
    m_half_delta = 0.5f * delta;
  }

  /**
   * \brief Set the upward and downward jump limits.
   *
   * \param[in] limitDown The limit for the downward jumps.
   * \param[in] limitUp The limit for the upward jumps.
   */
  inline void setLimits(const float &limitDown, const float &limitUp)
  {
    m_limitDown = limitDown;
    m_limitUp = limitUp;
  }
};
#endif
