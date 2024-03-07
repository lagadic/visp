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
 * \file vpStatisticalTestShewhart.h
 * \brief Statistical Process Control Shewhart's test implementation.
 */

#ifndef _vpStatisticalTestShewhartTest_h_
#define _vpStatisticalTestShewhartTest_h_

#include <vector>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpStatisticalTestSigma.h>

/**
 * \ingroup group_core_math_tools
 * \brief Class that permits a Shewhart test.
 */

class VISP_EXPORT vpStatisticalTestShewhart : public vpStatisticalTestSigma
{
public:
  typedef enum vpWecoRulesAlarm
  {
    THREE_SIGMA_WECO = 0,
    TWO_SIGMA_WECO = 1,
    ONE_SIGMA_WECO = 2,
    SAME_SIDE_WECO = 3,
    NONE_WECO = 4,
    COUNT_WECO = 5
  } vpWecoRulesAlarm;

  static std::string vpWecoRulesAlarmToString(const vpWecoRulesAlarm &alarm);

  static const bool CONST_ALL_WECO_ACTIVATED[COUNT_WECO - 1];

protected:
  static const int NB_DATA_SIGNAL = 8;
  unsigned int m_nbDataInBuffer; /*!< Indicate how many data are available in the circular buffer.*/
  float m_signal[NB_DATA_SIGNAL]; /*!< The last values of the signal.*/
  bool m_activateWECOrules; /*!< If true, activate the WECO rules (NB: it increases the sensitivity of the Shewhart
                                 control chart but the false alarm frequency is also increased.)*/
  bool m_activatedWECOrules[COUNT_WECO - 1]; /*!< The WECO rules that are activated. The more are activated, the higher the
                                              sensitivity of the Shewhart control chart is but the higher the false
                                              alarm frequency is.*/
  int m_idCurrentData; /*!< The index of the current data in m_signal.*/
  vpWecoRulesAlarm m_alarm; /*!< The type of alarm raised due to WECO rules.*/
  float m_oneSigmaNegLim; /*!< The mean - sigma lower threshold.*/
  float m_oneSigmaPosLim; /*!< The mean + sigma lower threshold.*/
  float m_twoSigmaNegLim; /*!< The mean - 2 sigma lower threshold.*/
  float m_twoSigmaPosLim; /*!< The mean + 2 sigma lower threshold.*/

  /**
   * \brief Compute the upper and lower limits of the test signal.
   */
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  virtual void computeLimits() override;
#else
  virtual void computeLimits();
#endif

/**
 * \brief Detects if a downward mean drift occured.
 *
 * \return \b vpMeanDriftType::MEAN_DRIFT_DOWNWARD if a downward mean drift occured, \b vpMeanDriftType::MEAN_DRIFT_NONE otherwise.
 *
 * \sa detectUpwardMeanDrift()
 */
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  virtual vpMeanDriftType detectDownwardMeanDrift() override;
#else
  virtual vpMeanDriftType detectDownwardMeanDrift();
#endif

  /**
   * \brief Detects if an upward mean drift occured on the mean.
   *
   * \return \b vpMeanDriftType::MEAN_DRIFT_UPWARD if an upward mean drift occured, \b vpMeanDriftType::MEAN_DRIFT_NONE otherwise.
   *
   * \sa detectDownwardMeanDrift()
   */
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  virtual vpMeanDriftType detectUpwardMeanDrift() override;
#else
  virtual vpMeanDriftType detectUpwardMeanDrift();
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
   * \brief Construct a new vpStatisticalTestShewhart object.
   *
   * \param[in] activateWECOrules If true, activate the WECO rules (NB: it increases the sensitivity of the Shewhart
   * control chart but the false alarm frequency is also increased.)
   * \param[in] nbSamplesForStats The number of samples to compute the statistics of the signal.
   */
  vpStatisticalTestShewhart(const bool &activateWECOrules = true, const bool activatedRules[COUNT_WECO - 1] = CONST_ALL_WECO_ACTIVATED, const unsigned int &nbSamplesForStats = 30);

  /**
   * \brief Construct a new vpStatisticalTestShewhart object.
   *
   * \param[in] activateWECOrules If true, activate the WECO rules (NB: it increases the sensitivity of the Shewhart
   * control chart but the false alarm frequency is also increased.)
   * \param[in] mean The expected mean of the signal.
   * \param[in] stdev The expected standard deviation of the signal.
   */
  vpStatisticalTestShewhart(const bool &activateWECOrules, const bool activatedRules[COUNT_WECO - 1], const float &mean, const float &stdev);

  /**
   * \brief Get the alarm raised by the last test due to the WECO rules.
   *
   * \return vpWecoRulesAlarm The type of raised alarm.
   */
  vpWecoRulesAlarm getAlarm() const
  {
    return m_alarm;
  }

  /**
   * \brief Get the last value of the signal.
   *
   * \return float The signal.
   */
  inline float getSignal() const
  {
    return m_signal[m_idCurrentData];
  }

  /**
   * \brief Get the NB_DATA_SIGNAL last signal values, sorted from the latest [0] to the newest [NB_DATA_SIGNAL - 1].
   *
   * \return std::vector<float> The last NB_DATA_SIGNAL values.
   */
  std::vector<float> getSignals() const;

  /**
   * \brief (Re)Initialize the test.
   *
   * \param[in] activateWECOrules If true, activate the WECO rules (NB: it increases the sensitivity of the Shewhart
   * control chart but the false alarm frequency is also increased.)
   * \param[in] nbSamplesForStats The number of samples to compute the statistics of the signal.
   */
  void init(const bool &activateWECOrules, const bool activatedRules[COUNT_WECO - 1] = CONST_ALL_WECO_ACTIVATED, const unsigned int &nbSamplesForStats = 30);

  /**
   * \brief (Re)Initialize the test.
   *
   * \param[in] activateWECOrules If true, activate the WECO rules (NB: it increases the sensitivity of the Shewhart
   * control chart but the false alarm frequency is also increased.)
   * \param[in] mean The expected mean of the signal.
   * \param[in] stdev The expected standard deviation of the signal.
   */
  void init(const bool &activateWECOrules, const bool activatedRules[COUNT_WECO - 1], const float &mean, const float &stdev);
};

#endif
