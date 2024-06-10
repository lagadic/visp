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

/*!
 * \file vpStatisticalTestAbstract.h
 * \brief Base class for Statistical Process Control methods implementation.
 */

#ifndef _vpStatisticalTestAbstract_h_
#define _vpStatisticalTestAbstract_h_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include <visp3/core/vpConfig.h>

BEGIN_VISP_NAMESPACE
/**
 * \ingroup group_core_math_tools
 * \brief Base class for methods detecting the drift of the mean of a process.
 *
 * To detect only downward drifts of the input signal \f$ s(t) \f$ use
 * testDownwardMeanDrift().To detect only upward drifts in \f$ s(t) \f$ use
 * testUpwardMeanDrift(). To detect both, downward and upward drifts use
 * testDownUpwardMeanDrift().
*/
class VISP_EXPORT vpStatisticalTestAbstract
{
public:
  /**
   * \brief Enum that indicates if a drift of the mean occurred.
   */
  typedef enum vpMeanDriftType
  {
    MEAN_DRIFT_NONE = 0, /*!< No mean drift occurred*/
    MEAN_DRIFT_DOWNWARD = 1, /*!< A downward drift of the mean occurred.*/
    MEAN_DRIFT_UPWARD = 2, /*!< An upward drift of the mean occurred.*/
    MEAN_DRIFT_BOTH = 3, /*!< Both an aupward and a downward drifts occurred.*/
    MEAN_DRIFT_COUNT = 4,
    MEAN_DRIFT_UNKNOWN = MEAN_DRIFT_COUNT
  } vpMeanDriftType;

  /**
   * \brief Cast a \b vpMeanDriftType into a string.
   *
   * \param[in] type The type of mean drift.
   * \return std::string The corresponding message.
   */
  static std::string vpMeanDriftTypeToString(const vpMeanDriftType &type);

  /**
   * \brief Cast a string into a \b vpMeanDriftType.
   *
   * \param[in] name The name of the mean drift.
   * \return vpMeanDriftType The corresponding \b vpMeanDriftType.
   */
  static vpMeanDriftType vpMeanDriftTypeFromString(const std::string &name);

  /**
   * \brief Get the list of available \b vpMeanDriftType objects that are handled.
   *
   * \param[in] prefix The prefix that should be placed before the list.
   * \param[in] sep The separator between each element of the list.
   * \param[in] suffix The suffix that should terminate the list.
   * \return std::string The list of handled type of process tests, presented as a string.
   */
  static std::string getAvailableMeanDriftType(const std::string &prefix = "<", const std::string &sep = " , ",
                                               const std::string &suffix = ">");

  /**
   * \brief Print the message corresponding to the type of mean drift.
   *
   * \param[in] type The type of mean drift.
   */
  static void print(const vpMeanDriftType &type);

protected:
  bool m_areStatisticsComputed; /*!< Set to true once the mean and the standard deviation are available.*/
  float m_count; /*!< Current number of data used to compute the mean and the standard deviation.*/
  float m_limitDown; /*!< Upper limit for the test signal m_wt.*/
  float m_limitUp; /*!< Lower limit for the test signal m_wt*/
  float m_mean; /*!< Mean of the monitored signal.*/
  unsigned int m_nbSamplesForStatistics; /*!< Number of samples to use to compute the mean and the standard deviation.*/
  float *m_s; /*!< Array that keeps the samples used to compute the mean and standard deviation.*/
  float m_stdev; /*!< Standard deviation of the monitored signal.*/
  float m_stdevmin; /*!< Minimum allowed standard deviation of the monitored signal.*/
  float m_sumForMean; /*!< Sum of the samples used to compute the mean and standard deviation.*/

  /**
   * \brief Detects if a downward mean drift occurred.
   *
   * \return \b vpMeanDriftType::MEAN_DRIFT_DOWNWARD if a downward mean drift occurred, \b vpMeanDriftType::MEAN_DRIFT_NONE otherwise.
   *
   * \sa detectUpwardMeanDrift()
   */
  virtual vpMeanDriftType detectDownwardMeanDrift() = 0;

  /**
   * \brief Detects if a upward mean drift occurred.
   *
   * \return \b vpMeanDriftType::MEAN_DRIFT_UPWARD if an upward mean drift occurred, \b vpMeanDriftType::MEAN_DRIFT_NONE otherwise.
   *
   * \sa detectDownwardMeanDrift()
   */
  virtual vpMeanDriftType detectUpwardMeanDrift() = 0;

  /**
   * \brief Update \b m_s and if enough values are available, compute the mean, the standard
   * deviation and the limits.
   *
   * \param[in] signal The new value of the signal to monitor.
   *
   * \return true if the statistics have been computed, false if data are missing.
   */
  virtual bool updateStatistics(const float &signal);

  /**
   * \brief Update the test signals.
   *
   * \param[in] signal The new value of the signal to monitor.
   */
  virtual void updateTestSignals(const float &signal) = 0;
public:
  /**
   * \brief Construct a new vpStatisticalTestAbstract object.
   */
  vpStatisticalTestAbstract();

  /**
   * \brief Construct by copy a new vpStatisticalTestAbstract object.
   */
  vpStatisticalTestAbstract(const vpStatisticalTestAbstract &other);

  /**
   * \brief Destroy the vpStatisticalTestAbstract object.
   */
  virtual ~vpStatisticalTestAbstract();

  /**
   * \brief Get the upper and lower limits of the test signal.
   *
   * \param[out] limitDown The lower limit.
   * \param[out] limitUp The upper limit.
   */
  inline void getLimits(float &limitDown, float &limitUp) const
  {
    limitDown = m_limitDown;
    limitUp = m_limitUp;
  }

  /**
   * \brief Get the mean used as reference.
   *
   * \return float The mean.
   */
  inline float getMean() const
  {
    return m_mean;
  }

  /**
   * \brief Get the standard deviation used as reference.
   *
   * \return float The standard deviation.
   */
  inline float getStdev() const
  {
    return m_stdev;
  }

  /**
   * \brief (Re)Initialize the algorithm.
   */
  void init();

  /**
   * \brief Copy operator of a vpStatisticalTestAbstract.
   *
   * \param[in] other The vpStatisticalTestAbstract to copy.
   * \return vpStatisticalTestAbstract& *this after copy.
   */
  vpStatisticalTestAbstract &operator=(const vpStatisticalTestAbstract &other);

  /**
   * \brief Set the minimum value of the standard deviation that is expected.
   * The computed standard deviation cannot be lower this value if set.
   *
   * \param[in] stdevmin The minimum value of the standard deviation that is expected.
   */
  void setMinStdev(const float &stdevmin)
  {
    m_stdevmin = stdevmin;
  }

  /**
   * \brief Set the number of samples required to compute the mean and standard deviation
   * of the signal and allocate the memory accordingly.
   *
   * \param[in] nbSamples The number of samples we want to use.
   */
  void setNbSamplesForStat(const unsigned int &nbSamples);

  /**
   * \brief Test if a downward or an upward mean drift occurred
   * according to the new value of the signal.
   *
   * \param[in] signal The new value of the signal.
   * \return vpMeanDriftType The type of mean drift that occurred.
   *
   * \sa testDownwardMeanDrift() testUpwardMeanDrift()
   */
  vpMeanDriftType testDownUpwardMeanDrift(const float &signal);

  /**
   * \brief Test if a downward mean drift occurred
   * according to the new value of the signal.
   *
   * \param[in] signal The new value of the signal.
   * \return vpMeanDriftType The type of mean drift that occurred.
   *
   * \sa testUpwardMeanDrift()
   */
  vpMeanDriftType testDownwardMeanDrift(const float &signal);

  /**
   * \brief Test if an upward mean drift occurred
   * according to the new value of the signal.
   *
   * \param[in] signal The new value of the signal.
   * \return vpMeanDriftType The type of mean drift that occurred.
   *
   * \sa testDownwardMeanDrift()
   */
  vpMeanDriftType testUpwardMeanDrift(const float &signal);
};
END_VISP_NAMESPACE
#endif
