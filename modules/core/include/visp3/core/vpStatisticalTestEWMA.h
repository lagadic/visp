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
 * \file vpStatisticalTestEWMA.h
 * \brief Statistical Process Control Exponentially Weighted Moving Average implementation.
 */

#ifndef _vpStatisticalTestEWMA_h_
#define _vpStatisticalTestEWMA_h_

#include <visp3/core/vpConfig.h>

#include <visp3/core/vpStatisticalTestAbstract.h>

BEGIN_VISP_NAMESPACE
/**
 * \ingroup group_core_math_tools
 * \brief Class that permits to perform Exponentially Weighted Moving Average mean drft tests.
 *
 * The EWMA test is designed to detect drift in the mean \f$ \mu \f$
 * of an observed signal \f$ s(t) \f$.
 *
 * The test signal \f$ w(t) \f$ is computed as follow:
 *
 * \f$ w(0) = \mu \f$
 *
 * \f$ w(t) = \alpha s(t) + ( 1 - \alpha ) * w(t-1) \f$
 *
 * Be \f$ \sigma \f$ the standard deviation of the input signal \f$ s(t) \f$.
 *
 * A downward alarm is raised if:
 * \f$ w(t) <= \mu - 3 * \sigma * \sqrt{ \frac{\alpha}{2 - \alpha}}\f$
 *
 * An upward alarm is raised if:
 * \f$ w(t) >= \mu + 3 * \sigma * \sqrt{ \frac{\alpha}{2 - \alpha}}\f$
 *
 * To detect only downward drifts of the input signal \f$ s(t) \f$ use
 * testDownwardMeanDrift().To detect only upward drifts in \f$ s(t) \f$ use
 * testUpwardMeanDrift(). To detect both, downward and upward drifts use
 * testDownUpwardMeanDrift().
*/
class VISP_EXPORT vpStatisticalTestEWMA : public vpStatisticalTestAbstract
{
protected:
  float m_alpha; /*!< Forgetting factor: the higher, the more weight the current signal value has.*/
  float m_wt; /*!< Test signal that permits to raise an alarm.*/
  float m_wtprev; /*!< Previous value of the test signal.*/

  /**
   * \brief Compute the upper and lower limits of the test signal.
   */
  virtual void computeDeltaAndLimits();

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
   * \brief Construct a new vpStatisticalTestEWMA object.
   *
   * \param[in] alpha The forgetting factor.
   */
  vpStatisticalTestEWMA(const float &alpha = 0.1f);

  /**
   * \brief Get the forgetting factor of the algorithm.
   *
   * \return float The forgetting factor.
   */
  inline float getAlpha() const
  {
    return m_alpha;
  }

  /**
   * \brief Get the current value of the test signal.
   *
   * \return float The current value of the test signal.
   */
  inline float getWt() const
  {
    return m_wt;
  }

  /**
   * \brief Initialize the EWMA algorithm.
   *
   * \param[in] alpha The forgetting factor.
   */
  void init(const float &alpha);

  /**
   * \brief Initialize the EWMA algorithm.
   *
   * \param[in] alpha The forgetting factor.
   * \param[in] mean The expected mean of the signal to monitor.
   * \param[in] stdev The expected standard deviation of the signal to monitor.
   */
  void init(const float &alpha, const float &mean, const float &stdev);

  /**
   * \brief Set the forgetting factor.
   *
   * \param[in] alpha The forgetting factor.
   */
  void setAlpha(const float &alpha);
};
END_VISP_NAMESPACE
#endif
