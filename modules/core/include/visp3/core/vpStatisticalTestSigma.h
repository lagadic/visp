/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 * \file vpStatisticalTestSigma.h
 * \brief Statistical Process Control sigma test implementation.
 */

#ifndef VP_STATISTICAL_TEST_SIGMA_H
#define VP_STATISTICAL_TEST_SIGMA_H

#include <visp3/core/vpConfig.h>

#include <visp3/core/vpStatisticalTestAbstract.h>

BEGIN_VISP_NAMESPACE
/**
 * \ingroup group_core_spc
 * \brief Class that permits a simple test comparing the current value to the
 * standard deviation of the signal.
 *
 * Be \f$ s(t) \f$ the signal to monitor, \f$ \mu \f$ and \f$ \sigma \f$ the mean and standard deviation
 * of this signal when it is "in control".
 *
 * Be \f$ h \f$ a user-defined alarm factor.
 *
 * A downward alarm is raised if:
 * \f$ s(t) >= \mu - h \sigma \f$
 *
 * An upward alarm is raised if:
 * \f$ s(t) >= \mu - h \sigma \f$
 *
 * \f$ h \f$ is often set to 3 if we assume the \f$ s(t) \f$ follows a normal distribution.
 *
 * To detect only downward drifts of the input signal \f$ s(t) \f$ use
 * testDownwardMeanDrift().To detect only upward drifts in \f$ s(t) \f$ use
 * testUpwardMeanDrift(). To detect both, downward and upward drifts use
 * testDownUpwardMeanDrift().
 *
 * <h2 id="header-details" class="groupheader">Tutorials & Examples</h2>
 *
 * <b>Tutorials</b><br>
 * <span style="margin-left:2em"> If you are interested in using Statistical Process Control methods, you may have a look at:</span><br>
 *
 * - \ref tutorial-spc
*/

class VISP_EXPORT vpStatisticalTestSigma : public vpStatisticalTestAbstract
{
protected:
  float m_h; /*!< The alarm factor applied to the standard deviation to compute the limits.*/
  float m_signal; /*!< The last value of the signal.*/

  /**
   * \brief Compute the upper and lower limits of the test signal.
   */
  virtual void computeLimits();

  /**
   * \brief Detects if a downward mean drift occurred.
   *
   * \return \b vpMeanDriftType::MEAN_DRIFT_DOWNWARD if a downward mean drift occurred, \b vpMeanDriftType::MEAN_DRIFT_NONE otherwise.
   *
   * \sa detectUpwardMeanDrift()
   */
  virtual vpMeanDriftType detectDownwardMeanDrift() VP_OVERRIDE;

  /**
   * \brief Detects if an upward mean drift occurred on the mean.
   *
   * \return \b vpMeanDriftType::MEAN_DRIFT_UPWARD if an upward mean drift occurred, \b vpMeanDriftType::MEAN_DRIFT_NONE otherwise.
   *
   * \sa detectDownwardMeanDrift()
   */
  virtual vpMeanDriftType detectUpwardMeanDrift() VP_OVERRIDE;

  /**
   * \brief Update m_s and if enough values are available, compute the mean, the standard
   * deviation and the limits.
   *
   * \param[in] signal The new value of the signal to monitor.
   */
  virtual bool updateStatistics(const float &signal) VP_OVERRIDE;

  /**
   * \brief Update the test signals.
   *
   * \param[in] signal The new value of the signal to monitor.
   */
  virtual void updateTestSignals(const float &signal) VP_OVERRIDE;

public:
  /**
   * \brief Construct a new vpStatisticalTestSigma object.
   *
   * \param[in] h The alarm factor applied to the standard deviation to compute the limits.
   * \param[in] nbSamplesForStats The number of samples to compute the statistics of the signal.
   */
  vpStatisticalTestSigma(const float &h = 3.f, const unsigned int &nbSamplesForStats = 30);

  /**
   * \brief Construct a new vpStatisticalTestSigma object.
   *
   * \param[in] h The alarm factor applied to the standard deviation to compute the limits.
   * \param[in] mean The expected mean of the signal.
   * \param[in] stdev The expected standard deviation of the signal.
   */
  vpStatisticalTestSigma(const float &h, const float &mean, const float &stdev);

  /**
   * \brief Get the last value of the signal.
   *
   * \return float The signal.
   */
  inline virtual float getSignal() const
  {
    return m_signal;
  }

  /**
   * \brief (Re)Initialize the test.
   *
   * \param[in] h The alarm factor applied to the standard deviation to compute the limits.
   * \param[in] nbSamplesForStats The number of samples to compute the statistics of the signal.
   */
  void init(const float &h = 3.f, const unsigned int &nbSamplesForStats = 30);

  /**
   * \brief (Re)Initialize the test.
   *
   * \param[in] h The alarm factor applied to the standard deviation to compute the limits.
   * \param[in] mean The expected mean of the signal.
   * \param[in] stdev The expected standard deviation of the signal.
   */
  void init(const float &h, const float &mean, const float &stdev);
};
END_VISP_NAMESPACE
#endif
