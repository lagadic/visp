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
 * \file vpStatisticalTestSigma.h
 * \brief Statistical Process Control sigma test implementation.
 */

#ifndef _vpStatisticalTestSigma_h_
#define _vpStatisticalTestSigma_h_

#include <visp3/core/vpConfig.h>

#include <visp3/core/vpStatisticalTestAbstract.h>

/**
 * \ingroup group_core_math_tools
 * \brief Class that permits a simple test comparing the current value to the
 * standard deviation of the signal.
 */

class VISP_EXPORT vpStatisticalTestSigma : public vpStatisticalTestAbstract
{
protected:
  float m_h; /*!< The alarm factor applied to the standard deviation to compute the limits.*/
  float m_s; /*!< The last value of the signal.*/

  /**
   * \brief Compute the upper and lower limits of the test signal.
   */
  virtual void computeLimits();

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
  inline float getS() const
  {
    return m_s;
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

#endif
