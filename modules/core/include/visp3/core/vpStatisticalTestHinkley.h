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
 * \file vpStatisticalTestHinkley.h
 * \brief Statistical Process Control Hinkley's test implementation.
 */

#ifndef _vpStatisticalTestHinkley_h_
#define _vpStatisticalTestHinkley_h_

#include <visp3/core/vpConfig.h>

#include <visp3/core/vpStatisticalTestAbstract.h>

BEGIN_VISP_NAMESPACE
/**
 * \ingroup group_core_math_tools
 * \brief This class implements the Hinkley's cumulative sum test.
 *
 * The Hinkley's cumulative sum test is designed to detect drift in the mean
  of an observed signal \f$ s(t) \f$. It is known to be robust (by
  taking into account all the past of the observed quantity),
  efficient, and inducing a very low computational load. The other
  attractive features of this test are two-fold. First, it can
  straightforwardly and accurately provide the drift instant. Secondly,
  due to its formulation (cumulative sum test), it can simultaneously
  handle both very abrupt and important changes, and gradual smaller
  ones without adapting the involved thresholds.

  Two tests are performed in parallel to look for downwards or upwards
  drifts in \f$ s(t) \f$, respectively defined by:

  \f[ S_k = \sum_{t=0}^{k} (s(t) - m_0 + \frac{\delta}{2}) \f]
  \f[ M_k = \max_{0 \leq i \leq k} S_i\f]
  \f[ T_k = \sum_{t=0}^{k} (s(t) - m_0 - \frac{\delta}{2}) \f]
  \f[ N_k = \min_{0 \leq i \leq k} T_i\f]

  In which \f$m_o\f$ is computed on-line and corresponds to the mean
  of the signal \f$ s(t) \f$ we want to detect a drift. \f$m_o\f$ is
  re-initialized at zero after each drift detection. \f$\delta\f$
  denotes the drift minimal magnitude that we want to detect and
  \f$\alpha\f$ is a predefined threshold. These values are set by
  default to 0.2 in the default constructor vpHinkley(). To modify the
  default values use setAlpha() and setDelta() or the
  vpHinkley(double alpha, double delta) constructor.

  A downward drift is detected if \f$ M_k - S_k > \alpha \f$.
  A upward drift is detected if \f$ T_k - N_k > \alpha \f$.

  To detect only downward drifts in \f$ s(t) \f$ use
  testDownwardMeanDrift().To detect only upward drifts in \f$ s(t) \f$ use
  testUpwardMeanDrift(). To detect both, downward and upward drifts use
  testDownUpwardMeanDrift().

  If a drift is detected, the drift location is given by the last instant
  \f$k^{'}\f$ when \f$ M_{k^{'}} - S_{k^{'}} = 0 \f$, or \f$ T_{k^{'}} -
  N_{k^{'}} = 0 \f$.
*/
class VISP_EXPORT vpStatisticalTestHinkley : public vpStatisticalTestAbstract
{
protected:
  float m_dmin2; /*!< Half of \f$\delta\f$, the drift minimal magnitude that we want to detect.*/
  float m_alpha; /*!< The \f$\alpha\f$ threshold indicating that a mean drift occurs. */
  float m_Sk; /*!< Test signal for downward mean drift.*/
  float m_Mk; /*!< Maximum of the test signal for downward mean drift \f$S_k\f$ .*/
  float m_Tk; /*!< Test signal for upward mean drift.*/
  float m_Nk; /*!< Minimum of the test signal for upward mean drift \f$T_k\f$*/
  bool m_computeDeltaAndAlpha; /*!< If true, compute \f$\delta\f$ and \f$\alpha\f$ from the standard deviation,
                                    the alarm factor and the detection factor.*/
  float m_h; /*!< The alarm factor, that permits to compute \f$\alpha\f$ from the standard deviation of the signal.*/
  float m_k; /*!< The detection factor, that permits to compute \f$\delta\f$ from the standard deviation of the signal.*/

  /**
   * \brief Compute \f$\delta\f$ and \f$\alpha\f$ from the standard deviation of the signal.
   */
  virtual void computeAlphaDelta();

  /**
   * \brief Compute the mean value \f$m_0\f$ of the signal. The mean value must be
   * computed before the mean drift is estimated on-line.
   *
   * \param[in] signal The new value of the signal to monitor.
   */
  void computeMean(double signal);

  /**
   * \brief Compute \f$S_k = \sum_{t=0}^{k} (s(t) - m_0 + \frac{\delta}{2})\f$
   *
   * \param[in] signal The new value of the signal to monitor.
   */
  void computeSk(double signal);

  /**
   * \brief Compute \f$M_k\f$, the maximum value of \f$S_k\f$.
   */
  void computeMk();

  /**
   * \brief Compute \f$T_k = \sum_{t=0}^{k} (s(t) - m_0 - \frac{\delta}{2})\f$
   *
   * \param[in] signal The new value of the signal to monitor.
   */
  void computeTk(double signal);

  /**
   * \brief Compute \f$N_k\f$, the minimum value of \f$T_k\f$.
   */
  void computeNk();

  /**
   * \brief Detects if a downward mean drift occurred.
   *
   * \return \b vpMeanDriftType::MEAN_DRIFT_DOWNWARD if a downward mean drift occurred, \b vpMeanDriftType::MEAN_DRIFT_NONE otherwise.
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
   * @brief Construct a new vpStatisticalTestHinkley object.
   * Call init() to initialise the Hinkley's test and set \f$\alpha\f$
   * and \f$\delta\f$ to default values.
   *
   * By default \f$ \delta = 0.2 \f$ and \f$ \alpha = 0.2\f$. Use
   * setDelta() and setAlpha() to modify these values.
   */
  vpStatisticalTestHinkley();

  /**
   * \brief Call init() to initialise the Hinkley's test and set \f$\alpha\f$
   * and \f$\delta\f$ thresholds.
   * \param[in] alpha : \f$\alpha\f$ threshold indicating that a mean drift occurs.
   * \param[in] delta : \f$\delta\f$ denotes the drift minimal magnitude that
   * we want to detect.
   * \param[in] nbSamplesForInit : number of signal samples to initialize the mean of the signal.
   *
   * \sa setAlpha(), setDelta()
   */
  vpStatisticalTestHinkley(const float &alpha, const float &delta, const unsigned int &nbSamplesForInit = 30);

  /**
   * \brief Construct a new vpStatisticalTestHinkley object. \f$\alpha\f$ and \f$\delta\f$ will be computed
   * from the standard deviation of the signal.
   *
   * \param[in] h : the alarm factor that permits to compute \f$\alpha\f$ from the standard deviation.
   * \param[in] k : the detection factor that permits to compute \f$\delta\f$ from the standard deviation.
   * \param[in] computeAlphaDeltaFromStdev : must be equal to true, otherwise throw a vpException.
   * \param[in] nbSamplesForInit : number of signal samples to initialize the mean of the signal.
   */
  vpStatisticalTestHinkley(const float &h, const float &k, const bool &computeAlphaDeltaFromStdev, const unsigned int &nbSamplesForInit = 30);

  /**
   * \brief Construct a new vpStatisticalTestHinkley object. \f$\alpha\f$ and \f$\delta\f$ will be computed
   * from the standard deviation of the signal.
   *
   * \param[in] h : the alarm factor that permits to compute \f$\alpha\f$ from the standard deviation.
   * \param[in] k : the detection factor that permits to compute \f$\delta\f$ from the standard deviation.
   * \param[in] mean : the expected mean of the signal.
   * \param[in] stdev : the expected standard deviation of the signal.
   */
  vpStatisticalTestHinkley(const float &h, const float &k, const float &mean, const float &stdev);

  /**
   * \brief Destroy the vpStatisticalTestHinkley object.
   */
  virtual ~vpStatisticalTestHinkley();

  /**
   * \brief Get the \f$\alpha\f$ threshold indicating that a mean drift occurs.
   *
   * \return The \f$\alpha\f$ threshold.
   */
  inline float getAlpha() const { return m_alpha; }

  /*!
   * \brief Get the test signal for downward mean drift.
   *
   * \return The value of \f$S_k = \sum_{t=0}^{k} (s(t) - m_0 + \frac{\delta}{2})\f$ .
  */
  inline float getSk() const { return m_Sk; }

  /*!
   * \brief Get the maximum of the test signal for downward mean drift \f$S_k\f$ .
   *
   * \return The value of \f$M_k\f$, the maximum value of \f$S_k\f$.
  */
  inline float getMk() const { return m_Mk; }

  /*!
   * \brief Get the test signal for upward mean drift..
   *
   * \return The value of \f$T_k = \sum_{t=0}^{k} (s(t) - m_0 - \frac{\delta}{2})\f$ .

  */
  inline float getTk() const { return m_Tk; }

  /*!
   * \brief Get the minimum of the test signal for upward mean drift \f$T_k\f$
   *
   * \return The value of \f$N_k\f$, the minimum value of \f$T_k\f$.
  */
  inline float getNk() const { return m_Nk; }

  /**
   * \brief Initialise the Hinkley's test by setting the mean signal value
   * \f$m_0\f$ to zero as well as \f$S_k, M_k, T_k, N_k\f$.
   */
  void init();

  /**
   * \brief Call init() to initialise the Hinkley's test and set \f$\alpha\f$
   * and \f$\delta\f$ thresholds.
   *
   * \param[in] alpha The threshold indicating that a mean drift occurs.
   * \param[in] delta The drift minimal magnitude that we want to detect.
   * \param[in] nbSamplesForInit : number of signal samples to initialize the mean of the signal.
   */
  void init(const float &alpha, const float &delta, const unsigned int &nbSamplesForInit);

  /**
   * \brief (Re)Initialize a new vpStatisticalTestHinkley object. \f$\alpha\f$ and \f$\delta\f$ will be computed
   * from the standard deviation of the signal.
   *
   * \param[in] h : the alarm factor that permits to compute \f$\alpha\f$ from the standard deviation.
   * \param[in] k : the detection factor that permits to compute \f$\delta\f$ from the standard deviation.
   * \param[in] computeAlphaDeltaFromStdev : must be equal to true, otherwise throw a vpException.
   * \param[in] nbSamplesForInit : number of signal samples to initialize the mean of the signal.
   */
  void init(const float &h, const float &k, const bool &computeAlphaDeltaFromStdev, const unsigned int &nbSamplesForInit);

  /**
   * \brief Call init() to initialise the Hinkley's test, set \f$\alpha\f$
   * and \f$\delta\f$ thresholds, and the mean of the signal \f$m_0\f$.
   *
   * \param[in] alpha The threshold indicating that a mean drift occurs.
   * \param[in] delta The drift minimal magnitude that we want to detect.
   * \param[in] mean The expected value of the mean.
   */
  void init(const float &alpha, const float &delta, const float &mean);

  /**
   * \brief (Re)Initialize a new vpStatisticalTestHinkley object. \f$\alpha\f$ and \f$\delta\f$ will be computed
   * from the standard deviation of the signal.
   *
   * \param[in] h : the alarm factor that permits to compute \f$\alpha\f$ from the standard deviation.
   * \param[in] k : the detection factor that permits to compute \f$\delta\f$ from the standard deviation.
   * \param[in] mean : the expected mean of the signal.
   * \param[in] stdev : the expected standard deviation of the signal.
   */
  void init(const float &h, const float &k, const float &mean, const float &stdev);

  /**
   * \brief Set the drift minimal magnitude that we want to detect.
   *
   * \param[in] delta The drift magnitude.
   */
  void setDelta(const float &delta);

  /**
   * \brief The threshold indicating that a mean drift occurs.
   *
   * \param[in] alpha The threshold.
   */
  void setAlpha(const float &alpha);
};
END_VISP_NAMESPACE
#endif
