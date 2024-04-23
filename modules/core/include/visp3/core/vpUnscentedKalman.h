/*
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
 *
 * Description:
 * Display a point cloud using PCL library.
 */

#ifndef _vpUnscentedKalman_h_
#define _vpUnscentedKalman_h_

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpUKSigmaDrawerAbstract.h>

#include <functional>

/*!
  \class vpUnscentedKalman
  \ingroup group_core_kalman
  This class permits to use Unscented Kalman filter to tackle non-linear problems.
*/
class VISP_EXPORT vpUnscentedKalman
{
public:
  typedef std::function<std::vector<vpColVector>(const std::vector<vpColVector> &, const double &)> process_function;
  typedef std::function<std::vector<vpColVector>(const std::vector<vpColVector> &)> measurement_function;

  /**
   * \brief Construct a new vpUnscentedKalman object.
   *
   * \param[in] Q The covariance introduced by performing the prediction step.
   * \param[in] R The covariance introduced by performing the update step.
   * \param[in] drawer Object that permits to draw the sigma points.
   * \param[in] f Process model function, which projects the sigma points forward in time.
   * \param[in] h Measurement function, which converts the sigma points in the measurement space.
   */
  vpUnscentedKalman(const vpMatrix &Q, const vpMatrix &R, vpUKSigmaDrawerAbstract *drawer, const process_function &f, const measurement_function &h);

  /**
   * \brief Set the guess of the initial state and covariance.
   *
   * \param[in] mu0 Guess of the initial state.
   * \param[in] P0 Guess of the initial covariance.
   */
  void init(const vpColVector &mu0, const vpMatrix &P0);

  /**
   * \brief Perform first the prediction step and then the filtering step.
   *
   * \param[in] z The new measurement.
   * \param[in] dt The time in the future we must predict.
   */
  void filter(const vpColVector &z, const double &dt);

  /**
   * \brief Predict the new state based on the last state and how far in time we want to predict.
   *
   * \param[in] dt The time in the future we must predict.
   */
  void predict(const double &dt);

  /**
   * \brief Update the estimate of the state based on a new measurement.
   *
   * \param[in] z The measurements at the current timestep.
   */
  void update(const vpColVector &z);

  /**
   * \brief Get the estimated (i.e. filtered) state.
   *
   * \return vpColVector The estimated state.
   */
  inline vpColVector getXest() const
  {
    return m_Xest;
  }

  /**
   * \brief Get the predicted state (i.e. the prior).
   *
   * \return vpColVector The predicted state.
   */
  inline vpColVector getXpred() const
  {
    return m_mu;
  }
private:
  vpColVector m_Xest; /*!< The estimated (i.e. filtered) state variables.*/
  vpMatrix m_Pest; /*!< The estimated (i.e. filtered) covariance matrix.*/
  vpMatrix m_Q; /*!< The covariance introduced by performing the prediction step.*/
  std::vector<vpColVector> m_chi; /*!< The sigma points.*/
  std::vector<double> m_wm; /*!< The weights for the mean computation.*/
  std::vector<double> m_wc; /*!< The weights for the covariance computation.*/
  std::vector<vpColVector> m_Y; /*!< The projection forward in time of the sigma points according to the process model, called the prior.*/
  vpColVector m_mu; /*!< The mean of the prior.*/
  vpMatrix m_P; /*!< The covariance matrix of the prior.*/
  vpMatrix m_R; /*!< The covariance introduced by performing the update step.*/
  std::vector<vpColVector> m_Z; /*!< The sigma points of the prior expressed in the measurement space, called the measurement sigma points.*/
  vpColVector m_muz; /*!< The mean of the measurement sigma points.*/
  vpMatrix m_Pz; /*!< The covariance matrix of the measurement sigma points.*/
  vpMatrix m_Pxz; /*!< The cross variance of the state and the measurements.*/
  vpColVector m_y; /*!< The residual.*/
  vpMatrix m_K; /*!< The Kalman gain.*/
  process_function m_f; /*!< Process model function, which projects the sigma points forward in time.*/
  measurement_function m_h; /*!< Measurement function, which converts the sigma points in the measurement space.*/
  vpUKSigmaDrawerAbstract *m_sigmaDrawer; /*!< Object that permits to draw the sigma points.*/

  /**
   * \brief Structure that stores the results of the unscented transform.
   *
   */
  typedef struct vpUnscentedTransformResult
  {
    vpColVector m_mu; /*!< The mean.*/
    vpMatrix m_P; /*!< The covariance matrix.*/
  } vpUnscentedTransformResult;

  /**
   * \brief Compute the unscented transform of the sigma points.
   *
   * \param[in] sigmaPoints The sigma points we consider.
   * \param[in] wm The weights to apply for the mean computation.
   * \param[in] wc The weights to apply for the covariance computation.
   * \param[in] cov The constant covariance matrix to add to the computed covariance matrix.
   * \return vpUnscentedTransformResult The mean and covariance of the sigma points.
   */
  static vpUnscentedTransformResult unscentedTransform(const std::vector<vpColVector> &sigmaPoints, const vpColVector &wm, const vpColVector &wc, const vpMatrix &cov);
};

#endif
