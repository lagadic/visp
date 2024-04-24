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
  /**
   * \brief Command model function, which projects effect of the command
   * on the state.
   * The first argument is the command(s), the second is the period and the return is the
   * effect of the command on the state after period seconds.
   */
  typedef std::function<vpColVector(const vpColVector &, const double &)> vpCommandOnlyFunction;

  /**
   * \brief Command model function, which projects effect of the command
   * on the state, when the effect of the command depends on the current state.
   * The first argument is the command(s), the second is the state and the third is the period.
   * The return is the effect of the command on the state after period seconds.
   */
  typedef std::function<vpColVector(const vpColVector &, const vpColVector &, const double &)> vpCommandStateFunction;

  /**
   * \brief Mean function, which computes the weighted mean of either the prior or the
   * prior expressed in the measurement space.
   * The first argument is either the prior or the prior expressed in the measurement space
   * and the second argument is the associated vector of weights. The return is the
   * corresponding mean.
   */
  typedef std::function<vpColVector(const std::vector<vpColVector> &, const std::vector<double> &)> vpMeanFunction;

  /**
   * \brief Measurement function, which converts the prior points in the measurement space.
   * The argument is a point of a prior point and the return is its projection in the measurement
   * space.
   */
  typedef std::function<vpColVector(const vpColVector &)> vpMeasurementFunction;

  /**
   * \brief Process model function, which projects the sigma points forward in time.
   * The first argument is a sigma point, the second is the period and the return is the
   * sigma point projected in the future (i.e. a point of the prior).
   */
  typedef std::function<vpColVector(const vpColVector &, const double &)> vpProcessFunction;

  /**
   * \brief Residual function, which computes either the equivalent of the substraction in the
   * state space or the equivalent of the substraction in the measurement space.
   * The first argument is the vector to which we must substract something
   * and the second argument is the thing to be substracted. The return is the
   * result of this "substraction".
   */
  typedef std::function<vpColVector(const vpColVector &, const vpColVector &)> vpResidualFunction;

  /**
   * \brief Residual function, which computes either the equivalent of the substraction in the
   * state space or the equivalent of the substraction in the measurement space.
   * The first argument is the vector to which we must substract something
   * and the second argument is the thing to be substracted. The return is the
   * result of this "substraction".
   */
  typedef std::function<vpColVector(const vpColVector &, const vpColVector &)> vpAddFunction;

  /**
   * \brief Construct a new vpUnscentedKalman object.
   *
   * \param[in] Q The covariance introduced by performing the prediction step.
   * \param[in] R The covariance introduced by performing the update step.
   * \param[in] drawer Object that permits to draw the sigma points.
   * \param[in] f Process model function, which projects the sigma points forward in time.
   * The first argument is a sigma point, the second is the period and the return is the
   * sigma point projected in the future (i.e. a point of the prior).
   * \param[in] h Measurement function, which converts the prior points in the measurement space.
   * The argument is a point of a prior point and the return is its projection in the measurement
   * space.
   */
  vpUnscentedKalman(const vpMatrix &Q, const vpMatrix &R, vpUKSigmaDrawerAbstract *drawer, const vpProcessFunction &f, const vpMeasurementFunction &h);

  /**
   * \brief Set the guess of the initial state and covariance.
   *
   * \param[in] mu0 Guess of the initial state.
   * \param[in] P0 Guess of the initial covariance.
   */
  void init(const vpColVector &mu0, const vpMatrix &P0);

  /**
   * \brief Set the measurement mean function to use when computing a mean
   * in the measurement space.
   *
   * \param meanFunc The mean function to use.
   */
  inline void setMeasurementMeanFunction(const vpMeanFunction &meanFunc)
  {
    m_measMeanFunc = meanFunc;
  }

  /**
   * \brief Set the measurement residual function to use when computing a substraction
   * in the measurement space.
   *
   * \param measResFunc The residual function to use.
   */
  inline void setMeasurementResidualFunction(const vpResidualFunction &measResFunc)
  {
    m_measResFunc = measResFunc;
  }

  /**
   * \brief Set the state mean function to use when computing a mean
   * in the state space.
   *
   * \param meanFunc The mean function to use.
   */
  inline void setStateMeanFunction(const vpMeanFunction &meanFunc)
  {
    m_stateMeanFunc = meanFunc;
  }

  /**
   * \brief Set the state residual function to use when computing a substraction
   * in the state space.
   *
   * \param stateResFunc The residual function to use.
   */
  inline void setStateResidualFunction(const vpResidualFunction &stateResFunc)
  {
    m_stateResFunc = stateResFunc;
  }

  /**
   * \brief Perform first the prediction step and then the filtering step.
   *
   * \param[in] z The new measurement.
   * \param[in] dt The time in the future we must predict.
   * \param[in] u The command(s) given to the system, if the impact of the system is known.
   *
   * \warning To use the commands, the method vpUnscentedKalman::setCommandOnlyFunction or
   * vpUnscentedKalman::setCommandStateFunction must be called beforehand.
   */
  void filter(const vpColVector &z, const double &dt, const vpColVector &u = vpColVector());

  /**
   * \brief Predict the new state based on the last state and how far in time we want to predict.
   *
   * \param[in] dt The time in the future we must predict.
   * \param[in] u The command(s) given to the system, if the impact of the system is known.
   *
   * \warning To use the commands, the method vpUnscentedKalman::setCommandOnlyFunction or
   * vpUnscentedKalman::setCommandStateFunction must be called beforehand.
   */
  void predict(const double &dt, const vpColVector &u = vpColVector());

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

  /**
   * \brief Simple function to compute an addition, which just does \f$ \textbf{res} = \textbf{a} + \textbf{toAdd} \f$
   *
   * \param[in] a Vector to which we must add something.
   * \param[in] toAdd The something we must add to \b a .
   * \return vpColVector \f$ \textbf{res} = \textbf{a} + \textbf{toAdd} \f$
   */
  inline static vpColVector simpleAdd(const vpColVector &a, const vpColVector &toAdd)
  {
    vpColVector res = a + toAdd;
    return res;
  }

  /**
   * \brief Simple function to compute a residual, which just does \f$ \textbf{res} = \textbf{a} - \textbf{toSubstract} \f$
   *
   * \param[in] a Vector to which we must substract something.
   * \param[in] toSubstract The something we must substract to \b a .
   * \return vpColVector \f$ \textbf{res} = \textbf{a} - \textbf{toSubstract} \f$
   */
  inline static vpColVector simpleResidual(const vpColVector &a, const vpColVector &toSubstract)
  {
    vpColVector res = a - toSubstract;
    return res;
  }

  /**
   * \brief Simple function to compute a mean, which just does \f$ \textbf{\mu} = \sum_{i} wm_i \textbf{vals}_i \f$
   *
   * \param[in] vals Vector containing all the vectors we must compute the mean.
   * \param[in] wm The correspond list of weights.
   * \return vpColVector \f$ \textbf{\mu} = \sum_{i} wm_i \textbf{vals}_i \f$
   */
  inline static vpColVector simpleMean(const std::vector<vpColVector> &vals, const std::vector<double> &wm)
  {
    unsigned int nbPoints = vals.size();
    if (nbPoints == 0) {
      throw(vpException(vpException::dimensionError, "No points to add when computing the mean"));
    }
    vpColVector mean = vals[0] * wm[0];
    for (unsigned int i = 1; i < nbPoints; ++i) {
      mean += vals[i] * wm[i];
    }
    return mean;
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
  vpProcessFunction m_f; /*!< Process model function, which projects the sigma points forward in time.*/
  vpMeasurementFunction m_h; /*!< Measurement function, which converts the sigma points in the measurement space.*/
  vpUKSigmaDrawerAbstract *m_sigmaDrawer; /*!< Object that permits to draw the sigma points.*/
  vpCommandOnlyFunction m_b; /*!< Function that permits to compute the effect of the commands on the prior, without knowledge of the state.*/
  vpCommandStateFunction m_bx; /*!< Function that permits to compute the effect of the commands on the prior, with knowledge of the state.*/
  vpMeanFunction m_measMeanFunc; /*!< Function to compute a weighted mean in the measurement space.*/
  vpResidualFunction m_measResFunc; /*!< Function to compute a substraction in the measurement space.*/
  std::function<vpColVector(const vpColVector &, const vpColVector &)> m_stateAddFunction; /*!< Function to compute an addition in the state space.*/
  vpMeanFunction m_stateMeanFunc; /*!< Function to compute a weighted mean in the state space.*/
  vpResidualFunction m_stateResFunc; /*!< Function to compute a substraction in the state space.*/

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
  static vpUnscentedTransformResult unscentedTransform(const std::vector<vpColVector> &sigmaPoints, const std::vector<double> &wm,
    const std::vector<double> &wc, const vpMatrix &cov, const vpResidualFunction &resFunc, const vpMeanFunction &meanFunc);
};

#endif
