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

#ifndef VP_UNSCENTED_KALMAN_H
#define VP_UNSCENTED_KALMAN_H

#include <visp3/core/vpConfig.h>

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpUKSigmaDrawerAbstract.h>

#include <functional> // std::function
#include <memory> // std::shared_ptr

BEGIN_VISP_NAMESPACE
/*!
  \class vpUnscentedKalman
  \ingroup group_core_kalman
  This class permits to use Unscented Kalman Filter (UKF) to tackle non-linear problems. Non-linearity
  can arise in the process function \f$ f: {R}^n \rightarrow {R}^n \f$, which makes evolve the internal
  state \f$ \textbf{x} \in {R}^n \f$ of the UKF over time, or in the measurement function \f$ h: {R}^n \rightarrow {R}^m \f$,
  which expresses the internal state of the UKF in the measurement space of dimension \f$ m \f$.

  We will briefly explain the principles of the UKF and the maths behind the wheel. We refer the interested
  readers to the [web-book](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python) by R. Labbe, chapter 10,
  for more details.

  The UKF is performed in two steps. First, the prediction step, during which we draw the sigma points \f$ \chi \f$ and compute
  their corresponding weights \f$ \textbf{w}^m \in {R}^{2n + 1} \f$ and \f$ \textbf{w}^c \in {R}^{2n + 1} \f$.
  Be \f$ \textbf{x} \in {R}^n \f$ the internal state of the UKF and \f$ \textbf{P} \in {R}^{n\text{ x }n} \f$ the process covariance matrix.
  We have:

  \f[
  \begin{array}{lcl}
      \chi &=& sigma-function(\textbf{x}, \textbf{P}) \\
      \textbf{w}^m, \textbf{w}^c &=& weight-function(n, parameters)
   \end{array}
  \f]

  There are different ways of drawing the sigma points and associated weights in the litterature, such as the one
  proposed by Julier or the one proposed by E. A. Wan and R. van der Merwe.

  Be \f$ \textbf{u} \f$ the vector containing the known commands sent to the system, if any. Then, we pass each sigma
  point through the process function \f$ f(\chi, \Delta t) \f$, the command function
  \f$ b( \textbf{u}, \Delta t ) \f$ and the command function depending on the state \f$ bx( \textbf{u}, \chi, \Delta t ) \f$
  to project them forward in time, forming the new prior:

  \f$ {Y} = f( \chi , \Delta t ) + b( \textbf{u}, \Delta t ) + bx( \textbf{u}, \chi, \Delta t )  \f$

  Then, we apply the Unscented Transform to compute the mean \f$ \boldsymbol{\mu} \f$
  and covariance \f$ \overline{\textbf{P}} \f$ of the prior:

  \f[
  \begin{array}{lcl}
      \boldsymbol{\mu},  \overline{\textbf{P}} &=& UT({Y}, \textbf{w}^m, \textbf{w}^c, \textbf{Q}) \\
      \boldsymbol{\mu} &=& \sum_{i=0}^{2n} w_i^m {Y}_i \\
      \overline{\textbf{P}} &=& \sum_{i=0}^{2n} ( w_i^c ({Y}_i - \boldsymbol{\mu}) ({Y}_i - \boldsymbol{\mu})^T ) + \textbf{Q}
  \end{array}
  \f]

  where \f$ \textbf{Q} \f$ is the covariance of the error introduced by the process function.

  The second step is the update step. It is performed in the measurement space, so we must convert the sigma points of
  the prior into measurements using the measurement function  \f$ h: {R}^n \rightarrow {R}^m \f$:

  \f$ {Z} = h({Y}) \f$

  Then, we use once again the Unscented Transform to compute the mean \f$ \boldsymbol{\mu}_z \in {R}^m \f$ and the
  covariance \f$ \textbf{P}_z \in {R}^{m\text{ x }m} \f$ of these points:

  \f[
  \begin{array}{lcl}
      \boldsymbol{\mu}_z,  \textbf{P}_z &=& UT({Z}, \textbf{w}^m, \textbf{w}^c, \textbf{R}) \\
      \boldsymbol{\mu}_z &=& \sum_{i=0}^{2n} w_i^m {Z}_i \\
      \textbf{P}_z &=& \sum_{i=0}^{2n} ( w_i^c ({Z}_i - \boldsymbol{\mu}_z) ({Z}_i - \boldsymbol{\mu}_z)^T ) + \textbf{R}
   \end{array}
  \f]

  where \f$ \textbf{R} \f$ is the measurement covariance matrix.

  Then, we compute the residual \f$ \textbf{y} \f$ of the measurement \f$ \textbf{z} \f$:

  \f$ \textbf{y} = \textbf{z} - \boldsymbol{\mu}_z \f$

  To compute the Kalman's gain, we first need to compute the cross covariance of the state and the measurements:

  \f$ \textbf{P}_{xy} = \sum_{i=0}^{2n} w_i^c ({Y}_i - \boldsymbol{\mu})({Z}_i - \boldsymbol{\mu}_z)^T \f$

  The Kalman's gain is then defined as:

  \f$ \textbf{K} = \textbf{P}_{xz} \textbf{P}_z^{-1} \f$

  Finally, we can compute the new state estimate \f$ \textbf{x} \f$ and the new covariance \f$ \textbf{P} \f$:

  \f[
  \begin{array}{lcl}
   \textbf{x} &=& \boldsymbol{\mu} + \textbf{K} \textbf{y} \\
   \textbf{P} &=& \overline{\textbf{P}} - \textbf{K} \textbf{P}_z \textbf{K}^T
  \end{array}
  \f]
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
   * \brief Function that computes either the equivalent of an addition or the equivalent
   * of a substraction in the state space or in the measurement space.
   * The first argument is the vector to which we must add/substract something
   * and the second argument is the thing to be added/substracted. The return is the
   * result of this operation.
   */
  typedef std::function<vpColVector(const vpColVector &, const vpColVector &)> vpAddSubFunction;

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
  vpUnscentedKalman(const vpMatrix &Q, const vpMatrix &R, std::shared_ptr<vpUKSigmaDrawerAbstract> &drawer, const vpProcessFunction &f, const vpMeasurementFunction &h);

  /**
   * \brief Set the guess of the initial state and covariance.
   *
   * \param[in] mu0 Guess of the initial state.
   * \param[in] P0 Guess of the initial covariance.
   */
  void init(const vpColVector &mu0, const vpMatrix &P0);

  /**
   * \brief Set the command function to use when computing the prior.
   *
   * \param b The command function to use.
   */
  inline void setCommandOnlyFunction(const vpCommandOnlyFunction &b)
  {
    m_b = b;
  }

  /**
   * \brief Set the command function to use when computing the prior.
   *
   * \param bx The command function to use.
   */
  inline void setCommandStateFunction(const vpCommandStateFunction &bx)
  {
    m_bx = bx;
  }

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
  inline void setMeasurementResidualFunction(const vpAddSubFunction &measResFunc)
  {
    m_measResFunc = measResFunc;
  }

  /**
   * \brief Set the state addition function to use when computing a addition
   * in the state space.
   *
   * \param stateAddFunc The addition function to use.
   */
  inline void setStateAddFunction(const vpAddSubFunction &stateAddFunc)
  {
    m_stateAddFunction = stateAddFunc;
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
  inline void setStateResidualFunction(const vpAddSubFunction &stateResFunc)
  {
    m_stateResFunc = stateResFunc;
  }

  /**
   * \brief Permit to change the covariance introduced at each prediction step.
   *
   * \param[in] Q The process covariance matrix.
   */
  inline void setProcessCovariance(const vpMatrix &Q)
  {
    m_Q = Q;
  }

  /**
   * \brief Permit to change the covariance introduced at each update step.
   *
   * \param[in] R The measurement covariance matrix.
   */
  inline void setMeasurementCovariance(const vpMatrix &R)
  {
    m_R = R;
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
   * \brief Get the estimated (i.e. filtered) covariance of the state.
   *
   * \return vpMatrix The filtered covariance matrix.
   */
  inline vpMatrix getPest() const
  {
    return m_Pest;
  }

  /**
   * \brief Get the predicted covariance of the state, i.e. the covariance of the prior.
   *
   * \return vpMatrix The predicted covariance matrix.
   */
  inline vpMatrix getPpred() const
  {
    return m_Ppred;
  }

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
   * \brief Simple function to compute a mean, which just does \f$ \boldsymbol{\mu} = \sum_{i} wm_i \textbf{vals}_i \f$
   *
   * \param[in] vals Vector containing all the vectors we must compute the mean.
   * \param[in] wm The correspond list of weights.
   * \return vpColVector \f$ \boldsymbol{\mu} = \sum_{i} wm_i \textbf{vals}_i \f$
   */
  inline static vpColVector simpleMean(const std::vector<vpColVector> &vals, const std::vector<double> &wm)
  {
    size_t nbPoints = vals.size();
    if (nbPoints == 0) {
      throw(vpException(vpException::dimensionError, "No points to add when computing the mean"));
    }
    vpColVector mean = vals[0] * wm[0];
    for (size_t i = 1; i < nbPoints; ++i) {
      mean += vals[i] * wm[i];
    }
    return mean;
  }
private:
  bool m_hasUpdateBeenCalled; /*!< Set to true when update is called, reset at the beginning of predict.*/
  vpColVector m_Xest; /*!< The estimated (i.e. filtered) state variables.*/
  vpMatrix m_Pest; /*!< The estimated (i.e. filtered) covariance matrix.*/
  vpMatrix m_Q; /*!< The covariance introduced by performing the prediction step.*/
  std::vector<vpColVector> m_chi; /*!< The sigma points.*/
  std::vector<double> m_wm; /*!< The weights for the mean computation.*/
  std::vector<double> m_wc; /*!< The weights for the covariance computation.*/
  std::vector<vpColVector> m_Y; /*!< The projection forward in time of the sigma points according to the process model, called the prior.*/
  vpColVector m_mu; /*!< The mean of the prior.*/
  vpMatrix m_Ppred; /*!< The covariance matrix of the prior.*/
  vpMatrix m_R; /*!< The covariance introduced by performing the update step.*/
  std::vector<vpColVector> m_Z; /*!< The sigma points of the prior expressed in the measurement space, called the measurement sigma points.*/
  vpColVector m_muz; /*!< The mean of the measurement sigma points.*/
  vpMatrix m_Pz; /*!< The covariance matrix of the measurement sigma points.*/
  vpMatrix m_Pxz; /*!< The cross variance of the state and the measurements.*/
  vpColVector m_y; /*!< The residual.*/
  vpMatrix m_K; /*!< The Kalman gain.*/
  vpProcessFunction m_f; /*!< Process model function, which projects the sigma points forward in time.*/
  vpMeasurementFunction m_h; /*!< Measurement function, which converts the sigma points in the measurement space.*/
  std::shared_ptr<vpUKSigmaDrawerAbstract> m_sigmaDrawer; /*!< Object that permits to draw the sigma points.*/
  vpCommandOnlyFunction m_b; /*!< Function that permits to compute the effect of the commands on the prior, without knowledge of the state.*/
  vpCommandStateFunction m_bx; /*!< Function that permits to compute the effect of the commands on the prior, with knowledge of the state.*/
  vpMeanFunction m_measMeanFunc; /*!< Function to compute a weighted mean in the measurement space.*/
  vpAddSubFunction m_measResFunc; /*!< Function to compute a substraction in the measurement space.*/
  vpAddSubFunction m_stateAddFunction; /*!< Function to compute an addition in the state space.*/
  vpMeanFunction m_stateMeanFunc; /*!< Function to compute a weighted mean in the state space.*/
  vpAddSubFunction m_stateResFunc; /*!< Function to compute a substraction in the state space.*/

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
    const std::vector<double> &wc, const vpMatrix &cov, const vpAddSubFunction &resFunc, const vpMeanFunction &meanFunc);
};
END_VISP_NAMESPACE
#endif
#endif
