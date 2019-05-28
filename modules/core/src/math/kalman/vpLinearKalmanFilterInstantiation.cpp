/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * See http://visp.inria.fr for more information.
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
 * Kalman filtering.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpLinearKalmanFilterInstantiation.cpp
  \brief Implementation of some specific Kalman filters.
*/

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpLinearKalmanFilterInstantiation.h>

#include <math.h>
#include <stdlib.h>

/*!

  Initialize the Kalman filter material depending on the selected
  state model set with setStateModel(). This function is provided as a
  wrapper over all the other initializer functions like
  initStateConstVel_MeasurePos(),
  initStateConstVelWithColoredNoise_MeasureVel(),
  initStateConstAccWithColoredNoise_MeasureVel().

  \warning It is requiered to set the state model before using this method.

  \param n_signal : Number of signal to filter.

  \param sigma_state : Vector that contains the variance of the state
  noise. The dimension of this vector is equal to the state vector
  \f${\bf x}\f$ size multiplied by the number of signal to
  filter. Values are used to initialize the \f${\bf Q}\f$ state
  covariance matrix.

  \param sigma_measure : Vector that contains the variance of the
  measurement noise. The dimension of this vector is equal to the
  measure vector \f${\bf x}\f$ size multiplied by the number of signal
  to filter. Values are used to initialize the \f${\bf R}\f$ measure
  covariance matrix.

  \param rho : Degree of correlation between successive accelerations. Values
  are in [0:1[.

  \param delta_t : Sampling time \f$\Delta t\f$ expressed is
  second. Depending on the filter modelization, this value may not be
  used. This is for example the case for the
  vpLinearKalmanFilterInstantiation::stateConstVelWithColoredNoise_MeasureVel
model implemented in initStateConstVelWithColoredNoise_MeasureVel().

  \exception vpException::badValue : Bad rho value wich is not in [0:1[.

  \exception vpException::notInitialized : If the state model is not
  initialized. To initialize it you need to call setStateModel().

  The example below shows how to initialize the filter for a one
  dimensional signal.

  \code
#include <visp3/core/vpLinearKalmanFilterInstantiation.h>

int main()
{
  vpLinearKalmanFilterInstantiation kalman;
  // Select a constant velocity state model with colored noise
  // Measures are velocities
  kalman.setStateModel(vpLinearKalmanFilterInstantiation::stateConstVelWithColoredNoise_MeasureVel);

  // Initialise the filter for a one dimension signal
  int signal = 1;
  vpColVector sigma_state(2);   // State vector size is 2
  vpColVector sigma_measure(1); // Measure vector size is 1
  double rho = 0.9;
  double dummy = 0; // non used parameter for the selected state model

  kalman.initFilter(signal, sigma_state, sigma_measure, rho, dummy);
}
  \endcode

  The example below shows a more complete example to filter a two
  dimensional target trajectory with an estimation of the target
  velocities from velocity measures.

  \code
#include <visp3/core/vpLinearKalmanFilterInstantiation.h>

int main()
{
  vpLinearKalmanFilterInstantiation kalman;

  // Set the constant velocity state model used for the filtering
  vpLinearKalmanFilterInstantiation::vpStateModel model;
  model = vpLinearKalmanFilterInstantiation::stateConstVelWithColoredNoise_MeasureVel;
  kalman.setStateModel(model);

  // We are now able to retrieve the size of the state vector and measure vector
  int state_size = kalman.getStateSize();

  // Filter the x and y velocities of a target (2 signals are to consider)
  int nsignal = 2;

  // Initialize the filter parameters:
  // - Firstly, the state variance
  int size = state_size*nsignal;
  vpColVector sigma_state(size);
  sigma_state[1] = 0.001; // Variance on the acceleration for the 1st signal (x)
  sigma_state[3] = 0.002; // Variance on the acceleration for the 2nd signal (y)
  // - Secondly, the measures variance
  vpColVector sigma_measure(nsignal); // 2 velocity measures available
  sigma_measure[0] = 0.03; // Variance on the x velocity measure
  sigma_measure[1] = 0.06; // Variance on the y velocity measure
  // - Thirdly, the correlation between succesive accelerations
  double rho = 0.9;

  double dummy = 0; // non used parameter for the selected state model

  // Initialize the filter
  // The state model was set before
  kalman.initFilter(nsignal, sigma_state, sigma_measure, rho, dummy);

  // Does the filtering
  vpColVector vm(2); // Measured velocities
  for ( ; ; ) {
    // Get the two dimentional velocity measures
    // vm[0] = ...;
    // vm[1] = ...;

    // Compute the filtering and the prediction
    kalman.filter(vm);
    // Print the estimation of the velocities (1st value of the state vector)
    std::cout << "Estimated x velocity: kalman.Xest[0]" << std::endl;
    std::cout << "Estimated y velocity: kalman.Xest[kalman.getStateSize()]"
              << std::endl;
    // The one step prediction is available in kalman.Xpre variable
  }
}
  \endcode
 */
void vpLinearKalmanFilterInstantiation::initFilter(unsigned int n_signal, vpColVector &sigma_state,
                                                   vpColVector &sigma_measure, double rho, double delta_t)
{
  switch (model) {
  case stateConstVelWithColoredNoise_MeasureVel:
    initStateConstVelWithColoredNoise_MeasureVel(n_signal, sigma_state, sigma_measure, rho);
    break;
  case stateConstVel_MeasurePos:
    initStateConstVel_MeasurePos(n_signal, sigma_state, sigma_measure, delta_t);
    break;
  case stateConstAccWithColoredNoise_MeasureVel:
    initStateConstAccWithColoredNoise_MeasureVel(n_signal, sigma_state, sigma_measure, rho, delta_t);
    break;
  case unknown:
    vpERROR_TRACE("Kalman state model is not set");
    throw(vpException(vpException::notInitialized, "Kalman state model is not set"));
  }
}

/*!
  Modelisation of a constant speed state model with white noise. The
  measure is assumed to be the position of the target.

  The considered state model is the following

  \f[
  \left\{
  \begin{array}{rlrl}
  x_{(k+1)}  & = x_{(k)}   & + \Delta t \; {\dot{x}}_{(k)} & + {w_1}_{(k)} \\
  {\dot{x}}_{(k+1)} & =  & {\dot{x}}_{(k)}   &+{w_2}_{(k)}
  \end{array}
  \right.
  \f]

  The terms \f${w_1}_{(k)}\f$ and \f${w_2}_{(k)}\f$ account for deviations
  from the assumed constant velocity trajectory. They are assumed zero-mean,
  white, mutually uncorrelated, stationary random variable with variance
  \f$\sigma^2_{Q_1}\f$ and \f$\sigma^2_{Q_2}\f$.

  We recall that the recursive state evolution equation is given by
  \f[
  {\bf x}_k= {\bf F}_{k-1} {\bf x}_{k-1} + {\bf w}_{k-1} \\
  \f]

  From this state model, the transition matrix \f${\bf F}\f$ and the
  state covariance matrix \f${\bf Q}\f$ are given by:

  \f[
  {\bf F} =
  \left[
  \begin{array}{cc}
  1 & \Delta t\\
  0 & 1
  \end{array}
  \right]
  \f]

  and

  \f[
  {\bf Q} = \sigma^2_Q
  \left[
  \begin{array}{cc}
  \frac{1}{3}\Delta t^3 & \frac{1}{2}\Delta t^2\\
  \frac{1}{2}\Delta t^2 & \Delta t
  \end{array}
  \right]
  \f]

  The initial value of the state vector at iteration 0 is set to:
  \f[
  {\bf x_{(0)}} =
  \left[
  \begin{array}{c}
  z_{(0)}\\
  0
  \end{array}
  \right]
  \f]

  The value at iteration 1 is set to:
  \f[
  {\bf x_{(1)}} =
  \left[
  \begin{array}{c}
  z_{(1)}\\
  (z_{(1)} - z_{(0)})/ \Delta t
  \end{array}
  \right]
  \f]

  The initial value \f$P_{(0|0)}\f$ of the prediction covariance
  matrix is given by:

  \f[
  {\bf P_{(0|0)}} =
  \left[ \begin{array}{cc}
  \sigma^2_R & \frac{\sigma^2_R}{2 \Delta t}\\
  \frac{\sigma^2_R}{2 \Delta t} & \frac{2}{3}\sigma^2_Q \Delta t +
  \frac{\sigma^2_R}{2 \Delta t^2} \end{array} \right] \f]

  \param n_signal : Number of signal to filter.

  \param sigma_state : Vector that fix the variance of the state covariance
  matrix \f$[\sigma^2_Q \; 0]^T\f$. The dimension of this vector is 2
  multiplied by the number of signal to filter.

  \param sigma_measure : Variance \f$\sigma^2_R\f$ of the measurement
  noise. The dimension of this vector is equal to the number of signal
  to filter.

  \param delta_t : Sampling time \f$\Delta t\f$ expressed is second.

*/
void vpLinearKalmanFilterInstantiation::initStateConstVel_MeasurePos(unsigned int n_signal, vpColVector &sigma_state,
                                                                     vpColVector &sigma_measure, double delta_t)
{
  // init_done = true ;
  setStateModel(stateConstVel_MeasurePos);

  init(size_state, size_measure, n_signal);

  iter = 0;
  Pest = 0;
  Xest = 0;
  F = 0;
  H = 0;
  R = 0;
  Q = 0;
  this->dt = delta_t;

  double dt2 = dt * dt;
  double dt3 = dt2 * dt;

  for (unsigned int i = 0; i < size_measure * n_signal; i++) {
    // State model
    //         | 1  dt |
    //     F = |       |
    //         | 0   1 |

    F[2 * i][2 * i] = 1;
    F[2 * i][2 * i + 1] = dt;
    F[2 * i + 1][2 * i + 1] = 1;

    // Measure model
    H[i][2 * i] = 1;
    H[i][2 * i + 1] = 0;

    double sR = sigma_measure[i];
    double sQ = sigma_state[2 * i]; // sigma_state[2*i+1] is not used

    // Measure noise
    R[i][i] = sR;

    // State covariance matrix 6.2.2.12
    Q[2 * i][2 * i] = sQ * dt3 / 3;
    Q[2 * i][2 * i + 1] = sQ * dt2 / 2;
    Q[2 * i + 1][2 * i] = sQ * dt2 / 2;
    Q[2 * i + 1][2 * i + 1] = sQ * dt;

    Pest[2 * i][2 * i] = sR;
    Pest[2 * i][2 * i + 1] = sR / (2 * dt);
    Pest[2 * i + 1][2 * i] = sR / (2 * dt);
    Pest[2 * i + 1][2 * i + 1] = sQ * 2 * dt / 3.0 + sR / (2 * dt2);
  }
}

/*!

  Modelisation of a constant speed state model with colored noise. The
  measure is assumed to be the velocity of the target.

  This state model assume that there is some memory associated with
  noise measurements as acceleration terms. They can be represented as
  remaining correlated (or colored) over succesive time intervals,
  leading to the following state model:

  \f[
  \left\{
  \begin{array}{rll}
  x_{(k+1)}  & = x_{(k)} + \nu_{(k)} &\\
  \nu_{(k+1)}& = \rho \nu_{(k)}      &+w_{(k)}
  \end{array}
  \right.
  \f]

  The term \f$w_{(k)}\f$ account for deviations from the assumed
  constant velocity trajectory. It is assumed zero-mean, white,
  mutually uncorrelated, stationary random variable with variance
  \f$\sigma^2_Q\f$. The term \f$\rho\f$ is the degree of correlation
  between successive accelerations. Values can range from 0 to 1.

  We recall that the recursive state evolution equation is given by
  \f[
  {\bf x}_k= {\bf F}_{k-1} {\bf x}_{k-1} + {\bf w}_{k-1} \\
  \f]

  From this state model, the transition matrix \f${\bf F}\f$ and the
  state covariance matrix \f${\bf Q}\f$ are given by:

  \f[
  {\bf F} =
  \left[
  \begin{array}{cc}
  1 & 1\\
  0 & \rho
  \end{array}
  \right]
  \f]

  and

  \f[
  {\bf Q} =
  \left[
  \begin{array}{cc}
  0 & 0\\
  0 & \sigma^2_Q
  \end{array}
  \right]
  \f]

  The measurement model is given by:
  \f[
  z_{(k)} = {\bf H} {\bf x}_{(k)} + r_{(k)}
  \f]

  where \f${\bf H} = [1 \; 0  \; 0]\f$, \f$z_{(k)}\f$ is the measure of the
  velocity and \f$r_{(k)}\f$ is the measurement noise, assumed
  zero-mean, white mutually uncorrelated stationary random variables
  with variance \f$\sigma^2_R\f$, giving the covariance matrix:

  \f[
  {\bf R} = \left[\sigma^2_R\right]
  \f]

  The initial value of the state vector is set to:
  \f[
  {\bf x_{(0)}} =
  \left[
  \begin{array}{c}
  z_{(0)}\\
  0
  \end{array}
  \right]
  \f]

  The initial value \f$P_{(0|0)}\f$ of the prediction covariance
  matrix is given by:

  \f[
  {\bf P_{(0|0)}} =
  \left[ \begin{array}{cc}
  \sigma^2_R & 0\\
  0 & \sigma^2_Q/(1-\rho^2)
  \end{array}
  \right]
  \f]

  \param n_signal : Number of signal to filter.

  \param sigma_state : Vector that fix the variance of the state covariance
matrix \f$[0 \; \sigma^2_Q]^T\f$. The dimension of this vector is 2 multiplied
by the number of signal to filter.

  \param sigma_measure : Variance \f$\sigma^2_R\f$ of the measurement
  noise. The dimension of this vector is equal to the number of signal
  to filter.

  \param rho : Degree of correlation between successive accelerations. Values
  are in [0:1[.

  \exception vpException::badValue : Bad rho value wich is not in [0:1[.

  The example below shows how to filter a two dimensional target
  trajectory with an estimation of the target velocity from velocity
  measures.

  \code
#include <visp3/core/vpLinearKalmanFilterInstantiation.h>

int main()
{
  vpLinearKalmanFilterInstantiation kalman;
  // Filter the x and y velocities of a target (2 signals are to consider)
  int nsignal = 2;

  // Initialize the filter parameters:
  // - Firstly, the state variance
  vpColVector sigma_state(4); // 4 = 2 for the state size x 2 signal
  sigma_state[1] = 0.001; // Variance on the acceleration for the 1st signal (x)
  sigma_state[3] = 0.002; // Variance on the acceleration for the 2nd signal (y)
  // - Secondly, the measures variance
  vpColVector sigma_measure(nsignal); // 2 velocity measures available
  sigma_measure[0] = 0.03; // Variance on the x velocity measure
  sigma_measure[1] = 0.06; // Variance on the y velocity measure
  // - Thirdly, the correlation between succesive accelerations
  double rho = 0.9;

  // Initialize the filter
  kalman.initStateConstVelWithColoredNoise_MeasureVel(nsignal, sigma_state, sigma_measure, rho);

  // Does the filtering
  vpColVector vm(2); // Measured velocities
  for ( ; ; ) {
    // Get the two dimentional velocity measures
    // vm[0] = ...;
    // vm[1] = ...;

    // Compute the filtering and the prediction
    kalman.filter(vm);
    // Print the estimation of the velocities (1st value of the state vector)
    std::cout << "Estimated x velocity: kalman.Xest[0]" << std::endl;
    std::cout << "Estimated y velocity: kalman.Xest[kalman.getStateSize()]"
              << std::endl;
    // The one step prediction is available in kalman.Xpre variable
  }
}
  \endcode
*/
void vpLinearKalmanFilterInstantiation::initStateConstVelWithColoredNoise_MeasureVel(unsigned int n_signal,
                                                                                     vpColVector &sigma_state,
                                                                                     vpColVector &sigma_measure,
                                                                                     double rho)
{
  if ((rho < 0) || (rho >= 1)) {
    vpERROR_TRACE("Bad rho value %g; should be in [0:1[", rho);
    throw(vpException(vpException::badValue, "Bad rho value"));
  }

  setStateModel(stateConstVelWithColoredNoise_MeasureVel);

  init(size_state, size_measure, n_signal);

  iter = 0;
  Pest = 0;
  Xest = 0;
  F = 0;
  H = 0;
  R = 0;
  Q = 0;

  for (unsigned int i = 0; i < size_measure * n_signal; i++) {
    // State model
    //         | 1    1  |
    //     F = |         |
    //         | 0   rho |

    F[2 * i][2 * i] = 1;
    F[2 * i][2 * i + 1] = 1;
    F[2 * i + 1][2 * i + 1] = rho;

    // Measure model
    H[i][2 * i] = 1;
    H[i][2 * i + 1] = 0;

    double sR = sigma_measure[i];
    double sQ = sigma_state[2 * i + 1]; // sigma_state[2*i] is not used

    // Measure noise
    R[i][i] = sR;

    // State covariance matrix
    Q[2 * i][2 * i] = 0;
    Q[2 * i][2 * i + 1] = 0;
    Q[2 * i + 1][2 * i] = 0;
    Q[2 * i + 1][2 * i + 1] = sQ;

    Pest[2 * i][2 * i] = sR;
    Pest[2 * i][2 * i + 1] = 0.;
    Pest[2 * i + 1][2 * i] = 0;
    Pest[2 * i + 1][2 * i + 1] = sQ / (1 - rho * rho);
  }
}

/*!

  Modelisation of a constant acceleration state model with colored noise. The
  measure is assumed to be the velocity of the target.

  This state model assume that there is some memory associated with
  noise measurements as acceleration terms. They can be represented as
  remaining correlated (or colored) over succesive time intervals,
  leading to the following state model:

  \f[
  \left\{
  \begin{array}{rll}
  x_{(k+1)}  & = x_{(k)} + \Delta t \; \dot{x}_{(k)} + \nu_{(k)} &\\
  \nu_{(k+1)}& = \rho \nu_{(k)}      &+w_{1(k)} \\
  \dot{x}_{(k+1)}  & = \dot{x}_{(k)} &+w_{2(k)}\\
  \end{array}
  \right.
  \f]

  The terms \f$w_{1(k)}\f$ and \f$w_{2(k)}\f$ account for deviations
  from the assumed constant acceleration trajectory. They are assumed
  zero-mean, white, mutually uncorrelated, stationary random variable
  with variance \f$\sigma^2_{Q_1}\f$ and \f$\sigma^2_{Q_2}\f$. The term
  \f$\rho\f$ is the degree of correlation between successive
  accelerations. Values can range from 0 to 1.

  We recall that the recursive state evolution equation is given by
  \f[
  {\bf x}_k= {\bf F}_{k-1} {\bf x}_{k-1} + {\bf w}_{k-1} \\
  \f]

  From this state model, the transition matrix \f${\bf F}\f$ and the
  state covariance matrix \f${\bf Q}\f$ are given by:

  \f[
  {\bf F} =
  \left[
  \begin{array}{ccc}
  1 & 1 & \Delta t\\
  0 & \rho & 0 \\
  0 & 0 & 1
  \end{array}
  \right]
  \f]

  and

  \f[
  {\bf Q} =
  \left[
  \begin{array}{ccc}
  0 & 0 & 0\\
  0 & \sigma^2_{Q_1} & 0\\
  0 & 0& \sigma^2_{Q_2} \\
  \end{array}
  \right]
  \f]

  The measurement model is given by:
  \f[
  z_{(k)} = {\bf H} {\bf x}_{(k)} + r_{(k)}
  \f]

  where \f${\bf H} = [1  \; 0  \; 0]\f$, \f$z_{(k)}\f$ is the measure of the
velocity and \f$r_{(k)}\f$ is the measurement noise, assumed zero-mean, white
mutually uncorrelated stationary random variables with variance
  \f$\sigma^2_R\f$, giving the covariance matrix:

  \f[
  {\bf R} = \left[\sigma^2_R\right]
  \f]

  The initial value of the state vector is set to:
  \f[
  {\bf x_{(0)}} =
  \left[
  \begin{array}{c}
  z_{(0)}\\
  0 \\
  0
  \end{array}
  \right]
  \f]

  The initial value \f$P_{(0|0)}\f$ of the prediction covariance
  matrix is given by:

  \f[
  {\bf P_{(0|0)}} =
  \left[ \begin{array}{ccc}
  \sigma^2_R & 0 & \sigma^2_R / \Delta t\\
  0 & \sigma^2_{Q_1}/(1-\rho^2) & -\rho \sigma^2_{Q_1} / (1-\rho^2)\Delta t \\
  \sigma^2_R / \Delta t &  -\rho \sigma^2_{Q_1} / (1-\rho^2)\Delta t &
(2\sigma^2_R +\sigma^2_{Q_1}/(1-\rho^2) )/\Delta t^2 \end{array} \right] \f]

  \param n_signal : Number of signal to filter.

  \param sigma_state : Vector that fix the variance of the state
  covariance matrix \f$[0 \; \sigma^2_{Q_1} \;
  \sigma^2_{Q_2}]^T\f$. The dimension of this vector is 3 multiplied
  by the number of signal to filter.

  \param sigma_measure : Variance \f$\sigma^2_R\f$ of the measurement
  noise. The dimension of this vector is equal to the number of signal
  to filter.

  \param rho : Degree of correlation between successive accelerations. Values
  are in [0:1[.

  \param delta_t : Sampling time \f$\Delta t\f$ expressed is second.

  \exception vpException::badValue : Bad rho value wich is not in [0:1[.

  The example below shows how to filter a two dimensional target
  trajectory with an estimation of the target velocity from velocity
  measures.

  \code
#include <visp3/core/vpLinearKalmanFilterInstantiation.h>

int main()
{
  vpLinearKalmanFilterInstantiation kalman;
  // Filter the x and y velocities of a target (2 signals are to consider)
  int nsignal = 2;

  // Initialize the filter parameters:
  // - Firstly, the state variance
  vpColVector sigma_state(6); // 6 = 3 for the state size x 2 signal
  sigma_state[1] = 0.001; // Variance on the acceleration for the 1st signal (x)
  sigma_state[2] = 0.001; // Variance on the acceleration for the 1st signal (x)
  sigma_state[4] = 0.002; // Variance on the acceleration for the 2nd signal (y)
  sigma_state[5] = 0.002; // Variance on the acceleration for the 2nd signal (y)
  // - Secondly, the measures variance
  vpColVector sigma_measure(nsignal); // 2 velocity measures available
  sigma_measure[0] = 0.03; // Variance on the x velocity measure
  sigma_measure[1] = 0.06; // Variance on the y velocity measure
  // - Thirdly, the correlation between succesive accelerations
  double rho = 0.9;
  // - Lastly, the sampling time
  double dt = 0.020; // 20 ms

  // Initialize the filter
  kalman.initStateConstAccWithColoredNoise_MeasureVel(nsignal, sigma_state, sigma_measure, rho, dt);

  // Does the filtering
  vpColVector vm(2); // Measured velocities
  for ( ; ; ) {
    // Get the two dimentional velocity measures
    // vm[0] = ...;
    // vm[1] = ...;

    // Compute the filtering and the prediction
    kalman.filter(vm);
    // Print the estimation of the velocities (1st value of the state vector)
    std::cout << "Estimated x velocity: kalman.Xest[0]" << std::endl;
    std::cout << "Estimated y velocity: kalman.Xest[kalman.getStateSize()]"
              << std::endl;
    // The one step prediction is available in kalman.Xpre variable
  }
}
  \endcode

*/
void vpLinearKalmanFilterInstantiation::initStateConstAccWithColoredNoise_MeasureVel(unsigned int n_signal,
                                                                                     vpColVector &sigma_state,
                                                                                     vpColVector &sigma_measure,
                                                                                     double rho, double delta_t)
{
  if ((rho < 0) || (rho >= 1)) {
    vpERROR_TRACE("Bad rho value %g; should be in [0:1[", rho);
    throw(vpException(vpException::badValue, "Bad rho value"));
  }
  setStateModel(stateConstAccWithColoredNoise_MeasureVel);

  init(size_state, size_measure, n_signal);

  iter = 0;
  Pest = 0;
  Xest = 0;
  F = 0;
  H = 0;
  R = 0;
  Q = 0;
  this->dt = delta_t;
  // initialise les matrices decrivant les modeles
  for (unsigned int i = 0; i < size_measure * nsignal; i++) {
    // State model
    //         | 1    1   dt |
    //     F = | o   rho   0 |
    //         | 0    0    1 |

    F[3 * i][3 * i] = 1;
    F[3 * i][3 * i + 1] = 1;
    F[3 * i][3 * i + 2] = dt;
    F[3 * i + 1][3 * i + 1] = rho;
    F[3 * i + 2][3 * i + 2] = 1;

    // Measure model
    H[i][3 * i] = 1;
    H[i][3 * i + 1] = 0;
    H[i][3 * i + 2] = 0;

    double sR = sigma_measure[i];
    double sQ1 = sigma_state[3 * i + 1];
    double sQ2 = sigma_state[3 * i + 2];

    // Measure noise
    R[i][i] = sR;

    // State covariance matrix
    Q[3 * i + 1][3 * i + 1] = sQ1;
    Q[3 * i + 2][3 * i + 2] = sQ2;

    Pest[3 * i][3 * i] = sR;
    Pest[3 * i][3 * i + 1] = 0.;
    Pest[3 * i][3 * i + 2] = sR / dt;
    Pest[3 * i + 1][3 * i + 1] = sQ1 / (1 - rho * rho);
    Pest[3 * i + 1][3 * i + 2] = -rho * sQ1 / ((1 - rho * rho) * dt);
    Pest[3 * i + 2][3 * i + 2] = (2 * sR + sQ1 / (1 - rho * rho)) / (dt * dt);
    // complete the lower triangle
    Pest[3 * i + 1][3 * i] = Pest[3 * i][3 * i + 1];
    Pest[3 * i + 2][3 * i] = Pest[3 * i][3 * i + 2];
    Pest[3 * i + 2][3 * i + 1] = Pest[3 * i + 1][3 * i + 2];
  }
}

/*!

  Do the filtering and prediction of the measure signal.

  \param z : Measures \f${\bf z}_k\f$ used to initialise the filter. The
  dimension of this vector is equal to the number of signal to filter (given
  by getNumberOfSignal()) multiplied by the size of the measure vector (given
  by getMeasureSize()) .

  \exception vpException::notInitialized : If the filter is not
  initialized. To initialize the filter see initFilter().

*/
void vpLinearKalmanFilterInstantiation::filter(vpColVector &z)
{
  if (nsignal < 1) {
    vpERROR_TRACE("Bad signal number. You need to initialize the Kalman filter");
    throw(vpException(vpException::notInitialized, "Bad signal number"));
  }

  // Specific initialization of the filter that depends on the state model
  if (iter == 0) {
    Xest = 0;
    switch (model) {
    case stateConstVel_MeasurePos:
    case stateConstVelWithColoredNoise_MeasureVel:
    case stateConstAccWithColoredNoise_MeasureVel:
      for (unsigned int i = 0; i < size_measure * nsignal; i++) {
        Xest[size_state * i] = z[i];
      }
      prediction();
      //      init_done = true;
      break;
    case unknown:
      vpERROR_TRACE("Kalman state model is not set");
      throw(vpException(vpException::notInitialized, "Kalman state model is not set"));
      break;
    }
    iter++;

    return;
  } else if (iter == 1) {
    if (model == stateConstVel_MeasurePos) {
      for (unsigned int i = 0; i < size_measure * nsignal; i++) {
        double z_prev = Xest[size_state * i]; // Previous mesured position
        //	std::cout << "Mesure pre: " << z_prev << std::endl;
        Xest[size_state * i] = z[i];
        Xest[size_state * i + 1] = (z[i] - z_prev) / dt;
      }
      prediction();
      iter++;

      return;
    }
  }

  filtering(z);
  prediction();
}
