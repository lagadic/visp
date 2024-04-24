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

//! \example tutorial-ukf-nonlinear-complex-example.cpp

// UKF includes
#include <visp3/core/vpUKSigmaDrawerMerwe.h>
#include <visp3/core/vpUnscentedKalman.h>

// ViSP includes
#include <visp3/core/vpGaussRand.h>
#include <visp3/gui/vpPlot.h>

namespace
{
/**
 * \brief Normalize the \b angle in the interval [-Pi; Pi].
 *
 * \param[in] angle Angle to normalize.
 * \return double Normalized angle.
 */
double normalizeAngle(const double &angle)
{
  double angleIn0to2pi = vpMath::modulo(angle, 2. * M_PI);
  double angleInMinPiPi = angleIn0to2pi;
  if (angleInMinPiPi > M_PI) {
    // Substract 2 PI to be in interval [-Pi; Pi]
    angleInMinPiPi -= 2. * M_PI;
  }
  return angleInMinPiPi;
}

/**
 * \brief Compute the weighted mean of measurement vectors.
 *
 * \param[in] measurements The measurement vectors.
 * \param[in] wm The associated weights.
 * \return vpColVector
 */
vpColVector measurementMean(const std::vector<vpColVector> &measurements, const std::vector<double> &wm)
{
  vpColVector mean(measurements[0].size(), 0.);
  unsigned int nbPoints = measurements.size();
  unsigned int sizeMeasurement = measurements[0].size();
  unsigned int nbLandmarks = sizeMeasurement / 2;
  std::vector<double> sumCos(nbLandmarks, 0.);
  std::vector<double> sumSin(nbLandmarks, 0.);
  for (unsigned int i = 0; i < nbPoints; ++i) {
    for (unsigned int j = 0; j < nbLandmarks; ++j) {
      mean[2*j] += wm[i] * measurements[i][2*j];
      sumCos[j] += wm[i] * std::cos(measurements[i][2*j+1]);
      sumSin[j] += wm[i] * std::sin(measurements[i][2*j+1]);
    }
  }
  for (unsigned int j = 0; j < nbLandmarks; ++j) {
    mean[2*j+1] = std::atan2(sumSin[j], sumCos[j]);
  }
  return mean;
}

/**
 * \brief Compute the substraction between two vectors expressed in the measurement space,
 * such as v[0] = dist_0 ; v[1] = bearing_0; v[2] = dist_1 ; v[3] = bearing_1 ...
 *
 * \param[in] meas Measurement to which we must substract something.
 * \param[in] toSubstract The something we must substract.
 * \return vpColVector \b meas - \b toSubstract .
 */
vpColVector measurementResidual(const vpColVector &meas, const vpColVector &toSubstract)
{
  vpColVector res = meas - toSubstract;
  unsigned int nbMeasures = res.size();
  for (unsigned int i = 1; i < nbMeasures; i += 2) {
    res[i] = normalizeAngle(res[i]);
  }
  return res;
}

/**
 * \brief Compute the addition between two vectors expressed in the state space,
 * such as v[0] = x ; v[1] = y; v[2] = heading .
 *
 * \param[in] state State to which we must add something.
 * \param[in] toAdd The something we must add.
 * \return vpColVector \b state + \b toAdd .
 */
vpColVector stateAdd(const vpColVector &state, const vpColVector &toAdd)
{
  vpColVector add = state + toAdd;
  add[2] = normalizeAngle(add[2]);
  return add;
}

/**
 * \brief Compute the weighted mean of state vectors.
 *
 * \param[in] states The state vectors.
 * \param[in] wm The associated weights.
 * \return vpColVector
 */
vpColVector stateMean(const std::vector<vpColVector> &states, const std::vector<double> &wm)
{
  vpColVector mean(3, 0.);
  unsigned int nbPoints = states.size();
  double sumCos = 0.;
  double sumSin = 0.;
  for (unsigned int i = 0; i < nbPoints; ++i) {
    mean[0] += wm[i] * states[i][0];
    mean[1] += wm[i] * states[i][1];
    sumCos += wm[i] * std::cos(states[i][2]);
    sumSin += wm[i] * std::sin(states[i][2]);
  }
  mean[2] = std::atan2(sumSin, sumCos);
  return mean;
}

/**
 * \brief Compute the substraction between two vectors expressed in the state space,
 * such as v[0] = x ; v[1] = y; v[2] = heading .
 *
 * \param[in] state State to which we must substract something.
 * \param[in] toSubstract The something we must substract.
 * \return vpColVector \b state - \b toSubstract .
 */
vpColVector stateResidual(const vpColVector &state, const vpColVector &toSubstract)
{
  vpColVector res = state - toSubstract;
  res[2] = normalizeAngle(res[2]);
  return res;
}

/**
 * \brief As the state model {x, y, \f$ \theta \f$} does not contain any velocity
 * information, it does not evolve without commands.
 *
 * \param[in] x The state vector
 * \return vpColVector The state vector unchanged.
 */
vpColVector fx(const vpColVector &x, const double & /*dt*/)
{
  return x;
}
}

/**
 * \brief Class that approximates a 4-wheel robot using a bicycle model.
 */
class vpBicycleModel
{
public:
  /**
   * \brief Construct a new vpBicycleModel object.
   *
   * \param[in] w The length of the wheelbase.
   */
  vpBicycleModel(const double &w)
    : m_w(w)
  { }

  /**
   * \brief Models the effect of the command on the state model.
   *
   * \param[in] u The commands. u[0] = velocity ; u[1] = steeringAngle .
   * \param[in] x The state model. x[0] = x ; x[1] = y ; x[2] = heading
   * \param[in] dt The period.
   * \return vpColVector The state model after applying the command.
   */
  vpColVector computeMotion(const vpColVector &u, const vpColVector &x, const double &dt)
  {
    double heading = x[2];
    double vel = u[0];
    double steeringAngle = u[1];
    double distance = vel * dt;

    if (std::abs(steeringAngle) > 0.001) {
      // The robot is turning
      double beta = (distance / m_w) * std::tan(steeringAngle);
      double radius = m_w / std::tan(steeringAngle);
      double sinh = std::sin(heading);
      double sinhb = std::sin(heading + beta);
      double cosh = std::cos(heading);
      double coshb = std::cos(heading + beta);
      vpColVector motion(3);
      motion[0] = -radius * sinh + radius * sinhb;
      motion[1] = radius * cosh - radius * coshb;
      motion[2] = beta;
      return motion;
    }
    else {
      // The robot is moving in straight line
      vpColVector motion(3);
      motion[0] = distance * std::cos(heading);
      motion[1] = distance * std::sin(heading);
      motion[2] = 0.;
      return motion;
    }
  }

  /**
   * \brief Models the effect of the command on the state model.
   *
   * \param[in] u The commands. u[0] = velocity ; u[1] = steeringAngle .
   * \param[in] x The state model. x[0] = x ; x[1] = y ; x[2] = heading
   * \param[in] dt The period.
   * \return vpColVector The state model after applying the command.
   */
  vpColVector move(const vpColVector &u, const vpColVector &x, const double &dt)
  {
    vpColVector motion = computeMotion(u, x, dt);
    vpColVector newX = x + motion;
    newX[2] = normalizeAngle(newX[2]);
    return newX;
  }
private:
  double m_w; // The length of the wheelbase.
};

/**
 * \brief Class that permits to convert the position + heading of the 4-wheel
 * robot into measurements from a landmark.
 */
class vpLandmarkMeasurements
{
public:
  /**
   * \brief Construct a new vpLandmarkMeasurements object.
   *
   * \param[in] x The position along the x-axis of the landmark.
   * \param[in] y The position along the y-axis of the landmark.
   * \param[in] range_std The standard deviation of the range measurements.
   * \param[in] elev_angle_std The standard deviation of the relative angle measurements.
   */
  vpLandmarkMeasurements(const double &x, const double &y, const double &range_std, const double &rel_angle_std)
    : m_x(x)
    , m_y(y)
    , m_rngRange(range_std, 0., vpTime::measureTimeMicros())
    , m_rngRelativeAngle(rel_angle_std, 0., vpTime::measureTimeMicros() + 4221)
  { }

  /**
   * \brief Convert the prior of the UKF into the measurement space.
   *
   * \param[in] chi The prior.
   * \return vpColVector The prior expressed in the measurement space.
   */
  vpColVector state_to_measurement(const vpColVector &chi)
  {
    vpColVector meas(2);
    double dx = m_x - chi[0];
    double dy = m_y - chi[1];
    meas[0] = std::sqrt(dx * dx + dy * dy);
    meas[1] = normalizeAngle(std::atan2(dy, dx) - chi[2]);
    return meas;
  }

  /**
   * \brief Perfect measurement of the range and relative orientation of the robot
   * located at pos.
   *
   * \param[in] pos The actual position of the robot (pos[0]: x, pos[1]: y, pos[2] = heading.
   * \return vpColVector [0] the range [1] the relative orientation of the robot.
   */
  vpColVector measureGT(const vpColVector &pos)
  {
    double dx = m_x - pos[0];
    double dy = m_y - pos[1];
    double range = std::sqrt(dx * dx + dy * dy);
    double orientation = normalizeAngle(std::atan2(dy, dx) - pos[2]);
    vpColVector measurements(2);
    measurements[0] = range;
    measurements[1] = orientation;
    return measurements;
  }

  /**
   * \brief Noisy measurement of the range and relative orientation that
   * correspond to pos.
   *
   * \param[in] pos The actual position of the robot (pos[0]: x ; pos[1] = y ; pos[2] = heading).
   * \return vpColVector [0] the range [1] the relative orientation.
   */
  vpColVector measureWithNoise(const vpColVector &pos)
  {
    vpColVector measurementsGT = measureGT(pos);
    vpColVector measurementsNoisy = measurementsGT;
    measurementsNoisy[0] += m_rngRange();
    measurementsNoisy[1] += m_rngRelativeAngle();
    measurementsNoisy[1] = normalizeAngle(measurementsNoisy[1]);
    return measurementsNoisy;
  }

private:
  double m_x; // The position along the x-axis of the landmark
  double m_y; // The position along the y-axis of the landmark
  vpGaussRand m_rngRange; // Noise simulator for the range measurement
  vpGaussRand m_rngRelativeAngle; // Noise simulator for the relative angle measurement
};

/**
 * \brief Class that represent a grid of landmarks that measure the distance and
 * relative orientation of the 4-wheel robot.
 */
class vpLandmarksGrid
{
public:
  /**
  * \brief Construct a new vpLandmarksGrid object.
  *
  * @param landmarks The list of landmarks forming the grid.
  */
  vpLandmarksGrid(const std::vector<vpLandmarkMeasurements> &landmarks)
    : m_landmarks(landmarks)
  { }

  /**
   * \brief Convert the prior of the UKF into the measurement space.
   *
   * \param[in] chi The prior.
   * \return vpColVector The prior expressed in the measurement space.
   */
  vpColVector state_to_measurement(const vpColVector &chi)
  {
    unsigned int nbLandmarks = m_landmarks.size();
    vpColVector measurements(2*nbLandmarks);
    for (unsigned int i = 0; i < nbLandmarks; ++i) {
      vpColVector landmarkMeas = m_landmarks[i].state_to_measurement(chi);
      measurements[2*i] = landmarkMeas[0];
      measurements[2*i + 1] = landmarkMeas[1];
    }
    return measurements;
  }

  /**
   * \brief Perfect measurement from each landmark of the range and relative orientation of the robot
   * located at pos.
   *
   * \param[in] pos The actual position of the robot (pos[0]: x, pos[1]: y, pos[2] = heading.
   * \return vpColVector n x ([0] the range [1] the relative orientation of the robot), where
   * n is the number of landmarks.
   */
  vpColVector measureGT(const vpColVector &pos)
  {
    unsigned int nbLandmarks = m_landmarks.size();
    vpColVector measurements(2*nbLandmarks);
    for (unsigned int i = 0; i < nbLandmarks; ++i) {
      vpColVector landmarkMeas = m_landmarks[i].measureGT(pos);
      measurements[2*i] = landmarkMeas[0];
      measurements[2*i + 1] = landmarkMeas[1];
    }
    return measurements;
  }

  /**
   * \brief Noisy measurement from each landmark of the range and relative orientation that
   * correspond to pos.
   *
   * \param[in] pos The actual position of the robot (pos[0]: x ; pos[1] = y ; pos[2] = heading).
   * \return vpColVector n x ([0] the range [1] the relative orientation of the robot), where
   * n is the number of landmarks.
   */
  vpColVector measureWithNoise(const vpColVector &pos)
  {
    unsigned int nbLandmarks = m_landmarks.size();
    vpColVector measurements(2*nbLandmarks);
    for (unsigned int i = 0; i < nbLandmarks; ++i) {
      vpColVector landmarkMeas = m_landmarks[i].measureWithNoise(pos);
      measurements[2*i] = landmarkMeas[0];
      measurements[2*i + 1] = landmarkMeas[1];
    }
    return measurements;
  }

private:
  std::vector<vpLandmarkMeasurements> m_landmarks; /*!< The list of landmarks forming the grid.*/
};

int main(/*const int argc, const char *argv[]*/)
{
  const double dt = 1.; // Period of 1s
  const double step = 10.; // Number of update of the robot position between two UKF filtering
  const double sigmaRange = 0.3; // Standard deviation of the range measurement: 0.3m
  const double sigmaSteering = vpMath::rad(1); // Standard deviation of the steering angle: 1deg
  const double sigmaBearing = vpMath::rad(0.5); // Standard deviation of the bearing angle: 0.5deg
  const double sigmaVel = 0.1; // Standard deviation of the velocity: 0.1m/s
  const double wheelbase = 0.5; // Wheelbase of 0.5m
  const std::vector<vpLandmarkMeasurements> landmarks = { vpLandmarkMeasurements(5, 10, sigmaRange, sigmaBearing)
                                                        , vpLandmarkMeasurements(10, 5, sigmaRange, sigmaBearing)
                                                        , vpLandmarkMeasurements(15, 15, sigmaRange, sigmaBearing) }; // Vector of landmarks constituing the grid
  const unsigned int nbLandmarks = landmarks.size(); // Number of landmarks constituing the grid
  const unsigned int nbCmds = 200;
  std::vector<vpColVector> cmds;
  for (unsigned int i = 0; i < nbCmds; ++i) {
    cmds.push_back(vpColVector({ 1.1, 0.1 }));
  }

  // Initialize the attributes of the UKF
  vpUKSigmaDrawerMerwe drawer(3, 0.00001, 2., 0, stateResidual, stateAdd);

  vpMatrix R1landmark(2, 2, 0.); // The covariance of the noise introduced by the measurement with 1 landmark
  R1landmark[0][0] = sigmaRange*sigmaRange;
  R1landmark[1][1] = sigmaBearing*sigmaBearing;
  vpMatrix R(2*nbLandmarks, 2 * nbLandmarks);
  for (unsigned int i = 0; i < nbLandmarks; ++i) {
    R.insert(R1landmark, 2*i, 2*i);
  }

  const double processVariance = 0.0001;
  vpMatrix Q; // The covariance of the process
  Q.eye(3);
  Q = Q * processVariance;

  vpMatrix P0(3, 3); //  The initial guess of the process covariance
  P0.eye(3);
  P0[0][0] = 0.1;
  P0[1][1] = 0.1;
  P0[2][2] = 0.05;

  vpColVector X0(3);
  X0[0] = 2.; // x = 2m
  X0[1] = 6.; // y = 6m
  X0[2] = 0.3; // robot orientation = 0.3 rad

  vpUnscentedKalman::vpProcessFunction f = fx;
  vpLandmarksGrid grid(landmarks);
  vpBicycleModel robot(wheelbase);
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  vpUnscentedKalman::vpMeasurementFunction h = std::bind(&vpLandmarksGrid::state_to_measurement, &grid, _1);
  vpUnscentedKalman::vpCommandStateFunction bx = std::bind(&vpBicycleModel::computeMotion, &robot, _1, _2, _3);

  // Initialize the UKF
  vpUnscentedKalman ukf(Q, R, &drawer, f, h);
  ukf.init(X0, P0);
  ukf.setCommandStateFunction(bx);
  ukf.setMeasurementMeanFunction(measurementMean);
  ukf.setMeasurementResidualFunction(measurementResidual);
  ukf.setStateAddFunction(stateAdd);
  ukf.setStateMeanFunction(stateMean);
  ukf.setStateResidualFunction(stateResidual);

  // Initialize the plot
  vpPlot plot(1);
  plot.initGraph(0, 2);
  plot.setTitle(0, "Position of the robot");
  plot.setUnitX(0, "Position along x(m)");
  plot.setUnitY(0, "Position along y (m)");
  plot.setLegend(0, 0, "GT");
  plot.setLegend(0, 1, "Filtered");

  // Initialize the simulation
  vpColVector robot_pos = X0;

  for (int i = 0; i < nbCmds; ++i) {
    robot_pos = robot.move(cmds[i], robot_pos, dt / step);
    if (i % static_cast<int>(step) == 0) {
      // Perform the measurement
      vpColVector z = grid.measureWithNoise(robot_pos);

      // Use the UKF to filter the measurement
      ukf.filter(z, dt, cmds[i]);

      // Plot the filtered state
      vpColVector Xest = ukf.getXest();
      plot.plot(0, 1, Xest[0], Xest[1]);
    }

    // Plot the ground truth
    plot.plot(0, 0, robot_pos[0], robot_pos[1]);
  }
  std::cout << "Press Enter to quit..." << std::endl;
  std::cin.get();
  return 0;
}
