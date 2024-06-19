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
 */

/** \example ukf-nonlinear-complex-example.cpp
 * Example of a complex non-linear use-case of the Unscented Kalman Filter (UKF).
 * The system we are interested in is a 4-wheel robot, moving at a low velocity.
 * As such, it can be modeled using a bicycle model.
 *
 * The state vector of the UKF is:
 * \f[
 * \begin{array}{lcl}
 *   \textbf{x}[0] &=& x \\
 *   \textbf{x}[1] &=& y \\
 *   \textbf{x}[2] &=& \theta
 * \end{array}
 * \f]
 * where \f$ \theta \f$ is the heading of the robot.
 *
 * The measurement \f$ \textbf{z} \f$ corresponds to the distance and relative orientation of the
 * robot with different landmarks. Be \f$ p_x^i \f$ and \f$ p_y^i \f$ the position of the \f$ i^{th} \f$ landmark
 * along the x and y axis, the measurement vector can be written as:
 * \f[
 *   \begin{array}{lcl}
 *      \textbf{z}[2i] &=& \sqrt{(p_x^i - x)^2 + (p_y^i - y)^2} \\
 *      \textbf{z}[2i+1] &=& \tan^{-1}{\frac{p_y^i - y}{p_x^i - x}} - \theta
 *    \end{array}
 * \f]
 *
 * Some noise is added to the measurement vector to simulate measurements which are
 * not perfect.
 *
 * The mean of several angles must be computed in the Unscented Transform. The definition we chose to use
 * is the following:
 *
 * \f$ mean(\boldsymbol{\theta}) = atan2 (\frac{\sum_{i=1}^n \sin{\theta_i}}{n}, \frac{\sum_{i=1}^n \cos{\theta_i}}{n})  \f$
 *
 * As the Unscented Transform uses a weighted mean, the actual implementation of the weighted mean
 * of several angles is the following:
 *
 * \f$ mean_{weighted}(\boldsymbol{\theta}) = atan2 (\sum_{i=1}^n w_m^i \sin{\theta_i}, \sum_{i=1}^n w_m^i \cos{\theta_i})  \f$
 *
 * where \f$ w_m^i \f$ is the weight associated to the \f$ i^{th} \f$ measurements for the weighted mean.
 *
 * Additionally, the addition and subtraction of angles must be carefully done, as the result
 * must stay in the interval \f$[- \pi ; \pi ]\f$ or \f$[0 ; 2 \pi ]\f$ . We decided to use
 * the interval \f$[- \pi ; \pi ]\f$ .
*/

// UKF includes
#include <visp3/core/vpUKSigmaDrawerMerwe.h>
#include <visp3/core/vpUnscentedKalman.h>

// ViSP includes
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpGaussRand.h>
#ifdef VISP_HAVE_DISPLAY
#include <visp3/gui/vpPlot.h>
#endif

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

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
  const unsigned int nbPoints = measurements.size();
  const unsigned int sizeMeasurement = measurements[0].size();
  const unsigned int nbLandmarks = sizeMeasurement / 2;
  vpColVector mean(sizeMeasurement, 0.);
  std::vector<double> sumCos(nbLandmarks, 0.);
  std::vector<double> sumSin(nbLandmarks, 0.);
  for (unsigned int i = 0; i < nbPoints; ++i) {
    for (unsigned int j = 0; j < nbLandmarks; ++j) {
      mean[2*j] += wm[i] * measurements[i][2*j];
      sumCos[j] += wm[i] * std::cos(measurements[i][(2*j)+1]);
      sumSin[j] += wm[i] * std::sin(measurements[i][(2*j)+1]);
    }
  }
  for (unsigned int j = 0; j < nbLandmarks; ++j) {
    mean[(2*j)+1] = std::atan2(sumSin[j], sumCos[j]);
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

/**
 * \brief Compute the commands realising a turn at constant linear velocity.
 *
 * \param[in] v Constant linear velocity.
 * \param[in] angleStart Starting angle (in degrees).
 * \param[in] angleStop Stop angle (in degrees).
 * \param[in] nbSteps Number of steps to perform the turn.
 * \return std::vector<vpColVector> The corresponding list of commands.
 */
std::vector<vpColVector> generateTurnCommands(const double &v, const double &angleStart, const double &angleStop, const unsigned int &nbSteps)
{
  std::vector<vpColVector> cmds;
  double dTheta = vpMath::rad(angleStop - angleStart) / static_cast<double>(nbSteps - 1);
  for (unsigned int i = 0; i < nbSteps; ++i) {
    double theta = vpMath::rad(angleStart) + dTheta * static_cast<double>(i);
    vpColVector cmd(2);
    cmd[0] = v;
    cmd[1] = theta;
    cmds.push_back(cmd);
  }
  return cmds;
}

/**
 * \brief Generate the list of commands for the simulation.
 *
 * @return std::vector<vpColVector> The list of commands to use in the simulation
 */
std::vector<vpColVector> generateCommands()
{
  std::vector<vpColVector> cmds;
  // Starting by an straight line acceleration
  unsigned int nbSteps = 30;
  double dv = (1.1 - 0.001) / static_cast<double>(nbSteps - 1);
  for (unsigned int i = 0; i < nbSteps; ++i) {
    vpColVector cmd(2);
    cmd[0] = 0.001 + static_cast<double>(i) * dv;
    cmd[1] = 0.;
    cmds.push_back(cmd);
  }

  // Left turn
  double lastLinearVelocity = cmds[cmds.size() -1][0];
  std::vector<vpColVector> leftTurnCmds = generateTurnCommands(lastLinearVelocity, 0, 2, 15);
  cmds.insert(cmds.end(), leftTurnCmds.begin(), leftTurnCmds.end());
  for (unsigned int i = 0; i < 100; ++i) {
    cmds.push_back(cmds[cmds.size() -1]);
  }

  // Right turn
  lastLinearVelocity = cmds[cmds.size() -1][0];
  std::vector<vpColVector> rightTurnCmds = generateTurnCommands(lastLinearVelocity, 2, -2, 15);
  cmds.insert(cmds.end(), rightTurnCmds.begin(), rightTurnCmds.end());
  for (unsigned int i = 0; i < 200; ++i) {
    cmds.push_back(cmds[cmds.size() -1]);
  }

  // Left  turn again
  lastLinearVelocity = cmds[cmds.size() -1][0];
  leftTurnCmds = generateTurnCommands(lastLinearVelocity, -2, 0, 15);
  cmds.insert(cmds.end(), leftTurnCmds.begin(), leftTurnCmds.end());
  for (unsigned int i = 0; i < 150; ++i) {
    cmds.push_back(cmds[cmds.size() -1]);
  }

  lastLinearVelocity = cmds[cmds.size() -1][0];
  leftTurnCmds = generateTurnCommands(lastLinearVelocity, 0, 1, 25);
  cmds.insert(cmds.end(), leftTurnCmds.begin(), leftTurnCmds.end());
  for (unsigned int i = 0; i < 150; ++i) {
    cmds.push_back(cmds[cmds.size() -1]);
  }

  return cmds;
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
   * \param[in] rel_angle_std The standard deviation of the relative angle measurements.
   */
  vpLandmarkMeasurements(const double &x, const double &y, const double &range_std, const double &rel_angle_std)
    : m_x(x)
    , m_y(y)
    , m_rngRange(range_std, 0., 4224)
    , m_rngRelativeAngle(rel_angle_std, 0., 2112)
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
      measurements[(2*i) + 1] = landmarkMeas[1];
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
      measurements[(2*i) + 1] = landmarkMeas[1];
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
      measurements[(2*i) + 1] = landmarkMeas[1];
    }
    return measurements;
  }

private:
  std::vector<vpLandmarkMeasurements> m_landmarks; /*!< The list of landmarks forming the grid.*/
};

int main(const int argc, const char *argv[])
{
  bool opt_useDisplay = true;
  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg == "-d") {
      opt_useDisplay = false;
    }
    else if ((arg == "-h") || (arg == "--help")) {
      std::cout << "SYNOPSIS" << std::endl;
      std::cout << "  " << argv[0] << " [-d][-h]" << std::endl;
      std::cout << std::endl << std::endl;
      std::cout << "DETAILS" << std::endl;
      std::cout << "  -d" << std::endl;
      std::cout << "    Deactivate display." << std::endl;
      std::cout << std::endl;
      std::cout << "  -h, --help" << std::endl;
      return 0;
    }
  }

  const double dt = 0.1; // Period of 0.1s
  const double step = 1.; // Number of update of the robot position between two UKF filtering
  const double sigmaRange = 0.3; // Standard deviation of the range measurement: 0.3m
  const double sigmaBearing = vpMath::rad(0.5); // Standard deviation of the bearing angle: 0.5deg
  const double wheelbase = 0.5; // Wheelbase of 0.5m
  const std::vector<vpLandmarkMeasurements> landmarks = { vpLandmarkMeasurements(5, 10, sigmaRange, sigmaBearing)
                                                        , vpLandmarkMeasurements(10, 5, sigmaRange, sigmaBearing)
                                                        , vpLandmarkMeasurements(15, 15, sigmaRange, sigmaBearing)
                                                        , vpLandmarkMeasurements(20, 5, sigmaRange, sigmaBearing)
                                                        , vpLandmarkMeasurements(0, 30, sigmaRange, sigmaBearing)
                                                        , vpLandmarkMeasurements(50, 30, sigmaRange, sigmaBearing)
                                                        , vpLandmarkMeasurements(40, 10, sigmaRange, sigmaBearing) }; // Vector of landmarks constituing the grid
  const unsigned int nbLandmarks = landmarks.size(); // Number of landmarks constituing the grid
  std::vector<vpColVector> cmds = generateCommands();
  const unsigned int nbCmds = cmds.size();

  // Initialize the attributes of the UKF
  std::shared_ptr<vpUKSigmaDrawerAbstract> drawer = std::make_shared<vpUKSigmaDrawerMerwe>(3, 0.1, 2., 0, stateResidual, stateAdd);

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
  vpUnscentedKalman ukf(Q, R, drawer, f, h);
  ukf.init(X0, P0);
  ukf.setCommandStateFunction(bx);
  ukf.setMeasurementMeanFunction(measurementMean);
  ukf.setMeasurementResidualFunction(measurementResidual);
  ukf.setStateAddFunction(stateAdd);
  ukf.setStateMeanFunction(stateMean);
  ukf.setStateResidualFunction(stateResidual);

#ifdef VISP_HAVE_DISPLAY
  vpPlot *plot = nullptr;
  if (opt_useDisplay) {
  // Initialize the plot
    plot = new vpPlot(1);
    plot->initGraph(0, 2);
    plot->setTitle(0, "Position of the robot");
    plot->setUnitX(0, "Position along x(m)");
    plot->setUnitY(0, "Position along y (m)");
    plot->setLegend(0, 0, "GT");
    plot->setLegend(0, 1, "Filtered");
  }
#endif

  // Initialize the simulation
  vpColVector robot_pos = X0;

  for (unsigned int i = 0; i < nbCmds; ++i) {
    robot_pos = robot.move(cmds[i], robot_pos, dt / step);
    if (i % static_cast<int>(step) == 0) {
      // Perform the measurement
      vpColVector z = grid.measureWithNoise(robot_pos);

      // Use the UKF to filter the measurement
      ukf.filter(z, dt, cmds[i]);

#ifdef VISP_HAVE_DISPLAY
      if (opt_useDisplay) {
        // Plot the filtered state
        vpColVector Xest = ukf.getXest();
        plot->plot(0, 1, Xest[0], Xest[1]);
      }
#endif
    }

#ifdef VISP_HAVE_DISPLAY
    if (opt_useDisplay) {
    // Plot the ground truth
      plot->plot(0, 0, robot_pos[0], robot_pos[1]);
    }
#endif
  }

  if (opt_useDisplay) {
    std::cout << "Press Enter to quit..." << std::endl;
    std::cin.get();
  }

#ifdef VISP_HAVE_DISPLAY
  if (opt_useDisplay) {
    delete plot;
  }
#endif

  vpColVector finalError = grid.state_to_measurement(ukf.getXest()) - grid.measureGT(robot_pos);
  const double maxError = 0.3;
  if (finalError.frobeniusNorm() > maxError) {
    std::cerr << "Error: max tolerated error = " << maxError << ", final error = " << finalError.frobeniusNorm() << std::endl;
    return -1;
  }
  return 0;
}
#else
int main()
{
  std::cout << "vpUnscentedKalman is only available if you compile ViSP in C++11 standard or higher." << std::endl;
  return 0;
}
#endif
