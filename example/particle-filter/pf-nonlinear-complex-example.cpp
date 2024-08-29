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

/**
 * \example pf-nonlinear-complex-example.cpp
 * Example of a complex non-linear use-case of the Particle Filter (PF).
 * The system we are interested in is a 4-wheel robot, moving at a low velocity.
 * As such, it can be modeled using a bicycle model.
 *
 * The state vector of the PF is:
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
 * The mean of several angles must be computed in the Particle Fitler inference. The definition we chose to use
 * is the following:
 *
 * \f$ mean(\boldsymbol{\theta}) = atan2 (\frac{\sum_{i=1}^n \sin{\theta_i}}{n}, \frac{\sum_{i=1}^n \cos{\theta_i}}{n})  \f$
 *
 * As the Particle Filter inference uses a weighted mean, the actual implementation of the weighted mean
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

// ViSP includes
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpGaussRand.h>
#ifdef VISP_HAVE_DISPLAY
#include <visp3/gui/vpPlot.h>
#endif

// PF includes
#include <visp3/core/vpParticleFilter.h>


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
vpColVector stateMean(const std::vector<vpColVector> &states, const std::vector<double> &wm, const vpParticleFilter<vpColVector>::vpStateAddFunction &/*addFunc*/)
{
  vpColVector mean(3, 0.);
  unsigned int nbPoints = static_cast<unsigned int>(states.size());
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
   * \param[in] distMaxAllowed Maximum distance allowed for the likelihood computation.
   */
  vpLandmarkMeasurements(const double &x, const double &y, const double &range_std, const double &rel_angle_std
                        , const double &distMaxAllowed)
    : m_x(x)
    , m_y(y)
    , m_rngRange(range_std, 0., 4224)
    , m_rngRelativeAngle(rel_angle_std, 0., 2112)
  {
    double sigmaDistance = distMaxAllowed / 3.;
    double sigmaDistanceSquared = sigmaDistance * sigmaDistance;
    m_constantDenominator = 1. / std::sqrt(2. * M_PI * sigmaDistanceSquared);
    m_constantExpDenominator = -1. / (2. * sigmaDistanceSquared);
  }

  /**
   * \brief Convert a particle of the Particle Filter into the measurement space.
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
    meas[1] = normalizeAngle(std::atan2(dy, dx));
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
    double orientation = normalizeAngle(std::atan2(dy, dx));
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

  void computePositionFromMeasurements(const vpColVector &meas, double &x, double &y)
  {
    double alpha = meas[1];
    x = m_x - meas[0] * std::cos(alpha);
    y = m_y - meas[0] * std::sin(alpha);
  }

    /**
     * \brief Compute the likelihood of a particle  (value between 0. and 1.)
     * knowing the measurements.
     *
     * \param[in] particle The particle state.
     * \param[in] meas The measurements.
     * \return double The likelihood of a particle  (value between 0. and 1.)
     */
  double likelihood(const vpColVector &particle, const vpColVector &meas)
  {
    double xMeas = 0., yMeas = 0.;
    computePositionFromMeasurements(meas, xMeas, yMeas);
    double dx = xMeas - particle[0];
    double dy = yMeas - particle[1];
    double dist = std::sqrt(dx * dx + dy * dy);
    double likelihood = std::exp(m_constantExpDenominator * dist) * m_constantDenominator;
    likelihood = std::min(likelihood, 1.0); // Clamp to have likelihood <= 1.
    likelihood = std::max(likelihood, 0.); // Clamp to have likelihood >= 0.
    return likelihood;
  }

private:
  double m_x; // The position along the x-axis of the landmark
  double m_y; // The position along the y-axis of the landmark
  vpGaussRand m_rngRange; // Noise simulator for the range measurement
  vpGaussRand m_rngRelativeAngle; // Noise simulator for the relative angle measurement
  double m_constantDenominator; // Denominator of the Gaussian function used in the likelihood computation.
  double m_constantExpDenominator; // Denominator of the exponential in the Gaussian function used in the likelihood computation.
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
  * \param[in] landmarks The list of landmarks forming the grid.
  * \param[in] distMaxAllowed Maximum distance allowed for the likelihood computation.
  */
  vpLandmarksGrid(const std::vector<vpLandmarkMeasurements> &landmarks, const double &distMaxAllowed)
    : m_landmarks(landmarks)
    , m_nbLandmarks(landmarks.size())
  {
    double sigmaDistance = distMaxAllowed / 3.;
    double sigmaDistanceSquared = sigmaDistance * sigmaDistance;
    m_constantDenominator = 1. / std::sqrt(2. * M_PI * sigmaDistanceSquared);
    m_constantExpDenominator = -1. / (2. * sigmaDistanceSquared);
  }

  /**
   * \brief Convert a particle of the Particle Filter into the measurement space.
   *
   * \param[in] chi The prior.
   * \return vpColVector The prior expressed in the measurement space.
   */
  vpColVector state_to_measurement(const vpColVector &chi)
  {
    vpColVector measurements(2*m_nbLandmarks);
    for (unsigned int i = 0; i < m_nbLandmarks; ++i) {
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
    vpColVector measurements(2*m_nbLandmarks);
    for (unsigned int i = 0; i < m_nbLandmarks; ++i) {
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
    vpColVector measurements(2*m_nbLandmarks);
    for (unsigned int i = 0; i < m_nbLandmarks; ++i) {
      vpColVector landmarkMeas = m_landmarks[i].measureWithNoise(pos);
      measurements[2*i] = landmarkMeas[0];
      measurements[(2*i) + 1] = landmarkMeas[1];
    }
    return measurements;
  }

  void computePositionFromMeasurements(const vpColVector &meas, double &x, double &y)
  {
    x = 0.;
    y = 0.;
    for (unsigned int i = 0; i < m_nbLandmarks; ++i) {
      vpColVector landmarkMeas({ meas[2*i], meas[(2*i) + 1] });
      double xLand = 0., yLand = 0.;
      m_landmarks[i].computePositionFromMeasurements(landmarkMeas, xLand, yLand);
      x += xLand;
      y += yLand;
    }
    x /= static_cast<double>(m_nbLandmarks);
    y /= static_cast<double>(m_nbLandmarks);
  }

  /**
   * \brief Compute the likelihood of a particle  (value between 0. and 1.)
   * knowing the measurements.
   *
   * \param[in] particle The particle state.
   * \param[in] meas The measurements.
   * \return double The likelihood of a particle  (value between 0. and 1.)
   */
  double likelihood(const vpColVector &particle, const vpColVector &meas)
  {
    double meanLikelihood = 0.;
    double meanX = 0., meanY = 0.;
    for (unsigned int i = 0; i < m_nbLandmarks; ++i) {
      vpColVector landmarkMeas({ meas[2*i], meas[(2*i) + 1] });
      double x = 0., y = 0.;
      m_landmarks[i].computePositionFromMeasurements(landmarkMeas, x, y);
      meanX += x;
      meanY += y;
    }
    meanX /= static_cast<double>(m_nbLandmarks);
    meanY /= static_cast<double>(m_nbLandmarks);
    double dx = meanX - particle[0];
    double dy = meanY - particle[1];
    double dist = std::sqrt(dx * dx + dy * dy);
    double likelihood = std::exp(m_constantExpDenominator * dist) * m_constantDenominator;
    likelihood = std::min(likelihood, 1.0); // Clamp to have likelihood <= 1.
    likelihood = std::max(likelihood, 0.); // Clamp to have likelihood >= 0.
    return likelihood;
  }

private:
  std::vector<vpLandmarkMeasurements> m_landmarks; /*!< The list of landmarks forming the grid.*/
  const unsigned int m_nbLandmarks; /*!< Number of landmarks that the grid is made of.*/
  double m_constantDenominator; // Denominator of the Gaussian function used in the likelihood computation.
  double m_constantExpDenominator; // Denominator of the exponential in the Gaussian function used in the likelihood computation.
};

struct SoftwareArguments
{
  // --- Main loop parameters---
  static const int SOFTWARE_CONTINUE = 42;
  bool m_useDisplay; //!< If true, activate the plot and the renderer if VISP_HAVE_DISPLAY is defined.
  unsigned int m_nbStepsWarmUp; //!< Number of steps for the warmup phase.
  // --- PF parameters---
  unsigned int m_N; //!< The number of particles.
  double m_maxDistanceForLikelihood; //!< The maximum allowed distance between a particle and the measurement, leading to a likelihood equal to 0..
  double m_ampliMaxX; //!< Amplitude max of the noise for the state component corresponding to the X coordinate.
  double m_ampliMaxY; //!< Amplitude max of the noise for the state component corresponding to the Y coordinate.
  double m_ampliMaxTheta; //!< Amplitude max of the noise for the state component corresponding to the heading.
  long m_seedPF; //!< Seed for the random generators of the PF.
  int m_nbThreads; //!< Number of thread to use in the Particle Filter.

  SoftwareArguments()
    : m_useDisplay(true)
    , m_nbStepsWarmUp(200)
    , m_N(500)
    , m_maxDistanceForLikelihood(0.5)
    , m_ampliMaxX(0.25)
    , m_ampliMaxY(0.25)
    , m_ampliMaxTheta(0.1)
    , m_seedPF(4224)
    , m_nbThreads(1)
  { }

  int parseArgs(const int argc, const char *argv[])
  {
    int i = 1;
    while (i < argc) {
      std::string arg(argv[i]);
      if ((arg == "--nb-steps-warmup") && ((i+1) < argc)) {
        m_nbStepsWarmUp = std::atoi(argv[i + 1]);
        ++i;
      }
      else if ((arg == "--max-distance-likelihood") && ((i+1) < argc)) {
        m_maxDistanceForLikelihood = std::atof(argv[i + 1]);
        ++i;
      }
      else if (((arg == "-N") || (arg == "--nb-particles")) && ((i+1) < argc)) {
        m_N = std::atoi(argv[i + 1]);
        ++i;
      }
      else if ((arg == "--seed") && ((i+1) < argc)) {
        m_seedPF = std::atoi(argv[i + 1]);
        ++i;
      }
      else if ((arg == "--nb-threads") && ((i+1) < argc)) {
        m_nbThreads = std::atoi(argv[i + 1]);
        ++i;
      }
      else if ((arg == "--ampli-max-X") && ((i+1) < argc)) {
        m_ampliMaxX = std::atof(argv[i + 1]);
        ++i;
      }
      else if ((arg == "--ampli-max-Y") && ((i+1) < argc)) {
        m_ampliMaxY = std::atof(argv[i + 1]);
        ++i;
      }
      else if ((arg == "--ampli-max-theta") && ((i+1) < argc)) {
        m_ampliMaxTheta = std::atof(argv[i + 1]);
        ++i;
      }
      else if (arg == "-d") {
        m_useDisplay = false;
      }
      else if ((arg == "-h") || (arg == "--help")) {
        printUsage(std::string(argv[0]));
        SoftwareArguments defaultArgs;
        defaultArgs.printDetails();
        return 0;
      }
      else {
        std::cout << "WARNING: unrecognised argument \"" << arg << "\"";
        if (i + 1 < argc) {
          std::cout << " with associated value(s) { ";
          int nbValues = 0;
          int j = i + 1;
          bool hasToRun = true;
          while ((j < argc) && hasToRun) {
            std::string nextValue(argv[j]);
            if (nextValue.find("--") == std::string::npos) {
              std::cout << nextValue << " ";
              ++nbValues;
            }
            else {
              hasToRun = false;
            }
            ++j;
          }
          std::cout << "}" << std::endl;
          i += nbValues;
        }
      }
      ++i;
    }
    return SOFTWARE_CONTINUE;
  }

private:
  void printUsage(const std::string &softName)
  {
    std::cout << "SYNOPSIS" << std::endl;
    std::cout << "  " << softName << " [--nb-steps-warmup <uint>]" << std::endl;
    std::cout << "  [--max-distance-likelihood <double>] [-N, --nb-particles <uint>] [--seed <int>] [--nb-threads <int>]" << std::endl;
    std::cout << "  [--ampli-max-X <double>] [--ampli-max-Y <double>] [--ampli-max-theta <double>]" << std::endl;
    std::cout << "  [-d, --no-display] [-h]" << std::endl;
  }

  void printDetails()
  {
    std::cout << std::endl << std::endl;
    std::cout << "DETAILS" << std::endl;
    std::cout << "  --nb-steps-warmup" << std::endl;
    std::cout << "    Number of steps in the warmup loop." << std::endl;
    std::cout << "    Default: " << m_nbStepsWarmUp << std::endl;
    std::cout << std::endl;
    std::cout << "  --max-distance-likelihood" << std::endl;
    std::cout << "    Maximum mean distance of the projection of the markers corresponding" << std::endl;
    std::cout << "    to a particle with the measurements. Above this value, the likelihood of the particle is 0." << std::endl;
    std::cout << "    Default: " << m_maxDistanceForLikelihood << std::endl;
    std::cout << std::endl;
    std::cout << "  -N, --nb-particles" << std::endl;
    std::cout << "    Number of particles of the Particle Filter." << std::endl;
    std::cout << "    Default: " << m_N << std::endl;
    std::cout << std::endl;
    std::cout << "  --seed" << std::endl;
    std::cout << "    Seed to initialize the Particle Filter." << std::endl;
    std::cout << "    Use a negative value makes to use the current timestamp instead." << std::endl;
    std::cout << "    Default: " << m_seedPF << std::endl;
    std::cout << std::endl;
    std::cout << "  --nb-threads" << std::endl;
    std::cout << "    Set the number of threads to use in the Particle Filter (only if OpenMP is available)." << std::endl;
    std::cout << "    Use a negative value to use the maximum number of threads instead." << std::endl;
    std::cout << "    Default: " << m_nbThreads << std::endl;
    std::cout << std::endl;
    std::cout << "  --ampli-max-X" << std::endl;
    std::cout << "    Maximum amplitude of the noise added to a particle along the X-axis." << std::endl;
    std::cout << "    Default: " << m_ampliMaxX << std::endl;
    std::cout << std::endl;
    std::cout << "  --ampli-max-Y" << std::endl;
    std::cout << "    Maximum amplitude of the noise added to a particle along the Y-axis." << std::endl;
    std::cout << "    Default: " << m_ampliMaxY << std::endl;
    std::cout << std::endl;
    std::cout << "  --ampli-max-theta" << std::endl;
    std::cout << "    Maximum amplitude of the noise added to a particle affecting the heading of the robot." << std::endl;
    std::cout << "    Default: " << m_ampliMaxTheta << std::endl;
    std::cout << std::endl;
    std::cout << "  -d, --no-display" << std::endl;
    std::cout << "    Deactivate display." << std::endl;
    std::cout << "    Default: display is ";
#ifdef VISP_HAVE_DISPLAY
    std::cout << "ON" << std::endl;
#else
    std::cout << "OFF" << std::endl;
#endif
    std::cout << std::endl;
    std::cout << "  -h, --help" << std::endl;
    std::cout << "    Display this help." << std::endl;
    std::cout << std::endl;
  }
};

int main(const int argc, const char *argv[])
{
  SoftwareArguments args;
  int returnCode = args.parseArgs(argc, argv);
  if (returnCode != SoftwareArguments::SOFTWARE_CONTINUE) {
    return returnCode;
  }

  const double dt = 0.1; // Period of 0.1s
  const double step = 1.; // Number of update of the robot position between two PF filtering
  const double sigmaRange = 0.3; // Standard deviation of the range measurement: 0.3m
  const double sigmaBearing = vpMath::rad(0.5); // Standard deviation of the bearing angle: 0.5deg
  const double wheelbase = 0.5; // Wheelbase of 0.5m
  const std::vector<vpLandmarkMeasurements> landmarks = { vpLandmarkMeasurements(5, 10, sigmaRange, sigmaBearing, args.m_maxDistanceForLikelihood)
                                                        , vpLandmarkMeasurements(10, 5, sigmaRange, sigmaBearing, args.m_maxDistanceForLikelihood)
                                                        , vpLandmarkMeasurements(15, 15, sigmaRange, sigmaBearing, args.m_maxDistanceForLikelihood)
                                                        , vpLandmarkMeasurements(20, 5, sigmaRange, sigmaBearing, args.m_maxDistanceForLikelihood)
                                                        , vpLandmarkMeasurements(0, 30, sigmaRange, sigmaBearing, args.m_maxDistanceForLikelihood)
                                                        , vpLandmarkMeasurements(50, 30, sigmaRange, sigmaBearing, args.m_maxDistanceForLikelihood)
                                                        , vpLandmarkMeasurements(40, 10, sigmaRange, sigmaBearing, args.m_maxDistanceForLikelihood) }; // Vector of landmarks constituting the grid
  std::vector<vpColVector> cmds = generateCommands();
  const unsigned int nbCmds = static_cast<unsigned int>(cmds.size());

  // Initialize the attributes of the PF
  std::vector<double> stdevsPF = { args.m_ampliMaxX / 3., args.m_ampliMaxY / 3., args.m_ampliMaxTheta / 3. }; ///TODO: define
  int seedPF = args.m_seedPF;
  unsigned int nbParticles = args.m_N;
  int nbThreads = args.m_nbThreads;

  vpColVector X0(3);
  X0[0] = 2.; // x = 2m
  X0[1] = 6.; // y = 6m
  X0[2] = 0.3; // robot orientation = 0.3 rad

  vpParticleFilter<vpColVector>::vpProcessFunction f = fx;
  vpLandmarksGrid grid(landmarks, args.m_maxDistanceForLikelihood);
  vpBicycleModel robot(wheelbase);
  using std::placeholders::_1;
  using std::placeholders::_2;
  vpParticleFilter<vpColVector>::vpLikelihoodFunction likelihoodFunc = std::bind(&vpLandmarksGrid::likelihood, &grid, _1, _2);
  vpParticleFilter<vpColVector>::vpResamplingConditionFunction checkResamplingFunc = vpParticleFilter<vpColVector>::simpleResamplingCheck;
  vpParticleFilter<vpColVector>::vpResamplingFunction resamplingFunc = vpParticleFilter<vpColVector>::simpleImportanceResampling;
  vpParticleFilter<vpColVector>::vpFilterFunction weightedMeanFunc = stateMean;
  vpParticleFilter<vpColVector>::vpStateAddFunction addFunc = stateAdd;

  // Initialize the PF
  vpParticleFilter<vpColVector> filter(nbParticles, stdevsPF, seedPF, nbThreads);
  filter.init(X0, f, likelihoodFunc, checkResamplingFunc, resamplingFunc, weightedMeanFunc, addFunc);

#ifdef VISP_HAVE_DISPLAY
  vpPlot *plot = nullptr;
  if (args.m_useDisplay) {
  // Initialize the plot
    plot = new vpPlot(1);
    plot->initGraph(0, 3);
    plot->setTitle(0, "Position of the robot");
    plot->setUnitX(0, "Position along x(m)");
    plot->setUnitY(0, "Position along y (m)");
    plot->setLegend(0, 0, "GT");
    plot->setLegend(0, 1, "Filtered");
    plot->setLegend(0, 2, "Measure");
    plot->setColor(0, 0, vpColor::red);
    plot->setColor(0, 1, vpColor::blue);
    plot->setColor(0, 2, vpColor::black);
  }
#endif

  // Initialize the simulation
  vpColVector robot_pos = X0;

  // Warm-up step
  double averageFilteringTime = 0.;
  for (unsigned int i = 0; i < args.m_nbStepsWarmUp; ++i) {
     // Perform the measurement
    vpColVector z = grid.measureWithNoise(robot_pos);

    double t0 = vpTime::measureTimeMicros();
    //! [Perform_filtering]
    // Use the PF to filter the measurement
    filter.filter(z, dt);
    //! [Perform_filtering]
    averageFilteringTime += vpTime::measureTimeMicros() - t0;
  }

  double meanErrorFilter = 0., meanErrorNoise = 0.;
  for (unsigned int i = 0; i < nbCmds; ++i) {
    robot_pos = robot.move(cmds[i], robot_pos, dt / step);
    if (i % static_cast<int>(step) == 0) {
      // Perform the measurement
      vpColVector z = grid.measureWithNoise(robot_pos);

      double t0 = vpTime::measureTimeMicros();
      //! [Perform_filtering]
      // Use the PF to filter the measurement
      filter.filter(z, dt);
      //! [Perform_filtering]
      averageFilteringTime += vpTime::measureTimeMicros() - t0;

      //! [Get_filtered_state]
      vpColVector Xest = filter.computeFilteredState();
      //! [Get_filtered_state]

      //! [Errors_computation]
      double dxFilter = Xest[0] - robot_pos[0];
      double dyFilter = Xest[1] - robot_pos[1];
      double errorFilter = std::sqrt(dxFilter * dxFilter + dyFilter * dyFilter);
      meanErrorFilter += errorFilter;
      double xMeas = 0., yMeas = 0.;
      grid.computePositionFromMeasurements(z, xMeas, yMeas);
      double dxMeas = xMeas - robot_pos[0];
      double dyMeas = yMeas - robot_pos[1];
      meanErrorNoise += std::sqrt(dxMeas * dxMeas + dyMeas * dyMeas);

#ifdef VISP_HAVE_DISPLAY
      if (args.m_useDisplay) {
        // Plot the filtered state
        plot->plot(0, 1, Xest[0], Xest[1]);
        plot->plot(0, 2, xMeas, yMeas);
      }
#endif
    }

#ifdef VISP_HAVE_DISPLAY
    if (args.m_useDisplay) {
    // Plot the ground truth
      plot->plot(0, 0, robot_pos[0], robot_pos[1]);
    }
#endif
  }

  averageFilteringTime = averageFilteringTime / (static_cast<double>(nbCmds + args.m_nbStepsWarmUp));
  meanErrorFilter = meanErrorFilter / (static_cast<double>(nbCmds));
  meanErrorNoise = meanErrorNoise / (static_cast<double>(nbCmds));
  std::cout << "Mean error filter = " << meanErrorFilter << std::endl;
  std::cout << "Mean error noise = " << meanErrorNoise << std::endl;
  std::cout << "Mean filtering time = " << averageFilteringTime << "us" << std::endl;

  if (args.m_useDisplay) {
    std::cout << "Press Enter to quit..." << std::endl;
    std::cin.get();
  }

#ifdef VISP_HAVE_DISPLAY
  if (args.m_useDisplay) {
    delete plot;
  }
#endif

  const double maxError = 0.3;
  if (meanErrorFilter > meanErrorNoise) {
    std::cerr << "Error: noisy measurements error = " << meanErrorNoise << ", filter error = " << meanErrorFilter << std::endl;
    return -1;
  }
  else if (meanErrorFilter > maxError) {
    std::cerr << "Error: max tolerated error = " << maxError << ", average error = " << meanErrorFilter << std::endl;
    return -1;
  }
  return 0;
}
#else
int main()
{
  std::cout << "This example is only available if you compile ViSP in C++11 standard or higher." << std::endl;
  return 0;
}
#endif
