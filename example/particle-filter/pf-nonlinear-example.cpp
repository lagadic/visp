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

/** \example pf-nonlinear-example.cpp
 * Example of a simple non-linear use-case of the Particle Filter (PF).
 *
 * The system we are interested in is an aircraft flying in the sky and
 * observed by a radar station. Its velocity is not completely constant: a Gaussian
 * noise is added to the velocity to simulate the effect of wind on the motion of the
 * aircraft.
 *
 * We consider the plan perpendicular to the ground and passing by both the radar
 * station and the aircraft. The x-axis corresponds to the position on the ground
 * and the y-axis to the altitude.
 *
 * The state vector of the PF corresponds to a constant velocity model and can be written as:
 *  \f[
      \begin{array}{lcl}
        \textbf{x}[0] &=& x \\
        \textbf{x}[1] &=& \dot{x} \\
        \textbf{x}[1] &=& y \\
        \textbf{x}[2] &=& \dot{y}
     \end{array}
    \f]

   The measurement \f$ \textbf{z} \f$ corresponds to the distance and angle between the ground and the aircraft
   observed by the radar station. Be \f$ p_x \f$ and \f$ p_y \f$ the position of the radar station
   along the x and y axis, the measurement vector can be written as:
   \f[
      \begin{array}{lcl}
        \textbf{z}[0] &=& \sqrt{(p_x^i - x)^2 + (p_y^i - y)^2} \\
        \textbf{z}[1] &=& \tan^{-1}{\frac{y - p_y}{x - p_x}}
     \end{array}
   \f]

 * Some noise is added to the measurement vector to simulate a sensor which is
 * not perfect.
 *
 * The mean of several angles must be computed during the inference of the Particle Filter. The definition we chose to use
   is the following:

   \f$ mean(\boldsymbol{\theta}) = atan2 (\frac{\sum_{i=1}^n \sin{\theta_i}}{n}, \frac{\sum_{i=1}^n \cos{\theta_i}}{n})  \f$

   As the Particle Filter inference uses a weighted mean, the actual implementation of the weighted mean
   of several angles is the following:

   \f$ mean_{weighted}(\boldsymbol{\theta}) = atan2 (\sum_{i=1}^n w_m^i \sin{\theta_i}, \sum_{i=1}^n w_m^i \cos{\theta_i})  \f$

   where \f$ w_m^i \f$ is the weight associated to the \f$ i^{th} \f$ measurements for the weighted mean.

   Additionnally, the addition and subtraction of angles must be carefully done, as the result
   must stay in the interval \f$[- \pi ; \pi ]\f$ or \f$[0 ; 2 \pi ]\f$ . We decided to use
   the interval \f$[- \pi ; \pi ]\f$ .
*/

// ViSP includes
#include <visp3/core/vpConfig.h>
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
 * \brief The process function, that updates the prior.
 *
 * \param[in] chi A sigma point.
 * \param[in] dt The period.
 * \return vpColVector The sigma points projected in the future.
 */
vpColVector fx(const vpColVector &chi, const double &dt)
{
  vpColVector point(4);
  point[0] = chi[1] * dt + chi[0];
  point[1] = chi[1];
  point[2] = chi[3] * dt + chi[2];
  point[3] = chi[3];
  return point;
}

/**
 * \brief Compute the X and Y coordinates from the measurements.
 *
 * \param z z[0] = range z[1] = elevation angle.
 * \param x The X-axis coordinate that corresponds to the measurements.
 * \param y The Y-axis coordinate that correspond to the measurements.
 */
void computeCoordinatesFromMeasurement(const vpColVector &z, double &x, double &y)
{
  x = z[0] * std::cos(z[1]);
  y = z[0] * std::sin(z[1]);
}

/**
 * \brief Compute the Eucledian norm of the error between point 0 and point 1.
 * \param x0 The X-axis coordinate of point 0.
 * \param y0 The Y-axis coordinate of point 0.
 * \param x1 The X-axis coordinate of point 1.
 * \param y1 The Y-axis coordinate of point 1.
 * \return \f[||e|| = \sqrt{(x0 - x1)^2 + (y0 - y1)^2}\f]
 */
double computeError(const double &x0, const double &y0, const double &x1, const double &y1)
{
  double dx = x0 - x1;
  double dy = y0 - y1;
  double error = std::sqrt(dx * dx + dy * dy);
  return error;
}

/**
 * \brief Compute the error between a state vector and a ground-truth vector.
 *
 * \param[in] state state[0] = X, state[1] = vX, state[2] = Y, state[3] = vY
 * \param[in] gt_X gt_X[0] = X, gt_X[1] = Y
 * \return double The error.
 */
double computeStateError(const vpColVector &state, const vpColVector &gt_X)
{
  double error = computeError(state[0], state[2], gt_X[0], gt_X[1]);
  return error;
}

/**
 * \brief Compute the error between a measurement vector and a ground-truth vector.
 *
 * \param[in] z z[0] = range z[1] = elevation angle.
 * \param[in] gt_X gt_X[0] = X, gt_X[1] = Y
 * \return double The error.
 */
double computeMeasurementsError(const vpColVector &z, const vpColVector &gt_X)
{
  double xMeas = 0., yMeas = 0.;
  computeCoordinatesFromMeasurement(z, xMeas, yMeas);
  double error = computeError(xMeas, yMeas, gt_X[0], gt_X[1]);
  return error;
}
}

/**
 * \brief Class that permits to convert the position of the aircraft into
 * range and elevation angle measurements.
 */
class vpRadarStation
{
public:
  /**
   * \brief Construct a new vpRadarStation object.
   *
   * \param[in] x The position on the ground of the radar.
   * \param[in] y The altitude of the radar.
   * \param[in] range_std The standard deviation of the range measurements.
   * \param[in] elev_angle_std The standard deviation of the elevation angle measurements.
   * \param[in] distMaxAllowed Maximum distance allowed for the likelihood computation.
   */
  vpRadarStation(const double &x, const double &y, const double &range_std, const double &elev_angle_std,
                 const double &distMaxAllowed)
    : m_x(x)
    , m_y(y)
    , m_rngRange(range_std, 0., 4224)
    , m_rngElevAngle(elev_angle_std, 0., 2112)
  {
    double sigmaDistance = distMaxAllowed / 3.;
    double sigmaDistanceSquared = sigmaDistance * sigmaDistance;
    m_constantDenominator = 1. / std::sqrt(2. * M_PI * sigmaDistanceSquared);
    m_constantExpDenominator = -1. / (2. * sigmaDistanceSquared);
  }

  /**
   * \brief Convert a particle of the Particle Filter into the measurement space.
   *
   * \param chi The prior.
   * \return vpColVector The prior expressed in the measurement space.
   */
  vpColVector state_to_measurement(const vpColVector &chi)
  {
    vpColVector meas(2);
    double dx = chi[0] - m_x;
    double dy = chi[2] - m_y;
    meas[0] = std::sqrt(dx * dx + dy * dy);
    meas[1] = std::atan2(dy, dx);
    return meas;
  }

  /**
   * \brief Perfect measurement of the range and elevation angle that
   * correspond to pos.
   *
   * \param pos The actual position of the aircraft (pos[0]: projection of the position
   * on the ground, pos[1]: altitude).
   * \return vpColVector [0] the range [1] the elevation angle.
   */
  vpColVector measureGT(const vpColVector &pos)
  {
    double dx = pos[0] - m_x;
    double dy = pos[1] - m_y;
    double range = std::sqrt(dx * dx + dy * dy);
    double elevAngle = std::atan2(dy, dx);
    vpColVector measurements(2);
    measurements[0] = range;
    measurements[1] = elevAngle;
    return measurements;
  }

  /**
   * \brief Noisy measurement of the range and elevation angle that
   * correspond to pos.
   *
   * \param pos The actual position of the aircraft (pos[0]: projection of the position
   * on the ground, pos[1]: altitude).
   * \return vpColVector [0] the range [1] the elevation angle.
   */
  vpColVector measureWithNoise(const vpColVector &pos)
  {
    vpColVector measurementsGT = measureGT(pos);
    vpColVector measurementsNoisy = measurementsGT;
    measurementsNoisy[0] += m_rngRange();
    measurementsNoisy[1] += m_rngElevAngle();
    return measurementsNoisy;
  }

  /**
   * \brief Compute the likelihood of a particle  (value between 0. and 1.)
   * knowing the measurements.
   * The likelihood function is based on a Gaussian function that penalizes
   * a particle that is "far" from the position corresponding to the measurements.
   *
   * \param[in] particle The particle state.
   * \param[in] meas The measurements.
   * \return double The likelihood of a particle  (value between 0. and 1.)
   */
  double likelihood(const vpColVector &particle, const vpColVector &meas)
  {
    double xParticle = particle[0];
    double yParticle = particle[2];
    double xMeas = 0., yMeas = 0.;
    computeCoordinatesFromMeasurement(meas, xMeas, yMeas);
    double dist = computeError(xParticle, yParticle, xMeas, yMeas);
    double likelihood = std::exp(m_constantExpDenominator * dist) * m_constantDenominator;
    likelihood = std::min(likelihood, 1.0); // Clamp to have likelihood <= 1.
    likelihood = std::max(likelihood, 0.); // Clamp to have likelihood >= 0.
    return likelihood;
  }

private:
  double m_x; // The position on the ground of the radar
  double m_y; // The altitude of the radar
  vpGaussRand m_rngRange; // Noise simulator for the range measurement
  vpGaussRand m_rngElevAngle; // Noise simulator for the elevation angle measurement
  double m_constantDenominator; // Denominator of the Gaussian function used in the likelihood computation.
  double m_constantExpDenominator; // Denominator of the exponential in the Gaussian function used in the likelihood computation.
};

/**
 * \brief Class to simulate a flying aircraft.
 */
class vpACSimulator
{
public:
  /**
   * \brief Construct a new vpACSimulator object.
   *
   * \param[in] X0 Initial position of the aircraft.
   * \param[in] vel Velocity of the aircraft.
   * \param[in] vel_std Standard deviation of the variation of the velocity.
   */
  vpACSimulator(const vpColVector &X0, const vpColVector &vel, const double &vel_std)
    : m_pos(X0)
    , m_vel(vel)
    , m_rngVel(vel_std, 0.)
  {

  }

  /**
   * \brief Compute the new position of the aircraft after dt seconds have passed
   * since the last update.
   *
   * \param[in] dt Period since the last update.
   * \return vpColVector The new position of the aircraft.
   */
  vpColVector update(const double &dt)
  {
    vpColVector dx = m_vel * dt;
    dx[0] += m_rngVel() * dt;
    dx[1] += m_rngVel() * dt;
    m_pos += dx;
    return m_pos;
  }

private:
  vpColVector m_pos; // Position of the simulated aircraft
  vpColVector m_vel; // Velocity of the simulated aircraft
  vpGaussRand m_rngVel; // Random generator for slight variations of the velocity of the aircraft
};

struct SoftwareArguments
{
  // --- Main loop parameters---
  static const int SOFTWARE_CONTINUE = 42;
  bool m_useDisplay; //!< If true, activate the plot and the renderer if VISP_HAVE_DISPLAY is defined.
  unsigned int m_nbStepsWarmUp; //!< Number of steps for the warmup phase.
  unsigned int m_nbSteps; //!< Number of steps for the main loop.
  double m_dt; // Period, expressed in seconds
  double m_sigmaRange; // Standard deviation of the range measurement, expressed in meters.
  double m_sigmaElevAngle; // Standard deviation of the elevation angle measurent, expressed in radians.
  double m_stdevAircraftVelocity; // Standard deviation of the velocity of the simulated aircraft, to make it deviate a bit from the constant velocity model
  double m_gt_X_init; // Ground truth initial position along the X-axis, in meters
  double m_gt_Y_init; // Ground truth initial position along the Y-axis, in meters
  double m_gt_vX_init; // Ground truth initial velocity along the X-axis, in meters
  double m_gt_vY_init; // Ground truth initial velocity along the Y-axis, in meters
  // --- PF parameters---
  unsigned int m_N; //!< The number of particles.
  double m_maxDistanceForLikelihood; //!< The maximum allowed distance between a particle and the measurement, leading to a likelihood equal to 0..
  double m_ampliMaxX; //!< Amplitude max of the noise for the state component corresponding to the X coordinate.
  double m_ampliMaxY; //!< Amplitude max of the noise for the state component corresponding to the Y coordinate.
  double m_ampliMaxVx; //!< Amplitude max of the noise for the state component corresponding to the velocity along the X-axis.
  double m_ampliMaxVy; //!< Amplitude max of the noise for the state component corresponding to the velocity along the Y-axis.
  long m_seedPF; //!< Seed for the random generators of the PF.
  int m_nbThreads; //!< Number of thread to use in the Particle Filter.

  SoftwareArguments()
    : m_useDisplay(true)
    , m_nbStepsWarmUp(200)
    , m_nbSteps(300)
    , m_dt(3.)
    , m_sigmaRange(5)
    , m_sigmaElevAngle(vpMath::rad(0.5))
    , m_stdevAircraftVelocity(0.2)
    , m_gt_X_init(-500.)
    , m_gt_Y_init(1000.)
    , m_gt_vX_init(10.)
    , m_gt_vY_init(5.)
    , m_N(500)
    , m_maxDistanceForLikelihood(50.)
    , m_ampliMaxX(20.)
    , m_ampliMaxY(200.)
    , m_ampliMaxVx(1.)
    , m_ampliMaxVy(0.5)
    , m_seedPF(4224)
    , m_nbThreads(1)
  { }

  int parseArgs(const int argc, const char *argv[])
  {
    int i = 1;
    while (i < argc) {
      std::string arg(argv[i]);
      if ((arg == "--nb-steps-main") && ((i+1) < argc)) {
        m_nbSteps = std::atoi(argv[i + 1]);
        ++i;
      }
      else if ((arg == "--nb-steps-warmup") && ((i+1) < argc)) {
        m_nbStepsWarmUp = std::atoi(argv[i + 1]);
        ++i;
      }
      else if ((arg == "--dt") && ((i+1) < argc)) {
        m_dt = std::atoi(argv[i + 1]);
        ++i;
      }
      else if ((arg == "--stdev-range") && ((i+1) < argc)) {
        m_sigmaRange = std::atof(argv[i + 1]);
        ++i;
      }
      else if ((arg == "--stdev-elev-angle") && ((i+1) < argc)) {
        m_sigmaElevAngle = vpMath::rad(std::atof(argv[i + 1]));
        ++i;
      }
      else if ((arg == "--stdev-aircraft-vel") && ((i+1) < argc)) {
        m_stdevAircraftVelocity = std::atof(argv[i + 1]);
        ++i;
      }
      else if ((arg == "--gt-X0") && ((i+1) < argc)) {
        m_gt_X_init = std::atof(argv[i + 1]);
        ++i;
      }
      else if ((arg == "--gt-Y0") && ((i+1) < argc)) {
        m_gt_Y_init = std::atof(argv[i + 1]);
        ++i;
      }
      else if ((arg == "--gt-vX0") && ((i+1) < argc)) {
        m_gt_vX_init = std::atof(argv[i + 1]);
        ++i;
      }
      else if ((arg == "--gt-vY0") && ((i+1) < argc)) {
        m_gt_vY_init = std::atof(argv[i + 1]);
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
      else if ((arg == "--ampli-max-vX") && ((i+1) < argc)) {
        m_ampliMaxVx = std::atof(argv[i + 1]);
        ++i;
      }
      else if ((arg == "--ampli-max-vY") && ((i+1) < argc)) {
        m_ampliMaxVy = std::atof(argv[i + 1]);
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
    std::cout << "  " << softName << " [--nb-steps-main <uint>] [--nb-steps-warmup <uint>]" << std::endl;
    std::cout << "  [--dt <double>] [--stdev-range <double>] [--stdev-elev-angle <double>] [--stdev-aircraft-vel <double>]" << std::endl;
    std::cout << "  [--gt-X0 <double>] [--gt-Y0 <double>] [--gt-vX0 <double>] [--gt-vY0 <double>]" << std::endl;
    std::cout << "  [--max-distance-likelihood <double>] [-N, --nb-particles <uint>] [--seed <int>] [--nb-threads <int>]" << std::endl;
    std::cout << "  [--ampli-max-X <double>] [--ampli-max-Y <double>] [--ampli-max-vX <double>] [--ampli-max-vY <double>]" << std::endl;
    std::cout << "  [-d, --no-display] [-h]" << std::endl;
  }

  void printDetails()
  {
    std::cout << std::endl << std::endl;
    std::cout << "DETAILS" << std::endl;
    std::cout << "  --nb-steps-main" << std::endl;
    std::cout << "    Number of steps in the main loop." << std::endl;
    std::cout << "    Default: " << m_nbSteps << std::endl;
    std::cout << std::endl;
    std::cout << "  --nb-steps-warmup" << std::endl;
    std::cout << "    Number of steps in the warmup loop." << std::endl;
    std::cout << "    Default: " << m_nbStepsWarmUp << std::endl;
    std::cout << std::endl;
    std::cout << "  --dt" << std::endl;
    std::cout << "    Timestep of the simulation, in seconds." << std::endl;
    std::cout << "    Default: " << m_dt << std::endl;
    std::cout << std::endl;
    std::cout << "  --stdev-range" << std::endl;
    std::cout << "    Standard deviation of the range measurements, in meters." << std::endl;
    std::cout << "    Default: " << m_sigmaRange << std::endl;
    std::cout << std::endl;
    std::cout << "  --stdev-elev-angle" << std::endl;
    std::cout << "    Standard deviation of the elevation angle measurements, in degrees." << std::endl;
    std::cout << "    Default: " << vpMath::deg(m_sigmaElevAngle) << std::endl;
    std::cout << std::endl;
    std::cout << "  --stdev-aircraft-vel" << std::endl;
    std::cout << "    Standard deviation of the aircraft velocity, in m/s." << std::endl;
    std::cout << "    Default: " << m_stdevAircraftVelocity << std::endl;
    std::cout << std::endl;
    std::cout << "  --gt-X0" << std::endl;
    std::cout << "    Initial position along the X-axis of the aircraft, in meters." << std::endl;
    std::cout << "    Be careful, because singularities happen if the aircraft flies above the radar." << std::endl;
    std::cout << "    Default: " << m_gt_X_init << std::endl;
    std::cout << std::endl;
    std::cout << "  --gt-Y0" << std::endl;
    std::cout << "    Initial position along the Y-axis of the aircraft, in meters." << std::endl;
    std::cout << "    Be careful, because singularities happen if the aircraft flies above the radar." << std::endl;
    std::cout << "    Default: " << m_gt_Y_init << std::endl;
    std::cout << std::endl;
    std::cout << "  --gt-vX0" << std::endl;
    std::cout << "    Initial velocity along the X-axis of the aircraft, in m/s." << std::endl;
    std::cout << "    Be careful, because singularities happen if the aircraft flies above the radar." << std::endl;
    std::cout << "    Default: " << m_gt_vX_init << std::endl;
    std::cout << std::endl;
    std::cout << "  --gt-vY0" << std::endl;
    std::cout << "    Initial velocity along the Y-axis of the aircraft, in m/s." << std::endl;
    std::cout << "    Be careful, because singularities happen if the aircraft flies above the radar." << std::endl;
    std::cout << "    Default: " << m_gt_vY_init << std::endl;
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
    std::cout << "  --ampli-max-vX" << std::endl;
    std::cout << "    Maximum amplitude of the noise added to a particle to the velocity along the X-axis component." << std::endl;
    std::cout << "    Default: " << m_ampliMaxVx << std::endl;
    std::cout << std::endl;
    std::cout << "  --ampli-max-vY" << std::endl;
    std::cout << "    Maximum amplitude of the noise added to a particle to the velocity along the Y-axis component." << std::endl;
    std::cout << "    Default: " << m_ampliMaxVy << std::endl;
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

  const double dt = 3.; // Period of 3s
  const double sigmaRange = 5; // Standard deviation of the range measurement: 5m
  const double sigmaElevAngle = vpMath::rad(0.5); // Standard deviation of the elevation angle measurent: 0.5deg
  const double stdevAircraftVelocity = 0.2; // Standard deviation of the velocity of the simulated aircraft, to make it deviate a bit from the constant velocity model
  const double gt_X_init = -500.; // Ground truth initial position along the X-axis, in meters
  const double gt_Y_init = 1000.; // Ground truth initial position along the Y-axis, in meters
  const double gt_vX_init = 100.; // Ground truth initial velocity along the X-axis, in meters
  const double gt_vY_init = 5.; // Ground truth initial velocity along the Y-axis, in meters

  // Initialize the attributes of the PF
  std::vector<double> stdevsPF = { args.m_ampliMaxX /3., args.m_ampliMaxVx /3., args.m_ampliMaxY /3. , args.m_ampliMaxVy /3. };
  int seedPF = args.m_seedPF;
  unsigned int nbParticles = args.m_N;
  int nbThreads = args.m_nbThreads;

  vpColVector X0(4);
  X0[0] = 0.9 * gt_X_init; // x, i.e. 10% of error with regard to ground truth
  X0[1] = 0.9 * gt_vX_init; // dx/dt, i.e. 10% of error with regard to ground truth
  X0[2] = 0.9 * gt_Y_init; // y, i.e. 10% of error with regard to ground truth
  X0[3] = 0.9 * gt_vY_init; // dy/dt, i.e. 10% of error with regard to ground truth

  vpParticleFilter<vpColVector>::vpProcessFunction f = fx;
  vpRadarStation radar(0., 0., sigmaRange, sigmaElevAngle, args.m_maxDistanceForLikelihood);
  using std::placeholders::_1;
  using std::placeholders::_2;
  vpParticleFilter<vpColVector>::vpLikelihoodFunction likelihoodFunc = std::bind(&vpRadarStation::likelihood, &radar, _1, _2);
  vpParticleFilter<vpColVector>::vpResamplingConditionFunction checkResamplingFunc = vpParticleFilter<vpColVector>::simpleResamplingCheck;
  vpParticleFilter<vpColVector>::vpResamplingFunction resamplingFunc = vpParticleFilter<vpColVector>::simpleImportanceResampling;

  // Initialize the PF
  vpParticleFilter<vpColVector> filter(nbParticles, stdevsPF, seedPF, nbThreads);
  filter.init(X0, f, likelihoodFunc, checkResamplingFunc, resamplingFunc);

#ifdef VISP_HAVE_DISPLAY
  vpPlot *plot = nullptr;
  if (args.m_useDisplay) {
  // Initialize the plot
    plot = new vpPlot(4);
    plot->initGraph(0, 3);
    plot->setTitle(0, "Position along X-axis");
    plot->setUnitX(0, "Time (s)");
    plot->setUnitY(0, "Position (m)");
    plot->setLegend(0, 0, "GT");
    plot->setLegend(0, 1, "Filtered");
    plot->setLegend(0, 2, "Measure");
    plot->setColor(0, 0, vpColor::red);
    plot->setColor(0, 1, vpColor::blue);
    plot->setColor(0, 2, vpColor::black);

    plot->initGraph(1, 3);
    plot->setTitle(1, "Velocity along X-axis");
    plot->setUnitX(1, "Time (s)");
    plot->setUnitY(1, "Velocity (m/s)");
    plot->setLegend(1, 0, "GT");
    plot->setLegend(1, 1, "Filtered");
    plot->setLegend(1, 2, "Measure");
    plot->setColor(1, 0, vpColor::red);
    plot->setColor(1, 1, vpColor::blue);
    plot->setColor(1, 2, vpColor::black);

    plot->initGraph(2, 3);
    plot->setTitle(2, "Position along Y-axis");
    plot->setUnitX(2, "Time (s)");
    plot->setUnitY(2, "Position (m)");
    plot->setLegend(2, 0, "GT");
    plot->setLegend(2, 1, "Filtered");
    plot->setLegend(2, 2, "Measure");
    plot->setColor(2, 0, vpColor::red);
    plot->setColor(2, 1, vpColor::blue);
    plot->setColor(2, 2, vpColor::black);

    plot->initGraph(3, 3);
    plot->setTitle(3, "Velocity along Y-axis");
    plot->setUnitX(3, "Time (s)");
    plot->setUnitY(3, "Velocity (m/s)");
    plot->setLegend(3, 0, "GT");
    plot->setLegend(3, 1, "Filtered");
    plot->setLegend(3, 2, "Measure");
    plot->setColor(3, 0, vpColor::red);
    plot->setColor(3, 1, vpColor::blue);
    plot->setColor(3, 2, vpColor::black);
  }
#endif

  // Initialize the simulation
  vpColVector ac_pos(2);
  ac_pos[0] = gt_X_init;
  ac_pos[1] = gt_Y_init;
  vpColVector ac_vel(2);
  ac_vel[0] = gt_vX_init;
  ac_vel[1] = gt_vY_init;
  vpACSimulator ac(ac_pos, ac_vel, stdevAircraftVelocity);
  vpColVector gt_Xprec = ac_pos;
  vpColVector gt_Vprec = ac_vel;
  double averageFilteringTime = 0.;
  double meanErrorFilter = 0., meanErrorNoise = 0.;
  double xNoise_prec = 0., yNoise_prec = 0.;

  // Warmup loop
  const unsigned int nbStepsWarmUp = args.m_nbStepsWarmUp;
  for (unsigned int i = 0; i < nbStepsWarmUp; ++i) {
    // Update object pose
    vpColVector gt_X = ac.update(dt);

    // Perform the measurement
    vpColVector z = radar.measureWithNoise(gt_X);

    // Use the UKF to filter the measurement
    double t0 = vpTime::measureTimeMicros();
    filter.filter(z, dt);
    averageFilteringTime += vpTime::measureTimeMicros() - t0;
    gt_Xprec = gt_X;

    // Save the noisy position
    computeCoordinatesFromMeasurement(z, xNoise_prec, yNoise_prec);
  }

  for (unsigned int i = 0; i < args.m_nbSteps; ++i) {
    // Perform the measurement
    vpColVector gt_X = ac.update(dt);
    vpColVector gt_V = (gt_X - gt_Xprec) / dt;
    vpColVector z = radar.measureWithNoise(gt_X);

    // Use the PF to filter the measurement
    double t0 = vpTime::measureTimeMicros();
    filter.filter(z, dt);
    averageFilteringTime += vpTime::measureTimeMicros() - t0;

    vpColVector Xest = filter.computeFilteredState();
    vpColVector gtState = vpColVector({ gt_Xprec[0], gt_Vprec[0], gt_Xprec[1], gt_Vprec[1] });
    double normErrorFilter = computeStateError(Xest, gt_X);
    meanErrorFilter += normErrorFilter;
    double xNoise = 0., yNoise = 0.;
    computeCoordinatesFromMeasurement(z, xNoise, yNoise);
    double normErrorNoise = computeMeasurementsError(z, gt_X);
    meanErrorNoise += normErrorNoise;

#ifdef VISP_HAVE_DISPLAY
    if (args.m_useDisplay) {
    // Plot the ground truth, measurement and filtered state
      plot->plot(0, 0, i, gt_X[0]);
      plot->plot(0, 1, i, Xest[0]);
      plot->plot(0, 2, i, xNoise);

      double vxNoise = (xNoise - xNoise_prec) / dt;
      plot->plot(1, 0, i, gt_V[0]);
      plot->plot(1, 1, i, Xest[1]);
      plot->plot(1, 2, i, vxNoise);

      plot->plot(2, 0, i, gt_X[1]);
      plot->plot(2, 1, i, Xest[2]);
      plot->plot(2, 2, i, yNoise);

      double vyNoise = (yNoise - yNoise_prec) / dt;
      plot->plot(3, 0, i, gt_V[1]);
      plot->plot(3, 1, i, Xest[3]);
      plot->plot(3, 2, i, vyNoise);
    }
#endif

    gt_Xprec = gt_X;
    gt_Vprec = gt_V;
    xNoise_prec = xNoise;
    yNoise_prec = yNoise;
  }

  meanErrorFilter /= static_cast<double>(args.m_nbSteps);
  meanErrorNoise /= static_cast<double>(args.m_nbSteps);
  averageFilteringTime = averageFilteringTime / (static_cast<double>(args.m_nbSteps) + static_cast<double>(nbStepsWarmUp));
  std::cout << "Mean error filter = " << meanErrorFilter << "m" << std::endl;
  std::cout << "Mean error noise = " << meanErrorNoise << "m" << std::endl;
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

  const double maxError = 150.;
  if (meanErrorFilter > maxError) {
    std::cerr << "Error: max tolerated error = " << maxError << ", mean error = " << meanErrorFilter << std::endl;
    return -1;
  }
  else if (meanErrorFilter >= meanErrorNoise) {
    std::cerr << "Error: mean error without filter = " << meanErrorNoise << ", mean error with filter = " << meanErrorFilter << std::endl;
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
