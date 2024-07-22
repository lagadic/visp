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
 *
*****************************************************************************/

/** \example pf-nonlinear-example.cpp
 * Example on how to use a Particle Filter (PF) on a complex non-linear use-case.
 * The system is an object, whose coordinate frame origin is the point O, on which are sticked four markers.
 * The object revolves in a plane parallel to the ground around a fixed point W whose coordinate frame is the world frame.
 * The scene is observed by a pinhole camera whose coordinate frame has the origin C and which is
 * fixed to the ceiling.
 *
 * The state vector of the PF is:
 *  \f{eqnarray*}{
        \textbf{x}[0] &=& {}^WX_x \\
        \textbf{x}[1] &=& {}^WX_y \\
        \textbf{x}[2] &=& {}^WX_z \\
        \textbf{x}[3] &=& \omega \Delta t
   \f}

   The measurement \f$ \textbf{z} \f$ corresponds to the coordinates in pixels of the different markers.
   Be \f$ u_i \f$ and \f$ v_i \f$ the horizontal and vertical pixel coordinates of the \f$ i^{th} \f$ marker.
   The measurement vector can be written as:
   \f{eqnarray*}{
        \textbf{z}[2i] &=& u_i \\
        \textbf{z}[2i+1] &=& v_i
   \f}

 * Some noise is added to the measurement vector to simulate measurements which are
 * not perfect.
*/

// ViSP includes
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpGaussRand.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPixelMeterConversion.h>
//! [Display_includes]
#ifdef VISP_HAVE_DISPLAY
#include <visp3/gui/vpPlot.h>
#include <visp3/gui/vpDisplayFactory.h>
#endif
//! [Display_includes]
#include <visp3/vision/vpPose.h>

//! [PF_includes]
#include <visp3/core/vpParticleFilter.h>
//! [PF_includes]

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
//! [Process_function]
/**
 * \brief Process function that makes evolve the state model {\f$ {}^WX_x \f$, \f$ {}^WX_y \f$, \f$ {}^WX_z \f$, \f$ C = \omega \Delta t \f$}
 * over time.
 *
 * \param[in] x The state vector
 * \return vpColVector The state vector at the next iteration.
 */
vpColVector fx(const vpColVector &x, const double & /*dt*/)
{
  vpColVector x_kPlus1(4);
  x_kPlus1[0] = x[0] * std::cos(x[3]) - x[1] * std::sin(x[3]); // wX
  x_kPlus1[1] = x[0] * std::sin(x[3]) + x[1] * std::cos(x[3]); // wY
  x_kPlus1[2] = x[2]; // wZ
  x_kPlus1[3] = x[3]; // omega * dt
  return x_kPlus1;
}
//! [Process_function]

//! [Object_simulator]
/**
 * \brief Class that simulates the moving object.
 */
class vpObjectSimulator
{
public:
  /**
   * \brief Construct a new vpObjectSimulator object.
   *
   * \param[in] R The radius of the revolution around the world frame origin.
   * \param[in] w The pulsation of the motion.
   * \param[in] phi The phase of the motion.
   * \param[in] wZ The y-coordinate of the object in the world frame.
   */
  vpObjectSimulator(const double &R, const double &w, const double &phi, const double &wZ)
    : m_R(R)
    , m_w(w)
    , m_phi(phi)
    , m_wZ(wZ)
  { }

  /**
   * \brief Move the object to its new position, expressed in the world frame.
   *
   * \param[in] t The current time.
   * \return vpColVector The new position of the object in the world frame, expressed as homogeneous coordinates.
   */
  vpColVector move(const double &t)
  {
    vpColVector wX(4, 1.);
    wX[0] = m_R * std::cos(m_w * t + m_phi);
    wX[1] = m_R * std::sin(m_w * t + m_phi);
    wX[2] = m_wZ;
    return wX;
  }

private:
  double m_R; // Radius of the revolution around the world frame origin.
  double m_w; // Pulsation of the motion.
  double m_phi; // Phase of the motion.
  const double m_wZ; // The z-coordinate of the object in the world frame.
};
//! [Object_simulator]

//! [Markers_class]
/**
 * \brief Class that permits to convert the 3D position of the object into measurements.
 */
class vpMarkersMeasurements
{
public:
  /**
   * \brief Construct a new vpMarkersMeasurements object.
   *
   * \param[in] cam The camera parameters.
   * \param[in] cMw The pose of the world frame with regard to the camera frame.
   * \param[in] wRo The rotation matrix expressing the rotation between the world frame and object frame.
   * \param[in] markers The position of the markers in the object frame.
   * \param[in] sigmaDistance Standard deviation of the Gaussian function used for the computation of the likelihood.
   * An error greater than 3 times this standard deviation will lead to a likelihood equal to 0.
   * \param[in] noise_stdev The standard deviation for the noise generator
   * \param[in] seed The seed for the noise generator
   */
  vpMarkersMeasurements(const vpCameraParameters &cam, const vpHomogeneousMatrix &cMw, const vpRotationMatrix &wRo,
                        const std::vector<vpColVector> &markers, const double &sigmaDistance,
                        const double &noise_stdev, const long &seed)
    : m_cam(cam)
    , m_cMw(cMw)
    , m_wRo(wRo)
    , m_markers(markers)
    , m_rng(noise_stdev, 0., seed)
  {
    double sigmaDistanceSquared = sigmaDistance * sigmaDistance;
    m_constantDenominator = 1. / std::sqrt(2. * M_PI * sigmaDistanceSquared);
    m_constantExpDenominator = -1. / (2. * sigmaDistanceSquared);
  }

  //! [Likelihood_function]
  /**
   * \brief Compute the likelihood of a particle compared to the measurements.
   * The likelihood equals zero if the particle is completely different of
   * the measurements and equals one if it matches completely.
   * The chosen likelihood is a Gaussian function that penalizes the mean distance
   * between the projection of the markers corresponding to the particle position
   * and the measurements of the markers in the image.
   *
   * \param[in] x The particle.
   * \param[in] meas The measurement vector. meas[2i] = u_i meas[2i + 1] = v_i .
   * \return double The likelihood of the particle.
   */
  double likelihoodParticle(const vpColVector &x, const vpColVector &meas)
  {
    unsigned int nbMarkers = m_markers.size();
    double likelihood = 0.;
    vpHomogeneousMatrix wMo;
    vpTranslationVector wTo(x[0], x[1], x[2]);
    wMo.build(wTo, m_wRo);
    const unsigned int sizePt2D = 2;
    const unsigned int idX = 0, idY = 1, idZ = 2;
    double sumError = 0.;
    for (unsigned int i = 0; i < nbMarkers; ++i) {
      vpColVector cX = m_cMw * wMo * m_markers[i];
      vpImagePoint projParticle;
      vpMeterPixelConversion::convertPoint(m_cam, cX[idX] / cX[idZ], cX[idY] / cX[idZ], projParticle);
      vpImagePoint measPt(meas[sizePt2D * i + 1], meas[sizePt2D * i]);
      double error = vpImagePoint::sqrDistance(projParticle, measPt);
      sumError += error;
    }
    likelihood = std::exp(m_constantExpDenominator * sumError / static_cast<double>(nbMarkers)) * m_constantDenominator;
    likelihood = std::min(likelihood, 1.0); // Clamp to have likelihood <= 1.
    likelihood = std::max(likelihood, 0.); // Clamp to have likelihood >= 0.
    return likelihood;
  }
  //! [Likelihood_function]

  /**
   * \brief Convert the state of the PF into the measurement space.
   *
   * \param[in] x The state of the PF.
   * \return vpColVector The state expressed in the measurement space.
   */
  vpColVector state_to_measurement(const vpColVector &x)
  {
    unsigned int nbMarkers = m_markers.size();
    vpColVector meas(2*nbMarkers);
    vpHomogeneousMatrix wMo;
    vpTranslationVector wTo(x[0], x[1], x[2]);
    wMo.build(wTo, m_wRo);
    for (unsigned int i = 0; i < nbMarkers; ++i) {
      vpColVector cX = m_cMw * wMo * m_markers[i];
      double u = 0., v = 0.;
      vpMeterPixelConversion::convertPoint(m_cam, cX[0] / cX[2], cX[1] / cX[2], u, v);
      meas[2*i] = u;
      meas[2*i + 1] = v;
    }
    return meas;
  }

  //! [GT_measurements]
  /**
   * \brief Perfect measurement of the projection of the markers in the image when the object
   * is located at \b wX.
   *
   * \param[in] wX The actual position of the robot (wX[0]: x, wX[1]: y, wX[2] = z).
   * \return vpColVector [2*i] u_i [2*i + 1] v_i where i is the index of the marker.
   */
  vpColVector measureGT(const vpColVector &wX)
  {
    unsigned int nbMarkers = m_markers.size();
    vpColVector meas(2*nbMarkers);
    vpHomogeneousMatrix wMo;
    vpTranslationVector wTo(wX[0], wX[1], wX[2]);
    wMo.build(wTo, m_wRo);
    for (unsigned int i = 0; i < nbMarkers; ++i) {
      vpColVector cX = m_cMw * wMo * m_markers[i];
      double u = 0., v = 0.;
      vpMeterPixelConversion::convertPoint(m_cam, cX[0] / cX[2], cX[1] / cX[2], u, v);
      meas[2*i] = u;
      meas[2*i + 1] = v;
    }
    return meas;
  }
  //! [GT_measurements]

  //! [Noisy_measurements]
  /**
   * \brief Noisy measurement of the projection of the markers in the image when the object
   * is located at \b wX.
   *
   * \param[in] wX The actual position of the robot (wX[0]: x, wX[1]: y, wX[2] = z).
   * \return vpColVector [2*i] u_i [2*i + 1] v_i where i is the index of the marker.
   */
  vpColVector measureWithNoise(const vpColVector &wX)
  {
    vpColVector measurementsGT = measureGT(wX);
    vpColVector measurementsNoisy = measurementsGT;
    unsigned int sizeMeasurement = measurementsGT.size();
    for (unsigned int i = 0; i < sizeMeasurement; ++i) {
      measurementsNoisy[i] += m_rng();
    }
    return measurementsNoisy;
  }
  //! [Noisy_measurements]

private:
  vpCameraParameters m_cam; // The camera parameters
  vpHomogeneousMatrix m_cMw; // The pose of the world frame with regard to the camera frame.
  vpRotationMatrix m_wRo; // The rotation matrix that expresses the rotation between the world frame and object frame.
  std::vector<vpColVector> m_markers; // The position of the markers in the object frame.
  double m_constantDenominator; // Denominator of the Gaussian function used for the likelihood computation.
  double m_constantExpDenominator; // Denominator of the exponential of the Gaussian function used for the likelihood computation.
  vpGaussRand m_rng; // Noise simulator for the measurements
};
//! [Markers_class]

//! [Pose_for_display]
/**
 * \brief Compute the pose from the 3D coordinates of the markers and their coordinates in pixels
 * in the image.
 *
 * \param[in] point The 3D coordinates of the markers in the object frame.
 * \param[in] ip The pixel coordinates of the markers in the image.
 * \param[in] cam The camera parameters used to acquire the image.
 * \return vpHomogeneousMatrix The pose of the object in the camera frame.
 */
vpHomogeneousMatrix computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &ip, const vpCameraParameters &cam)
{
  vpPose pose;
  double x = 0, y = 0;
  for (unsigned int i = 0; i < point.size(); i++) {
    vpPixelMeterConversion::convertPoint(cam, ip[i], x, y);
    point[i].set_x(x);
    point[i].set_y(y);
    pose.addPoint(point[i]);
  }

  vpHomogeneousMatrix cMo;
  pose.computePose(vpPose::DEMENTHON_LAGRANGE_VIRTUAL_VS, cMo);
  return cMo;
}
//! [Pose_for_display]

struct SoftwareArguments
{
  // --- Main loop parameters---
  static const int SOFTWARE_CONTINUE = 42;
  bool m_useDisplay; //!< If true, activate the plot and the renderer if VISP_HAVE_DISPLAY is defined.
  unsigned int m_nbStepsWarmUp; //!< Number of steps for the warmup phase.
  unsigned int m_nbSteps; //!< ?umber of steps for the main loop.
  // --- PF parameters---
  unsigned int m_N; //!< The number of particles.
  double m_maxDistanceForLikelihood; //!< The maximum allowed distance between a particle and the measurement, leading to a likelihood equal to 0..
  double m_ampliMaxX; //!< Amplitude max of the noise for the state component corresponding to the X coordinate.
  double m_ampliMaxY; //!< Amplitude max of the noise for the state component corresponding to the Y coordinate.
  double m_ampliMaxZ; //!< Amplitude max of the noise for the state component corresponding to the Z coordinate.
  double m_ampliMaxOmega; //!< Amplitude max of the noise for the state component corresponding to the pulsation.
  long m_seedPF; //!< Seed for the random generators of the PF.
  unsigned int m_nbThreads; //!< Number of thread to use in the Particle Filter.

  SoftwareArguments()
    : m_useDisplay(true)
    , m_nbStepsWarmUp(200)
    , m_nbSteps(300)
    , m_N(500)
    , m_maxDistanceForLikelihood(10.)
    , m_ampliMaxX(0.02)
    , m_ampliMaxY(0.02)
    , m_ampliMaxZ(0.01)
    , m_ampliMaxOmega(0.02)
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
      else if ((arg == "--ampli-max-Z") && ((i+1) < argc)) {
        m_ampliMaxZ = std::atof(argv[i + 1]);
        ++i;
      }
      else if ((arg == "--ampli-max-omega") && ((i+1) < argc)) {
        m_ampliMaxOmega = std::atof(argv[i + 1]);
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
    std::cout << "  [--max-distance-likelihood <double>] [-N, --nb-particles <uint>] [--seed <int>] [--nb-threads <int>]" << std::endl;
    std::cout << "  [--ampli-max-X <double>] [--ampli-max-Y <double>] [--ampli-max-Z <double>] [--ampli-max-omega <double>]" << std::endl;
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
    std::cout << "  --ampli-max-Z" << std::endl;
    std::cout << "    Maximum amplitude of the noise added to a particle along the Z-axis." << std::endl;
    std::cout << "    Default: " << m_ampliMaxZ << std::endl;
    std::cout << std::endl;
    std::cout << "  --ampli-max-omega" << std::endl;
    std::cout << "    Maximum amplitude of the noise added to a particle affecting the pulsation of the motion." << std::endl;
    std::cout << "    Default: " << m_ampliMaxOmega << std::endl;
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

  //! [Constants_for_simulation]
  const double dt = 0.001; // Period of 0.1s
  const double sigmaMeasurements = 2.; // Standard deviation of the measurements: 2 pixels
  const double radius = 0.25; // Radius of revolution of 0.25m
  const double w = 2 * M_PI * 10; // Pulsation of the motion of revolution
  const double phi = 2; // Phase of the motion of revolution
  const long seed = 4224; // Seed for the random generator
  const std::vector<vpColVector> markers = { vpColVector({-0.05, 0.05, 0., 1.})
                                           , vpColVector({0.05, 0.05, 0., 1.})
                                           , vpColVector({0.05, -0.05, 0., 1.})
                                           , vpColVector({-0.05, -0.05, 0., 1.}) }; // Vector of the markers sticked on the object
  const unsigned int nbMarkers = markers.size();
  std::vector<vpPoint> markersAsVpPoint;
  for (unsigned int i = 0; i < nbMarkers; ++i) {
    vpColVector marker = markers[i];
    markersAsVpPoint.push_back(vpPoint(marker[0], marker[1], marker[2]));
  }

  vpHomogeneousMatrix cMw; // Pose of the world frame with regard to the camera frame
  cMw[0][0] = 1.; cMw[0][1] = 0.; cMw[0][2] = 0.; cMw[0][3] = 0.2;
  cMw[1][0] = 0.; cMw[1][1] = -1.; cMw[1][2] = 0.; cMw[1][3] = 0.3;
  cMw[2][0] = 0.; cMw[2][1] = 0.; cMw[2][2] = -1.; cMw[2][3] = 1.;

  vpHomogeneousMatrix wMo; // Pose of the object frame with regard to the world frame
  wMo[0][0] = 1.; wMo[0][1] = 0.; wMo[0][2] = 0.; wMo[0][3] = radius;
  wMo[1][0] = 0.; wMo[1][1] = 1.; wMo[1][2] = 0.; wMo[1][3] = 0;
  wMo[2][0] = 0.; wMo[2][1] = 0.; wMo[2][2] = 1.; wMo[2][3] = 0.2;
  vpRotationMatrix wRo; // Rotation between the object frame and world frame
  wMo.extract(wRo);
  const double wZ = wMo[2][3];
  //! [Constants_for_simulation]

  //! [Camera_for_measurements]
  // Create a camera parameter container
  // Camera initialization with a perspective projection without distortion model
  const double px = 600., py = 600., u0 = 320., v0 = 240.;
  vpCameraParameters cam;
  cam.initPersProjWithoutDistortion(px, py, u0, v0);
  //! [Camera_for_measurements]

  // Initialize the attributes of the PF
  //! [Constants_for_the_PF]
  const double maxDistanceForLikelihood = args.m_maxDistanceForLikelihood; // The maximum allowed distance between a particle and the measurement, leading to a likelihood equal to 0..
  const double sigmaLikelihood = maxDistanceForLikelihood / 3.; // The standard deviation of likelihood function.
  const unsigned int nbParticles = args.m_N; // Number of particles to use
  const double ampliMaxX = args.m_ampliMaxX, ampliMaxY = args.m_ampliMaxY, ampliMaxZ = args.m_ampliMaxZ;
  const double ampliMaxOmega = args.m_ampliMaxOmega;
  const std::vector<double> stdevsPF = { ampliMaxX/3., ampliMaxY/3., ampliMaxZ/3., ampliMaxOmega/3. }; // Standard deviation for each state component
  const long seedPF = args.m_seedPF; // Seed for the random generators of the PF
  const unsigned int nbThread = args.m_nbThreads;
  //! [Constants_for_the_PF]

  //! [Initial_estimates]
  vpColVector X0(4U); // The initial guess for the state
  X0[0] = radius; // wX = radius m
  X0[1] = 0.; // wY = 0m
  X0[2] = 0.95 * wZ; // Wrong estimation of the position along the z-axis: error of 5%
  X0[3] = 0.95 * w * dt; // Wrong estimation of the pulsation: error of 5%
  //! [Initial_estimates]

  //! [Init_functions]
  vpParticleFilter<vpColVector>::vpProcessFunction processFunc = fx;
  vpMarkersMeasurements markerMeas(cam, cMw, wRo, markers, sigmaLikelihood, sigmaMeasurements, seed);
  using std::placeholders::_1;
  using std::placeholders::_2;
  vpParticleFilter<vpColVector>::vpLikelihoodFunction likelihoodFunc = std::bind(&vpMarkersMeasurements::likelihoodParticle, &markerMeas, _1, _2);
  vpParticleFilter<vpColVector>::vpResamplingConditionFunction checkResamplingFunc = vpParticleFilter<vpColVector>::simpleResamplingCheck;
  vpParticleFilter<vpColVector>::vpResamplingFunction resamplingFunc = vpParticleFilter<vpColVector>::simpleImportanceResampling;
  //! [Init_functions]

  //! [Init_PF]
  // Initialize the PF
  vpParticleFilter<vpColVector> filter(nbParticles, stdevsPF, seedPF, nbThread);
  filter.init(X0, processFunc, likelihoodFunc, checkResamplingFunc, resamplingFunc);
  //! [Init_PF]

  //! [Init_plot]
#ifdef VISP_HAVE_DISPLAY
  // Initialize the plot
  vpPlot *plot = nullptr;
  if (args.m_useDisplay) {
    plot = new vpPlot(1);
    plot->initGraph(0, 3);
    plot->setTitle(0, "Position of the robot wX");
    plot->setUnitX(0, "Position along x(m)");
    plot->setUnitY(0, "Position along y (m)");
    plot->setLegend(0, 0, "GT");
    plot->setLegend(0, 1, "Filtered");
    plot->setLegend(0, 2, "Measure");
    plot->initRange(0, -1.25 * radius, 1.25 * radius, -1.25 * radius, 1.25 * radius);
    plot->setColor(0, 0, vpColor::red);
    plot->setColor(0, 1, vpColor::blue);
    plot->setColor(0, 2, vpColor::black);
  }
#endif
  //! [Init_plot]

  //! [Init_renderer]
  // Initialize the display
  // Depending on the detected third party libraries, we instantiate here the
  // first video device which is available
#ifdef VISP_HAVE_DISPLAY
  std::shared_ptr<vpDisplay> d;
  vpImage<vpRGBa> Idisp(800, 800, vpRGBa(static_cast<unsigned char>(255)));
  if (args.m_useDisplay) {
    d = vpDisplayFactory::createDisplay(Idisp, 800, 50, "Projection of the markers");
  }
#endif
  //! [Init_renderer]

  //! [Init_simu]
  // Initialize the simulation
  vpObjectSimulator object(radius, w, phi, wZ);
  vpColVector object_pos(4U, 0.);
  object_pos[3] = 1.;
  //! [Init_simu]

  double averageFilteringTime = 0.;

  //! [Warmup_loop]
  const unsigned int nbStepsWarmUp = args.m_nbStepsWarmUp;
  for (unsigned int i = 0; i < nbStepsWarmUp; ++i) {
    // Update object pose
    object_pos = object.move(dt * static_cast<double>(i));

    // Perform the measurement
    vpColVector z = markerMeas.measureWithNoise(object_pos);

    // Use the UKF to filter the measurement
    double t0 = vpTime::measureTimeMicros();
    filter.filter(z, dt);
    averageFilteringTime += vpTime::measureTimeMicros() - t0;
  }
  //! [Warmup_loop]

  //! [Simu_loop]
  const unsigned int nbSteps = args.m_nbSteps;
  const double invNbSteps = 1. / static_cast<double>(nbSteps);
  double meanErrorFilter = 0.;
  double meanErrorNoise = 0.;

  for (unsigned int i = 0; i < nbSteps; ++i) {
    //! [Update obj pose]
    // Update object pose
    object_pos = object.move(dt * static_cast<double>(i));
    //! [Update obj pose]

    //! [Update_measurement]
    // Perform the measurement
    vpColVector z = markerMeas.measureWithNoise(object_pos);
    //! [Update_measurement]

    double t0 = vpTime::measureTimeMicros();
    //! [Perform_filtering]
    // Use the UKF to filter the measurement
    filter.filter(z, dt);
    //! [Perform_filtering]
    averageFilteringTime += vpTime::measureTimeMicros() - t0;

    //! [Get_filtered_state]
    vpColVector Xest = filter.computeFilteredState();
    //! [Get_filtered_state]

    //! [Noisy_pose]
    // Prepare the pose computation:
    // the image points corresponding to the noisy markers are needed
    std::vector<vpImagePoint> ip;
    for (unsigned int id = 0; id < nbMarkers; ++id) {
      vpImagePoint markerProjNoisy(z[2*id + 1], z[2*id]);
      ip.push_back(markerProjNoisy);
    }

    // Compute the pose using the noisy markers
    vpHomogeneousMatrix cMo_noisy = computePose(markersAsVpPoint, ip, cam);
    vpHomogeneousMatrix wMo_noisy = cMw.inverse() * cMo_noisy;
    double wXnoisy = wMo_noisy[0][3];
    double wYnoisy = wMo_noisy[1][3];
    //! [Noisy_pose]

  //! [Update_displays]
#ifdef VISP_HAVE_DISPLAY
    if (args.m_useDisplay) {
      //! [Update_plot]
      // Plot the ground truth
      plot->plot(0, 0, object_pos[0], object_pos[1]);

      // Plot the filtered state
      plot->plot(0, 1, Xest[0], Xest[1]);

      // Plot the noisy pose
      plot->plot(0, 2, wXnoisy, wYnoisy);
      //! [Update_plot]

      //! [Update_renderer]
      // Display the projection of the markers
      vpDisplay::display(Idisp);
      vpColVector zGT = markerMeas.measureGT(object_pos);
      vpColVector zFilt = markerMeas.state_to_measurement(Xest);
      for (unsigned int id = 0; id < nbMarkers; ++id) {
        vpImagePoint markerProjGT(zGT[2*id + 1], zGT[2*id]);
        vpDisplay::displayCross(Idisp, markerProjGT, 5, vpColor::red);

        vpImagePoint markerProjFilt(zFilt[2*id + 1], zFilt[2*id]);
        vpDisplay::displayCross(Idisp, markerProjFilt, 5, vpColor::blue);

        vpImagePoint markerProjNoisy(z[2*id + 1], z[2*id]);
        vpDisplay::displayCross(Idisp, markerProjNoisy, 5, vpColor::black);
      }

      vpImagePoint ipText(20, 20);
      vpDisplay::displayText(Idisp, ipText, std::string("GT"), vpColor::red);
      ipText.set_i(ipText.get_i() + 20);
      vpDisplay::displayText(Idisp, ipText, std::string("Filtered"), vpColor::blue);
      ipText.set_i(ipText.get_i() + 20);
      vpDisplay::displayText(Idisp, ipText, std::string("Measured"), vpColor::black);
      vpDisplay::flush(Idisp);
      vpTime::wait(40);
      //! [Update_renderer]
    }
#endif
    //! [Update_displays]

    //! [Compute_error]
    double error = std::sqrt(std::pow(Xest[0] - object_pos[0], 2) + std::pow(Xest[1] - object_pos[1], 2) + std::pow(Xest[2] - object_pos[2], 2));
    meanErrorFilter += invNbSteps * error;
    error = std::sqrt(std::pow(wMo_noisy[0][3] - object_pos[0], 2) + std::pow(wMo_noisy[1][3] - object_pos[1], 2) + std::pow(wMo_noisy[2][3] - object_pos[2], 2));
    meanErrorNoise += invNbSteps * error;
    //! [Compute_error]
  }
  //! [Simu_loop]

  averageFilteringTime = averageFilteringTime / (static_cast<double>(nbSteps) + static_cast<double>(nbStepsWarmUp));
  std::cout << "Mean error filter = " << meanErrorFilter << std::endl;
  std::cout << "Mean error noise = " << meanErrorNoise << std::endl;
  std::cout << "Mean filtering time = " << averageFilteringTime << "us" << std::endl;

  if (args.m_useDisplay) {
    std::cout << "Press Enter to quit..." << std::endl;
    std::cin.get();
  }

  //! [Delete_displays]
#ifdef VISP_HAVE_DISPLAY
  // Delete the plot if it was allocated
  if (plot != nullptr) {
    delete plot;
  }
#endif
  //! [Delete_displays]

  if (meanErrorFilter > meanErrorNoise) {
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cout << "This example is only available if you compile ViSP in C++11 standard or higher." << std::endl;
  return EXIT_SUCCESS;
}
#endif
