/** \example tutorial-pf.cpp
 * Tutorial on how to use the Particle Filter (PF) on a complex non-linear use-case.
 * The system is an object, whose coordinate frame origin is the point O, on which are sticked four markers.
 * The object revolves in a plane parallel to the ground around a fixed point W whose coordinate frame is the world frame.
 * The scene is observed by a pinhole camera whose coordinate frame has the origin C and which is
 * fixed to the ceiling.
 *
 * The state vector of the PF is:
 * \f[
 * \begin{array}{lcl}
 *   \textbf{x}[0] &=& {}^WX_x \\
 *   \textbf{x}[1] &=& {}^WX_y \\
 *   \textbf{x}[2] &=& {}^WX_z \\
 *   \textbf{x}[3] &=& \omega \Delta t
 * \end{array}
 * \f]
 *
 * The measurement \f$ \textbf{z} \f$ corresponds to the coordinates in pixels of the different markers.
 * Be \f$ u_i \f$ and \f$ v_i \f$ the horizontal and vertical pixel coordinates of the \f$ i^{th} \f$ marker.
 * The measurement vector can be written as:
 * \f[
 *   \begin{array}{lcl}
 *       \textbf{z}[2i] &=& u_i \\
 *       \textbf{z}[2i+1] &=& v_i
 *   \end{array}
 * \f]
 *
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

//! [UKF_includes]
#include <visp3/core/vpUKSigmaDrawerMerwe.h>
#include <visp3/core/vpUnscentedKalman.h>
//! [UKF_includes]

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
  vpObjectSimulator(const double &R, const double &w, const double &phi, const double &wZ, const double &stdevRng)
    : m_R(R)
    , m_w(w)
    , m_phi(phi)
    , m_wZ(wZ)
    , m_rng(stdevRng, 0.)
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
    double tNoisy = (m_w + m_rng())* t + m_phi;
    wX[0] = m_R * std::cos(tNoisy);
    wX[1] = m_R * std::sin(tNoisy);
    wX[2] = m_wZ;
    return wX;
  }

private:
  double m_R; // Radius of the revolution around the world frame origin.
  double m_w; // Pulsation of the motion.
  double m_phi; // Phase of the motion.
  const double m_wZ; // The z-coordinate of the object in the world frame.
  vpGaussRand m_rng;
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
   * \param[in] noise_stdev The standard deviation for the noise generator
   * \param[in] seed The seed for the noise generator
   * \param[in] likelihood_stdev The standard deviation for the likelihood computation. A particle that is
   * 3. * likelihood_stdev further than the measurements will have a weight of 0.
   */
  vpMarkersMeasurements(const vpCameraParameters &cam, const vpHomogeneousMatrix &cMw, const vpRotationMatrix &wRo,
                        const std::vector<vpColVector> &markers, const double &noise_stdev, const long &seed,
                        const double &likelihood_stdev)
    : m_cam(cam)
    , m_cMw(cMw)
    , m_wRo(wRo)
    , m_markers(markers)
    , m_rng(noise_stdev, 0., seed)
  {
    double sigmaDistanceSquared = likelihood_stdev * likelihood_stdev;
    m_constantDenominator = 1. / std::sqrt(2. * M_PI * sigmaDistanceSquared);
    m_constantExpDenominator = -1. / (2. * sigmaDistanceSquared);

    const unsigned int nbMarkers = static_cast<unsigned int>(m_markers.size());
    for (unsigned int i = 0; i < nbMarkers; ++i) {
      vpColVector marker = markers[i];
      m_markersAsVpPoint.push_back(vpPoint(marker[0], marker[1], marker[2]));
    }
  }

  //! [Measurement_function]
  /**
   * \brief Convert the prior of the UKF into the measurement space.
   *
   * \param[in] x The prior.
   * \return vpColVector The prior expressed in the measurement space.
   */
  vpColVector state_to_measurement(const vpColVector &x)
  {
    unsigned int nbMarkers = static_cast<unsigned int>(m_markers.size());
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
  //! [Measurement_function]

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
    unsigned int nbMarkers = static_cast<unsigned int>(m_markers.size());
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

  //! [Likelihood_function]
  /**
   * \brief Compute the likelihood of a particle compared to the measurements.
   * The likelihood equals zero if the particle is completely different of
   * the measurements and equals one if it matches completely.
   * The chosen likelihood is a Gaussian function that penalizes the mean distance
   * between the projection of the markers corresponding to the particle position
   * and the measurements of the markers in the image.
   *
   * \param[in] coeffs The particle, which represent the parabola coefficients.
   * \param[in] meas The measurement vector.
   * \return double The likelihood of the particle.
   */
  double likelihood(const vpColVector &coeffs, const vpColVector &meas)
  {
    double likelihood = 0.;
    unsigned int nbMarkers = static_cast<unsigned int>(m_markers.size());
    double sumError = 0.;
    vpHomogeneousMatrix wMo;
    vpTranslationVector wTo(coeffs[0], coeffs[1], coeffs[2]);
    wMo.build(wTo, m_wRo);
    for (unsigned int i = 0; i < nbMarkers; ++i) {
      vpColVector cX = m_cMw * wMo * m_markers[i];
      double u = 0., v = 0.;
      vpMeterPixelConversion::convertPoint(m_cam, cX[0] / cX[2], cX[1] / cX[2], u, v);
      sumError += std::sqrt(std::pow(u - meas[2 * i], 2) + std::pow(v - meas[2 * i + 1], 2));
    }
    likelihood = std::exp(m_constantExpDenominator * sumError / nbMarkers) * m_constantDenominator;
    likelihood = std::min(likelihood, 1.0); // Clamp to have likelihood <= 1.
    likelihood = std::max(likelihood, 0.); // Clamp to have likelihood >= 0.
    return likelihood;
  }
  //! [Likelihood_function]
private:
  vpCameraParameters m_cam; // The camera parameters
  vpHomogeneousMatrix m_cMw; // The pose of the world frame with regard to the camera frame.
  vpRotationMatrix m_wRo; // The rotation matrix that expresses the rotation between the world frame and object frame.
  std::vector<vpColVector> m_markers; // The position of the markers in the object frame.
  std::vector<vpPoint> m_markersAsVpPoint; // The position of the markers in the object frame, expressed as vpPoint.
  vpGaussRand m_rng; // Noise simulator for the measurements
  double m_constantDenominator; // Denominator of the Gaussian function used for the likelihood computation.
  double m_constantExpDenominator; // Denominator of the exponential of the Gaussian function used for the likelihood computation.
};
//! [Markers_class]

int main(/*const int argc, const char *argv[]*/)
{
  //! [Constants_for_simulation]
  const unsigned int nbIter = 200; // Number of time steps for the simulation
  const double dt = 0.001; // Period of 0.1s
  const double sigmaMeasurements = 2.; // Standard deviation of the measurements: 2 pixels
  const double radius = 0.25; // Radius of revolution of 0.25m
  const double w = 2 * M_PI * 10; // Pulsation of the motion of revolution
  const double phi = 2; // Phase of the motion of revolution
  const std::vector<vpColVector> markers = { vpColVector({-0.05, 0.05, 0., 1.})
                                           , vpColVector({0.05, 0.05, 0., 1.})
                                           , vpColVector({0.05, -0.05, 0., 1.})
                                           , vpColVector({-0.05, -0.05, 0., 1.}) }; // Vector of the markers sticked on the object
  const unsigned int nbMarkers = static_cast<unsigned int>(markers.size());
  std::vector<vpPoint> markersAsVpPoint;
  for (unsigned int i = 0; i < nbMarkers; ++i) {
    vpColVector marker = markers[i];
    markersAsVpPoint.push_back(vpPoint(marker[0], marker[1], marker[2]));
  }

  const long seed = 42; // Seed for the random generator
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
  double px = 600; double py = 600; double u0 = 320; double v0 = 240;
  vpCameraParameters cam;
  cam.initPersProjWithoutDistortion(px, py, u0, v0);
  //! [Camera_for_measurements]

  // Initialize the attributes of the UKF
  //! [Sigma_points_drawer]
  std::shared_ptr<vpUKSigmaDrawerAbstract> drawer = std::make_shared<vpUKSigmaDrawerMerwe>(4, 0.001, 2., -1);
  //! [Sigma_points_drawer]

  //! [Covariance_measurements]
  vpMatrix R1landmark(2, 2, 0.); // The covariance of the noise introduced by the measurement with 1 landmark
  R1landmark[0][0] = sigmaMeasurements*sigmaMeasurements;
  R1landmark[1][1] = sigmaMeasurements*sigmaMeasurements;
  vpMatrix R(2*nbMarkers, 2 * nbMarkers);
  for (unsigned int i = 0; i < nbMarkers; ++i) {
    R.insert(R1landmark, 2*i, 2*i);
  }
  //! [Covariance_measurements]

  //! [Covariance_process]
  const double processVariance = 0.000025; // Variance of the process of (0.005cm)^2
  vpMatrix Q; // The covariance of the process
  Q.eye(4);
  Q = Q * processVariance;
  //! [Covariance_process]

  //! [Initial_estimates]
  vpMatrix P0(4, 4); //  The initial guess of the process covariance
  P0.eye(4);
  P0[0][0] = 1.;
  P0[1][1] = 1.;
  P0[2][2] = 1.;
  P0[2][2] = 5.;

  vpColVector X0(4); // The initial guess for the state
  X0[0] = 0.95 * radius * std::cos(phi); // Wrong estimation of the position along the X-axis = 5% of error
  X0[1] = 0.95 * radius * std::sin(phi); // Wrong estimation of the position along the Y-axis = 5% of error
  X0[2] = 0.95 * wZ; // Wrong estimation of the position along the Z-axis: error of 5%
  X0[3] = 0.95 * w * dt; // Wrong estimation of the pulsation: error of 25%
  //! [Initial_estimates]

  //! [Constants_for_the_PF]
  const double maxDistanceForLikelihood = 30; // The maximum allowed distance between a particle and the measurement, leading to a likelihood equal to 0..
  const double sigmaLikelihood = maxDistanceForLikelihood / 3.; // The standard deviation of likelihood function.
  const unsigned int nbParticles = 300; // Number of particles to use
  const double ampliMaxX = 0.05 * X0[0], ampliMaxY = 0.05 * X0[1], ampliMaxZ = 0.05 * X0[2];
  const double ampliMaxW = 0.05 * X0[3];
  const std::vector<double> stdevsPF = { ampliMaxX/3., ampliMaxY/3., ampliMaxZ/3., ampliMaxW / 3. }; // Standard deviation for each state component
  const unsigned long pfSeed = 4221;
  unsigned long seedPF; // Seed for the random generators of the PF
  if (pfSeed < 0) {
    seedPF = vpTime::measureTimeMicros();
  }
  else {
    seedPF = pfSeed;
  }
  const int nbThread = -1;
  //! [Constants_for_the_PF]

  //! [Init_functions_ukf]
  vpUnscentedKalman::vpProcessFunction f = fx;
  vpMarkersMeasurements markerMeas(cam, cMw, wRo, markers, sigmaMeasurements, seed, sigmaLikelihood);
  using std::placeholders::_1;
  vpUnscentedKalman::vpMeasurementFunction h = std::bind(&vpMarkersMeasurements::state_to_measurement, &markerMeas, _1);
  //! [Init_functions_ukf]

  //! [Init_UKF]
  // Initialize the UKF
  vpUnscentedKalman ukf(Q, R, drawer, f, h);
  ukf.init(X0, P0);
  //! [Init_UKF]

  //! [Init_functions_pf]
  vpParticleFilter<vpColVector>::vpProcessFunction processFunc = fx;
  // using std::placeholders::_1;
  using std::placeholders::_2;
  vpParticleFilter<vpColVector>::vpLikelihoodFunction likelihoodFunc = std::bind(&vpMarkersMeasurements::likelihood, &markerMeas, _1, _2);
  vpParticleFilter<vpColVector>::vpResamplingConditionFunction checkResamplingFunc = vpParticleFilter<vpColVector>::simpleResamplingCheck;
  vpParticleFilter<vpColVector>::vpResamplingFunction resamplingFunc = vpParticleFilter<vpColVector>::simpleImportanceResampling;
  //! [Init_functions_pf]

  //! [Init_PF]
  // Initialize the PF
  vpParticleFilter<vpColVector> pfFilter(nbParticles, stdevsPF, seedPF, nbThread);
  pfFilter.init(X0, processFunc, likelihoodFunc, checkResamplingFunc, resamplingFunc);
  //! [Init_PF]

  //! [Init_plot]
#ifdef VISP_HAVE_DISPLAY
  // Initialize the plot
  vpPlot plot(1);
  plot.initGraph(0, 4);
  plot.setTitle(0, "Position of the robot wX");
  plot.setUnitX(0, "Position along x(m)");
  plot.setUnitY(0, "Position along y (m)");
  plot.setLegend(0, 0, "GT");
  plot.setLegend(0, 1, "UKF");
  plot.setLegend(0, 2, "PF");
  plot.setLegend(0, 3, "Measure");
  plot.initRange(0, -1.25 * radius, 1.25 * radius, -1.25 * radius, 1.25 * radius);
  plot.setColor(0, 0, vpColor::red);
  plot.setColor(0, 1, vpColor::blue);
  plot.setColor(0, 2, vpColor::purple);
  plot.setColor(0, 3, vpColor::black);

  vpPlot plotError(1, 350, 700, 700, 700, "Error w.r.t. GT");
  plotError.initGraph(0, 3);
  plotError.setUnitX(0, "Time (s)");
  plotError.setUnitY(0, "Error (m)");
  plotError.setLegend(0, 0, "UKF");
  plotError.setLegend(0, 1, "PF");
  plotError.setLegend(0, 2, "Measure");
  plotError.initRange(0, 0, nbIter * dt, 0, radius / 2.);
  plotError.setColor(0, 0, vpColor::blue);
  plotError.setColor(0, 1, vpColor::purple);
  plotError.setColor(0, 2, vpColor::black);
#endif
  //! [Init_plot]

  //! [Init_renderer]
  // Initialize the display
  // Depending on the detected third party libraries, we instantiate here the
  // first video device which is available
#ifdef VISP_HAVE_DISPLAY
  vpImage<vpRGBa> Idisp(800, 800, vpRGBa(255));
  std::shared_ptr<vpDisplay> d = vpDisplayFactory::createDisplay(Idisp, 800, 50, "Projection of the markers");
#endif
  //! [Init_renderer]

  //! [Init_simu]
  // Initialize the simulation
  vpObjectSimulator object(radius, w, phi, wZ, ampliMaxW / 3.);
  vpColVector object_pos(4, 0.);
  object_pos[3] = 1.;
  //! [Init_simu]

  //! [Simu_loop]
  for (unsigned int i = 0; i < nbIter; ++i) {
    double t = dt * static_cast<double>(i);
    std::cout << "[Timestep" << i << ", t = " << t << "]" << std::endl;
    //! [Update obj pose]
    // Update object pose
    object_pos = object.move(t);
    //! [Update obj pose]

    //! [Update_measurement]
    // Perform the measurement
    vpColVector z = markerMeas.measureWithNoise(object_pos);
    //! [Update_measurement]

    //! [Perform_filtering]
    // Use the UKF to filter the measurement
    double tUKF = vpTime::measureTimeMs();
    ukf.filter(z, dt);
    double dtUKF = vpTime::measureTimeMs() - tUKF;
    /// Use the PF to filter the measurement
    double tPF = vpTime::measureTimeMs();
    pfFilter.filter(z, dt);
    double dtPF = vpTime::measureTimeMs() - tPF;
    //! [Perform_filtering]

    //! [Update_displays]
#ifdef VISP_HAVE_DISPLAY
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

    //! [Update_plot]
    // Plot the ground truth
    plot.plot(0, 0, object_pos[0], object_pos[1]);

    // Plot the UKF filtered state
    vpColVector XestUKF = ukf.getXest();
    plot.plot(0, 1, XestUKF[0], XestUKF[1]);

    // Plot the PF filtered state
    vpColVector XestPF = pfFilter.computeFilteredState();
    plot.plot(0, 2, XestPF[0], XestPF[1]);

    // Plot the noisy pose
    plot.plot(0, 3, wXnoisy, wYnoisy);

    vpColVector cX_GT = cMw * object_pos;
    vpColVector wX_UKF(4, 1.);
    vpColVector wX_PF(4, 1.);
    for (unsigned int i = 0; i < 3; ++i) {
      wX_PF[i] = XestPF[i];
      wX_UKF[i] = XestUKF[i];
    }
    vpColVector cX_PF = cMw * wX_PF;
    vpColVector cX_UKF = cMw * wX_UKF;
    vpColVector error_PF = cX_PF - cX_GT;
    vpColVector error_UKF = cX_UKF - cX_GT;

    std::cout << "  [Unscented Kalman Filter method] " << std::endl;
    std::cout << "    Mean square error = " << error_UKF.frobeniusNorm() << " m^2" << std::endl;
    std::cout << "    Fitting duration = " << dtUKF << " ms" << std::endl;
    std::cout << "  [Particle Filter method] " << std::endl;
    std::cout << "    Mean square error = " << error_PF.frobeniusNorm() << " m^2" << std::endl;
    std::cout << "    Fitting duration = " << dtPF << " ms" << std::endl;

    // Plot the UKF filtered state error
    plotError.plot(0, 0, t, error_UKF.frobeniusNorm());

    // Plot the PF filtered state error
    plotError.plot(0, 1, t, error_PF.frobeniusNorm());

    // Plot the noisy error
    plotError.plot(0, 2, t, (cMo_noisy.getTranslationVector() - vpTranslationVector(cX_GT.extract(0, 3))).frobeniusNorm());
    //! [Update_plot]

    //! [Update_renderer]
    // Display the projection of the markers
    vpDisplay::display(Idisp);
    vpColVector zGT = markerMeas.measureGT(object_pos);
    vpColVector zFiltUkf = markerMeas.state_to_measurement(XestUKF);
    vpColVector zFiltPF = markerMeas.state_to_measurement(XestPF);
    for (unsigned int id = 0; id < nbMarkers; ++id) {
      vpImagePoint markerProjGT(zGT[2*id + 1], zGT[2*id]);
      vpDisplay::displayCross(Idisp, markerProjGT, 5, vpColor::red);

      vpImagePoint markerProjFiltUKF(zFiltUkf[2*id + 1], zFiltUkf[2*id]);
      vpDisplay::displayCross(Idisp, markerProjFiltUKF, 5, vpColor::blue);

      vpImagePoint markerProjFiltPF(zFiltPF[2*id + 1], zFiltPF[2*id]);
      vpDisplay::displayCross(Idisp, markerProjFiltPF, 5, vpColor::purple);

      vpImagePoint markerProjNoisy(z[2*id + 1], z[2*id]);
      vpDisplay::displayCross(Idisp, markerProjNoisy, 5, vpColor::black);
    }

    vpImagePoint ipText(20, 20);
    vpDisplay::displayText(Idisp, ipText, std::string("GT"), vpColor::red);
    ipText.set_i(ipText.get_i() + 20);
    vpDisplay::displayText(Idisp, ipText, std::string("Filtered by UKF"), vpColor::blue);
    ipText.set_i(ipText.get_i() + 20);
    vpDisplay::displayText(Idisp, ipText, std::string("Filtered by PF"), vpColor::purple);
    ipText.set_i(ipText.get_i() + 20);
    vpDisplay::displayText(Idisp, ipText, std::string("Measured"), vpColor::black);
    vpDisplay::flush(Idisp);
    vpTime::wait(40);
    //! [Update_renderer]
#endif
    //! [Update_displays]
  }
  //! [Simu_loop]
  std::cout << "Press Enter to quit..." << std::endl;
  std::cin.get();

  return 0;
}
#else
int main()
{
  std::cout << "This example is only available if you compile ViSP in C++11 standard or higher." << std::endl;
  return 0;
}
#endif
