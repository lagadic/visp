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

//! \example tutorial-pf.cpp

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpCannyEdgeDetection.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpMouseButton.h>
#include <visp3/core/vpTime.h>


//! [Include_PF]
#include <visp3/core/vpParticleFilter.h>
//! [Include_PF]

#include "vpTutoCommonData.h"
#include "vpTutoMeanSquareFitting.h"
#include "vpTutoParabolaModel.h"
#include "vpTutoRANSACFitting.h"
#include "vpTutoSegmentation.h"

#ifdef ENABLE_VISP_NAMESPACE
using VISP_NAMESPACE_NAME;
#endif

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && defined(VISP_HAVE_DISPLAY)
namespace tutorial
{
//! [Evaluation_functions]
/**
 * \brief Compute the square error between the parabola model and
 * the input point \b pt.
 *
 * \param[in] pt The input point.
 * \return float The square error.
 */
float evaluate(const vpImagePoint &pt, const vpTutoParabolaModel &model)
{
  float u = pt.get_u();
  float v = pt.get_v();
  float v_model = model.eval(u);
  float error = v - v_model;
  float squareError = error * error;
  return squareError;
}

/**
 * \brief Compute the mean-square error between the parabola model and
 * the input points \b pts. An M-estimator is used to reject outliers
 * when computing the mean square error.
 *
 * \param[in] pts The input points.
 * \return float The mean square error.
 */
float evaluate(const vpColVector &coeffs, const std::vector<vpImagePoint> &pts)
{
  unsigned int nbPts = pts.size();
  vpColVector residuals(nbPts);
  vpColVector weights(nbPts, 1.);
  vpTutoParabolaModel model(coeffs);
  // Compute the residuals
  for (unsigned int i = 0; i < nbPts; ++i) {
    float squareError = evaluate(pts[i], model);
    residuals[i] = squareError;
  }
  float meanError = residuals.sum() / static_cast<double>(nbPts);
  return meanError;
}

/**
 * \brief Compute the mean-square error between the parabola model and
 * the input points \b pts. An M-estimator is used to reject outliers
 * when computing the mean square error.
 *
 * \param[in] pts The input points.
 * \return float The mean square error.
 */
float evaluateRobust(const vpColVector &coeffs, const std::vector<vpImagePoint> &pts)
{
  unsigned int nbPts = pts.size();
  vpColVector residuals(nbPts);
  vpColVector weights(nbPts, 1.);
  vpTutoParabolaModel model(coeffs);
  // Compute the residuals
  for (unsigned int i = 0; i < nbPts; ++i) {
    float squareError = evaluate(pts[i], model);
    residuals[i] = squareError;
  }
  vpRobust robust;
  robust.MEstimator(vpRobust::TUKEY, residuals, weights);
  float sumWeights = weights.sum();
  float numerator = (weights.hadamard(residuals)).sum();
  float meanError = numerator / sumWeights;
  return meanError;
}
//! [Evaluation_functions]

//! [Init_from_file]
/**
 * \brief Read the initialization points from a file.
 *
 * \param[in] listPointsFile The name of the file containing the list
 * of initialization points.
 * \return std::vector<vpImagePoint> The vector of image points to use to initialize
 * the Particle Filter using a Least Mean Square minimization.
 */
std::vector<vpImagePoint> readInitPointsFromFile(const std::string &listPointsFile)
{
  std::vector<vpImagePoint> initPoints;
  std::ifstream myfile(listPointsFile);
  std::string line;
  if (myfile.is_open()) {
    while (std::getline(myfile, line)) {
      std::size_t nbCharICoord = line.find_first_of(' ');
      std::string iAsString = line.substr(0, nbCharICoord);
      std::string jAsString = line.substr(nbCharICoord + 1);
      float i = std::atof(iAsString.c_str());
      float j = std::atof(jAsString.c_str());
      vpImagePoint ip(i, j);
      initPoints.push_back(ip);
    }
    myfile.close();
  }
  return initPoints;
}
//! [Init_from_file]

//! [Display_function]
/**
   * \brief Display the fitted parabola on the image.
   *
   * \tparam T Either unsigned char or vpRGBa.
   * \param[in] coeffs The coefficients of the parabola, such as coeffs[0] = a coeffs[1] = b coeffs[2] = c
   * \param[in] I The image on which we want to display the parabola model.
   * \param[in] color The color we want to use to display the parabola.
   */
template<typename T>
void display(const vpColVector &coeffs, const vpImage<T> &I, const vpColor &color,
             const unsigned int &vertPosLegend, const unsigned int &horPosLegend)
{
#if defined(VISP_HAVE_DISPLAY)
  unsigned int width = I.getWidth();
  vpTutoParabolaModel model(coeffs);
  for (unsigned int u = 0; u < width; ++u) {
    float v = model.eval(u);
    vpDisplay::displayPoint(I, v, u, color, 1);
    vpDisplay::displayText(I, vertPosLegend, horPosLegend, "Particle Filter model", color);
  }
#else
  (void)coeffs;
  (void)I;
  (void)color;
  (void)vertPosLegend;
  (void)horPosLegend;
#endif
}
//! [Display_function]

//! [Initialization_function]
/**
 * \brief Compute the initial guess of the state for the Particle Filter
 * using Least-Mean-Square minimization.
 *
 * \param[in] data The data used in the tutorial.
 * \return vpColVector The vector containing the coefficients, used as initial guess,
 * of the parabola.
 */
vpColVector computeInitialGuess(const tutorial::vpTutoCommonData &data)
{
  const std::string listPointsFile("list_init_points.txt");
#ifdef VISP_HAVE_DISPLAY
  const unsigned int minNbPts = 3;
  const unsigned int sizeCross = 10;
  const unsigned int thicknessCross = 2;
  const vpColor colorCross = vpColor::red;
  const bool waitForClick = true;
  std::vector<vpImagePoint> initPoints;
  bool notEnoughPoints = true;
  vpImagePoint ipClick;
  vpMouseButton::vpMouseButtonType button;

  bool useFile = false;
  if (vpIoTools::checkFilename(listPointsFile)) {
    /// Initial display of the images
    vpDisplay::display(data.m_I_orig);
    vpDisplay::displayText(data.m_I_orig, data.m_ipLegend, "Left click to manually select the init points, right click to the points from the file \"" + listPointsFile + "\"", data.m_colorLegend);

    /// Update the display
    vpDisplay::flush(data.m_I_orig);

    /// Get the user input
    vpDisplay::getClick(data.m_I_orig, ipClick, button, waitForClick);

    /// Either add the clicked point to the list of initial points or stop the loop if enough points are available
    switch (button) {
    case vpMouseButton::vpMouseButtonType::button1:
      useFile = false;
      break;
    case vpMouseButton::vpMouseButtonType::button3:
      useFile = true;
      break;
    default:
      break;
    }
  }

  if (useFile) {
    /// Read the initialization points from a file.
    initPoints = tutorial::readInitPointsFromFile(listPointsFile);
  }
  else {
    while (notEnoughPoints) {
      /// Initial display of the images
      vpDisplay::display(data.m_I_orig);

      /// Display the how-to
      vpDisplay::displayText(data.m_I_orig, data.m_ipLegend, "Left click to add init point (min.: 3), right click to estimate the initial coefficients of the Particle Filter.", data.m_colorLegend);
      vpDisplay::displayText(data.m_I_orig, data.m_ipLegend + data.m_legendOffset, "A middle click reinitialize the list of init points.", data.m_colorLegend);
      vpDisplay::displayText(data.m_I_orig, data.m_ipLegend + data.m_legendOffset + data.m_legendOffset, "If not enough points have been selected, a right click has no effect.", data.m_colorLegend);

      /// Display the already selected points
      unsigned int nbInitPoints = initPoints.size();
      for (unsigned int i = 0; i < nbInitPoints; ++i) {
        vpDisplay::displayCross(data.m_I_orig, initPoints[i], sizeCross, colorCross, thicknessCross);
      }

      /// Update the display
      vpDisplay::flush(data.m_I_orig);

      /// Get the user input
      vpDisplay::getClick(data.m_I_orig, ipClick, button, true);

      /// Either add the clicked point to the list of initial points or stop the loop if enough points are available
      switch (button) {
      case vpMouseButton::vpMouseButtonType::button1:
        initPoints.push_back(ipClick);
        break;
      case vpMouseButton::vpMouseButtonType::button2:
        initPoints.clear();
        break;
      case vpMouseButton::vpMouseButtonType::button3:
        (initPoints.size() >= minNbPts ? notEnoughPoints = false : notEnoughPoints = true);
        break;
      default:
        break;
      }
    }
  }

  /// Compute the coefficients of the parabola using Least-Mean-Square minimization.
  tutorial::vpTutoMeanSquareFitting lmsFitter(data.m_degree);
  lmsFitter.fit(initPoints);
  vpColVector X0 = lmsFitter.getCoeffs();
  std::cout << "---[Initial fit]---" << std::endl;
  std::cout << lmsFitter.getModel();
  std::cout << "---[Initial fit]---" << std::endl;

  /// Display info about the initialization
  vpDisplay::display(data.m_I_orig);
  vpDisplay::displayText(data.m_I_orig, data.m_ipLegend, "Here are the points selected for the initialization.", data.m_colorLegend);
  unsigned int nbInitPoints = initPoints.size();
  for (unsigned int i = 0; i < nbInitPoints; ++i) {
    const vpImagePoint &ip = initPoints[i];
    vpDisplay::displayCross(data.m_I_orig, ip, sizeCross, colorCross, thicknessCross);
  }
  /// Save the init points if they were not read from a file
  if (!useFile) {
    std::ofstream ofs_initPoints(listPointsFile);
    for (unsigned int i = 0; i < nbInitPoints; ++i) {
      const vpImagePoint &ip = initPoints[i];
      ofs_initPoints << ip.get_i() << " " << ip.get_j() << std::endl;
    }
    ofs_initPoints.close();
  }
  lmsFitter.display(data.m_I_orig, vpColor::red, data.m_ipLegend.get_v() + 2 * data.m_legendOffset.get_v(), data.m_ipLegend.get_u());
  vpDisplay::displayText(data.m_I_orig, data.m_ipLegend + data.m_legendOffset, "A click to continue.", data.m_colorLegend);
  vpDisplay::flush(data.m_I_orig);
  vpDisplay::getClick(data.m_I_orig, waitForClick);

  return X0;
#else
  if (vpIoTools::checkFilename(listPointsFile)) {
    std::vector<vpImagePoint> initPoints = tutorial::readInitPointsFromFile(listPointsFile);
    tutorial::vpTutoMeanSquareFitting lmsFitter(data.m_degree);
    lmsFitter.fit(initPoints);
    vpColVector X0 = lmsFitter.getCoeffs();
    std::cout << "Initial coefficients = " << X0.t() << std::endl;
    return X0;
  }
  else {
    throw(vpException(vpException::fatalError, "A display is required to select the initial points"));
  }
#endif
}
//! [Initialization_function]

//! [Process_function]
vpColVector fx(const vpColVector &coeffs, const double &/*dt*/)
{
  vpColVector updatedCoeffs = coeffs; // We use a constant position model
  return updatedCoeffs;
}
//! [Process_function]

//! [Likelihood_functor]
class vpLikelihoodFunctor
{
public:
  vpLikelihoodFunctor(const double &stdev) : m_stdev(stdev)
  {
    double sigmaDistanceSquared = stdev * stdev;
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
   * \param[in] coeffs The particle, which represent the parabola coefficients.
   * \param[in] meas The measurement vector.
   * \return double The likelihood of the particle.
   */
  double likelihood(const vpColVector &coeffs, const std::vector<vpImagePoint> &meas)
  {
    double likelihood = 0.;
    double sumError = 0.;
    unsigned int nbPoints = meas.size();
    vpTutoParabolaModel model(coeffs);
    for (unsigned int i = 0; i < nbPoints; ++i) {
      double squareError = tutorial::evaluate(meas[i], model);
      sumError += squareError;
    }
    likelihood = std::exp(m_constantExpDenominator * sumError / static_cast<double>(nbPoints)) * m_constantDenominator;
    likelihood = std::min(likelihood, 1.0); // Clamp to have likelihood <= 1.
    likelihood = std::max(likelihood, 0.); // Clamp to have likelihood >= 0.
    return likelihood;
  }
  //! [Likelihood_function]
private:
  double m_stdev;
  double m_constantDenominator; // Denominator of the Gaussian function used for the likelihood computation.
  double m_constantExpDenominator; // Denominator of the exponential of the Gaussian function used for the likelihood computation.
};
//! [Likelihood_functor]
}

int main(const int argc, const char *argv[])
{
  tutorial::vpTutoCommonData data;
  int returnCode = data.init(argc, argv);
  if (returnCode != tutorial::vpTutoCommonData::SOFTWARE_CONTINUE) {
    return returnCode;
  }
  tutorial::vpTutoMeanSquareFitting lmsFitter(data.m_degree);
  tutorial::vpTutoRANSACFitting ransacFitter(data.m_ransacN, data.m_ransacK, data.m_ransacThresh, data.m_ransacRatioInliers, data.m_degree);
  const unsigned int vertOffset = data.m_legendOffset.get_i();
  const unsigned int horOffset = data.m_ipLegend.get_j();
  const unsigned int legendLmsVert = data.m_I_orig.getHeight() - 4 * vertOffset;
  const unsigned int legendLmsHor = horOffset;
  const unsigned int legendRansacVert = data.m_I_orig.getHeight() - 3 * vertOffset;
  const unsigned int legendRansacHor = horOffset;
  const unsigned int legendPFVert = data.m_I_orig.getHeight() - 2 * vertOffset, legendPFHor = horOffset;
  unsigned int nbIter = 0;

  // Initialize the attributes of the PF
  //! [Initial_estimates]
  vpColVector X0 = tutorial::computeInitialGuess(data);
  //! [Initial_estimates]

  //! [Constants_for_the_PF]
  const double maxDistanceForLikelihood = data.m_pfMaxDistanceForLikelihood; // The maximum allowed distance between a particle and the measurement, leading to a likelihood equal to 0..
  const double sigmaLikelihood = maxDistanceForLikelihood / 3.; // The standard deviation of likelihood function.
  const unsigned int nbParticles = data.m_pfN; // Number of particles to use
  std::vector<double> stdevsPF; // Standard deviation for each state component
  for (unsigned int i = 0; i < data.m_degree + 1; ++i) {
    double ampliMax = data.m_pfRatiosAmpliMax[i] * X0[i];
    stdevsPF.push_back(ampliMax / 3.);
  }
  unsigned long seedPF; // Seed for the random generators of the PF
  const float period = 33.3; // 33.3ms i.e. 30Hz
  if (data.m_pfSeed < 0) {
    seedPF = vpTime::measureTimeMicros();
  }
  else {
    seedPF = data.m_pfSeed;
  }
  const int nbThread = data.m_pfNbThreads;
  //! [Constants_for_the_PF]

  //! [Init_functions]
  vpParticleFilter<vpColVector>::vpProcessFunction processFunc = tutorial::fx;
  tutorial::vpLikelihoodFunctor likelihoodFtor(sigmaLikelihood);
  using std::placeholders::_1;
  using std::placeholders::_2;
  vpParticleFilter<std::vector<vpImagePoint>>::vpLikelihoodFunction likelihoodFunc = std::bind(&tutorial::vpLikelihoodFunctor::likelihood, &likelihoodFtor, _1, _2);
  vpParticleFilter<std::vector<vpImagePoint>>::vpResamplingConditionFunction checkResamplingFunc = vpParticleFilter<std::vector<vpImagePoint>>::simpleResamplingCheck;
  vpParticleFilter<std::vector<vpImagePoint>>::vpResamplingFunction resamplingFunc = vpParticleFilter<std::vector<vpImagePoint>>::simpleImportanceResampling;
  //! [Init_functions]

  //! [Init_PF]
  // Initialize the PF
  vpParticleFilter<std::vector<vpImagePoint>> filter(nbParticles, stdevsPF, seedPF, nbThread);
  filter.init(X0, processFunc, likelihoodFunc, checkResamplingFunc, resamplingFunc);
  //! [Init_PF]

  bool run = true;
  while ((!data.m_grabber.end() || data.m_useSimulated) && run) {
    std::cout << "Iter " << nbIter << std::endl;
    if (!data.m_useSimulated) {
      data.m_grabber.acquire(data.m_I_orig);
    }
    tutorial::performSegmentationHSV(data);

    /// Extracting the skeleton of the mask
    std::vector<vpImagePoint> edgePoints = tutorial::extractSkeletton(data);

#ifdef VISP_HAVE_DISPLAY
    /// Initial display of the images
    vpDisplay::display(data.m_I_orig);
    vpDisplay::display(data.m_I_segmented);
    vpDisplay::display(data.m_Iskeleton);
#endif

    /// Fit using least-square
    double tLms = vpTime::measureTimeMs();
    lmsFitter.fit(edgePoints);
    double dtLms = vpTime::measureTimeMs() - tLms;
    float lmsError = lmsFitter.evaluate(edgePoints);
    std::cout << "  [Least-Mean Square method] " << std::endl;
    std::cout << "    Coeffs = [" << lmsFitter.getCoeffs().transpose() << " ]" << std::endl;
    std::cout << "    Mean square error = " << lmsError << " pixels^2" << std::endl;
    std::cout << "    Fitting duration = " << dtLms << " ms" << std::endl;
    lmsFitter.display<unsigned char>(data.m_Iskeleton, vpColor::blue, legendLmsVert, legendLmsHor);

    /// Fit using RANSAC
    double tRansac = vpTime::measureTimeMs();
    ransacFitter.fit(edgePoints);
    double dtRansac = vpTime::measureTimeMs() - tRansac;
    float ransacError = ransacFitter.evaluate(edgePoints);
    std::cout << "  [RANSAC method] " << std::endl;
    std::cout << "    Coeffs = [" << ransacFitter.getModel().getCoeffs().transpose() << " ]" << std::endl;
    std::cout << "    Mean square error = " << ransacError << " pixels^2" << std::endl;
    std::cout << "    Fitting duration = " << dtRansac << " ms" << std::endl;
    ransacFitter.display<unsigned char>(data.m_Iskeleton, vpColor::red, legendRansacVert, legendRansacHor);

    /// Use the UKF to filter the measurement
    double tPF = vpTime::measureTimeMs();
    //! [Perform_filtering]
    filter.filter(edgePoints, period);
    //! [Perform_filtering]
    double dtPF = vpTime::measureTimeMs() - tPF;

    //! [Get_filtered_state]
    vpColVector Xest = filter.computeFilteredState();
    //! [Get_filtered_state]

    //! [Evaluate_performances]
    float pfError = tutorial::evaluate(Xest, edgePoints);
    //! [Evaluate_performances]
    tutorial::display(Xest, data.m_Iskeleton, vpColor::gray, legendPFVert, legendPFHor);
    std::cout << "  [Particle Filter method] " << std::endl;
    std::cout << "    Coeffs = [" << Xest.transpose() << " ]" << std::endl;
    std::cout << "    Mean square error = " << pfError << " pixels^2" << std::endl;
    std::cout << "    Fitting duration = " << dtPF << " ms" << std::endl;

#ifdef VISP_HAVE_DISPLAY
    // Display the images with overlayed info
    data.displayLegend(data.m_I_orig);
    vpDisplay::flush(data.m_I_orig);
    vpDisplay::flush(data.m_I_segmented);
    vpDisplay::flush(data.m_Iskeleton);
    run = data.manageClicks(data.m_I_orig, data.m_stepbystep);
#endif
    ++nbIter;
  }

  if (data.m_grabber.end() && (!data.m_stepbystep)) {
    /// Initial display of the images
    vpDisplay::display(data.m_I_orig);
    vpDisplay::displayText(data.m_I_orig, data.m_ipLegend, "End of sequence reached. Click to exit.", data.m_colorLegend);

    /// Update the display
    vpDisplay::flush(data.m_I_orig);

    /// Get the user input
    vpDisplay::getClick(data.m_I_orig, true);
  }
  return 0;
}
#else
int main()
{
  std::cerr << "ViSP must be compiled with C++ standard >= C++11 to use this tutorial." << std::endl;
  std::cerr << "ViSP must also have a 3rd party enabling display features, such as X11 or OpenCV." << std::endl;
  return EXIT_FAILURE;
}
#endif
