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

//! \example tutorial-pf-curve-fitting-all.cpp

// System includes
#include <algorithm>
#include <vector>

// ViSP includes
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpCannyEdgeDetection.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpMouseButton.h>
#include <visp3/core/vpTime.h>

#ifdef VISP_HAVE_DISPLAY
#include <visp3/gui/vpPlot.h>
#endif

//! [Include_PF]
#include <visp3/core/vpParticleFilter.h>
//! [Include_PF]

#include "vpTutoCommonData.h"
#include "vpTutoMeanSquareFitting.h"
#include "vpTutoParabolaModel.h"
#include "vpTutoSegmentation.h"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && defined(VISP_HAVE_DISPLAY)
#ifndef DOXYGEN_SHOULD_SKIP_THIS
namespace tutorial
{
//! [Evaluation_functions]
/**
 * \brief Compute the square error between the parabola model and
 * the input point \b pt.
 *
 * \param[in] pt The input point.
 * \return double The square error.
 */
double evaluate(const vpImagePoint &pt, const vpTutoParabolaModel &model)
{
  double u = pt.get_u();
  double v = pt.get_v();
  double v_model = model.eval(u);
  double error = v - v_model;
  double squareError = error * error;
  return squareError;
}

/**
 * \brief Compute the mean-square error between the parabola model and
 * the input points \b pts.
 *
 * \param[in] coeffs The coefficients of the polynomial.
 * \param[in] height The height of the input image.
 * \param[in] width The width of the input image.
 * \param[in] pts The input points.
 * \return double The root mean square error.
 */
double evaluate(const vpColVector &coeffs, const unsigned int &height, const unsigned int &width, const std::vector<vpImagePoint> &pts)
{
  unsigned int nbPts = pts.size();
  vpColVector residuals(nbPts);
  vpColVector weights(nbPts, 1.);
  vpTutoParabolaModel model(coeffs, height, width);
  // Compute the residuals
  for (unsigned int i = 0; i < nbPts; ++i) {
    double squareError = evaluate(pts[i], model);
    residuals[i] = squareError;
  }
  double meanSquareError = residuals.sum() / static_cast<double>(nbPts);
  return std::sqrt(meanSquareError);
}
//! [Evaluation_functions]

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
  vpTutoParabolaModel model(coeffs, I.getHeight(), I.getWidth());
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
 * \brief Select automatically the init points from the segmented image.
 *
 * \param[in] data The data common to the whole program.
 * \return std::vector<vpImagePoint> The vector of image points to use to initialize
 * the Particle Filter using a Least Mean Square minimization.
 */
std::vector<vpImagePoint> automaticInitialization(tutorial::vpTutoCommonData &data)
{
  // Initialization-related variables
  const unsigned int minNbPts = data.m_degree + 1;
  const unsigned int nbPtsToUse = 10 * minNbPts;
  std::vector<vpImagePoint> initPoints;

  // Perform HSV segmentation
  tutorial::performSegmentationHSV(data);

  // Extracting the skeleton of the mask
  std::vector<vpImagePoint> edgePoints = tutorial::extractSkeleton(data);
  unsigned int nbEdgePoints = edgePoints.size();

  if (nbEdgePoints < nbPtsToUse) {
    return edgePoints;
  }

  // Uniformly extract init points
  auto ptHasLowerU = [](const vpImagePoint &ptA, const vpImagePoint &ptB) {
    return ptA.get_u() < ptB.get_u();
    };
  std::sort(edgePoints.begin(), edgePoints.end(), ptHasLowerU);

  unsigned int idStart, idStop;
  if (nbEdgePoints > nbPtsToUse + 20) {
    // Avoid extreme points in case it's noise
    idStart = 10;
    idStop = edgePoints.size() - 10;
  }
  else {
    // We need to take all the points because we don't have enough
    idStart = 0;
    idStop = edgePoints.size();
  }

  // Sample uniformly the points starting from the left of the image to the right
  unsigned int sizeWindow = idStop - idStart + 1;
  unsigned int step = sizeWindow / (nbPtsToUse - 1);
  for (unsigned int id = idStart; id <= idStop; id += step) {
    initPoints.push_back(edgePoints[id]);
  }
  return initPoints;
}

/**
 * \brief Get the init points by user-interaction.
 *
 * \param[in] data The data common to the whole program.
 * \return std::vector<vpImagePoint> The vector that contains the init points.
 */
std::vector<vpImagePoint> manualInitialization(const tutorial::vpTutoCommonData &data)
{
  // Interaction variables
  const bool waitForClick = true;
  vpImagePoint ipClick;
  vpMouseButton::vpMouseButtonType button;

  // Display variables
  const unsigned int sizeCross = 10;
  const unsigned int thicknessCross = 2;
  const vpColor colorCross = vpColor::red;

  // Initialization-related variables
  const unsigned int minNbPts = data.m_degree + 1;
  std::vector<vpImagePoint> initPoints;

  bool notEnoughPoints = true;
  while (notEnoughPoints) {
    // Initial display of the images
    vpDisplay::display(data.m_I_orig);

    // Display the how-to
    vpDisplay::displayText(data.m_I_orig, data.m_ipLegend, "Left click to add init point (min.: " + std::to_string(minNbPts) + "), right click to estimate the initial coefficients of the Particle Filter.", data.m_colorLegend);
    vpDisplay::displayText(data.m_I_orig, data.m_ipLegend + data.m_legendOffset, "A middle click reinitialize the list of init points.", data.m_colorLegend);
    vpDisplay::displayText(data.m_I_orig, data.m_ipLegend + data.m_legendOffset + data.m_legendOffset, "If not enough points have been selected, a right click has no effect.", data.m_colorLegend);

    // Display the already selected points
    unsigned int nbInitPoints = initPoints.size();
    for (unsigned int i = 0; i < nbInitPoints; ++i) {
      vpDisplay::displayCross(data.m_I_orig, initPoints[i], sizeCross, colorCross, thicknessCross);
    }

    // Update the display
    vpDisplay::flush(data.m_I_orig);

    // Get the user input
    vpDisplay::getClick(data.m_I_orig, ipClick, button, waitForClick);

    // Either add the clicked point to the list of initial points or stop the loop if enough points are available
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

  return initPoints;
}

/**
 * \brief Compute the initial guess of the state for the Particle Filter
 * using Least-Mean-Square minimization.
 *
 * \param[in] data The data used in the tutorial.
 * \return vpColVector The vector containing the coefficients, used as initial guess,
 * of the parabola.
 */
vpColVector computeInitialGuess(tutorial::vpTutoCommonData &data)
{
  // Vector that contains the init points
  std::vector<vpImagePoint> initPoints;

#ifdef VISP_HAVE_DISPLAY
  // Interaction variables
  const bool waitForClick = true;
  vpImagePoint ipClick;
  vpMouseButton::vpMouseButtonType button;

  // Display variables
  const unsigned int sizeCross = 10;
  const unsigned int thicknessCross = 2;
  const vpColor colorCross = vpColor::red;

  bool automaticInit = false;

  // Initial display of the images
  vpDisplay::display(data.m_I_orig);
  vpDisplay::displayText(data.m_I_orig, data.m_ipLegend, "Left click to manually select the init points, right click to automatically initialize the PF", data.m_colorLegend);

  // Update the display
  vpDisplay::flush(data.m_I_orig);

  // Get the user input
  vpDisplay::getClick(data.m_I_orig, ipClick, button, waitForClick);

  // Either use the automatic initialization or the manual one depending on the user input
  switch (button) {
  case vpMouseButton::vpMouseButtonType::button1:
    automaticInit = false;
    break;
  case vpMouseButton::vpMouseButtonType::button3:
    automaticInit = true;
    break;
  default:
    break;
  }

  if (automaticInit) {
    // Get automatically the init points from the segmented image
    initPoints = tutorial::automaticInitialization(data);
  }
  else {
    // Get manually the init points from the original image
    initPoints = tutorial::manualInitialization(data);
  }

#else
  // Get the init points from the segmented image
  initPoints = tutorial::automaticInitialization(data);
#endif

  // Compute the coefficients of the parabola using Least-Mean-Square minimization.
  tutorial::vpTutoMeanSquareFitting lmsFitter(data.m_degree, data.m_I_orig.getHeight(), data.m_I_orig.getWidth());
  lmsFitter.fit(initPoints);
  vpColVector X0 = lmsFitter.getCoeffs();
  std::cout << "---[Initial fit]---" << std::endl;
  std::cout << lmsFitter.getModel();
  std::cout << "---[Initial fit]---" << std::endl;

  // Display info about the initialization
  vpDisplay::display(data.m_I_orig);
  vpDisplay::displayText(data.m_I_orig, data.m_ipLegend, "Here are the points selected for the initialization.", data.m_colorLegend);
  unsigned int nbInitPoints = initPoints.size();
  for (unsigned int i = 0; i < nbInitPoints; ++i) {
    const vpImagePoint &ip = initPoints[i];
    vpDisplay::displayCross(data.m_I_orig, ip, sizeCross, colorCross, thicknessCross);
  }

  // Update display and wait for click
  lmsFitter.display(data.m_I_orig, vpColor::red, data.m_ipLegend.get_v() + 2 * data.m_legendOffset.get_v(), data.m_ipLegend.get_u());
  vpDisplay::displayText(data.m_I_orig, data.m_ipLegend + data.m_legendOffset, "A click to continue.", data.m_colorLegend);
  vpDisplay::flush(data.m_I_orig);
  vpDisplay::getClick(data.m_I_orig, waitForClick);

  return X0;
}
//! [Initialization_function]

//! [Process_function]
vpColVector fx(const vpColVector &coeffs, const double &/*dt*/)
{
  vpColVector updatedCoeffs = coeffs; // We use a constant position model
  return updatedCoeffs;
}
//! [Process_function]

//! [Average_functor]
class vpTutoAverageFunctor
{
public:
  vpTutoAverageFunctor(const unsigned int &degree, const unsigned int &height, const unsigned int &width)
    : m_degree(degree)
    , m_height(height)
    , m_width(width)
  { }

  /**
   * \brief Compute the "weighted average" of polynomial models, by sampling control points and
   * then performing Least-Mean Square minimization to best fit the control points.
   *
   * \param[in] particles The vector containing the particles of the PF.
   * \param[in] weights Their associated weights
   *
   * \return vpColVector The coefficients of the polynomial model that best fits the control points.
   */
  vpColVector averagePolynomials(const std::vector<vpColVector> &particles, const std::vector<double> &weights, const vpParticleFilter<std::vector<vpImagePoint>>::vpStateAddFunction &/**/)
  {
    const unsigned int nbParticles = particles.size();
    const double nbParticlesAsDOuble = static_cast<double>(nbParticles);
    // Compute the sum of the weights to be able to determine the "importance" of a particle with regard to the whole set
    const double sumWeight = std::accumulate(weights.begin(), weights.end(), 0.);

    // Defining the total number of control points we want to generate
    const double nbPointsForAverage = 10. * nbParticlesAsDOuble;
    std::vector<vpImagePoint> initPoints;

    // Creating control points by each particle
    for (unsigned int i = 0; i < nbParticles; ++i) {
      // The number of control points a particle can generate is proportional to the ratio of its weight w.r.t. the sum of the weights
      double nbPoints = std::floor(weights[i] * nbPointsForAverage / sumWeight);
      if (nbPoints > 1.) {
        // The particle has a weight high enough to deserve more than one points
        vpTutoParabolaModel curve(particles[i], m_height, m_width);
        double widthAsDouble = static_cast<double>(m_width);
        // Uniform sampling of the control points along the polynomial model
        double step = widthAsDouble / (nbPoints - 1.);
        for (double u = 0.; u < widthAsDouble; u += step) {
          double v = curve.eval(u);
          vpImagePoint pt(v, u);
          initPoints.push_back(pt);
        }
      }
      else if (nbPoints == 1.) {
        // The weight of the particle make it have only one control point
        // We sample it at the middle of the image
        vpTutoParabolaModel curve(particles[i], m_height, m_width);
        double u = static_cast<double>(m_width) / 2.;
        double v = curve.eval(u);
        vpImagePoint pt(v, u);
        initPoints.push_back(pt);
      }
    }
    // We use Least-Mean Square minimization to compute the polynomial model that best fits all the control points
    vpTutoMeanSquareFitting lms(m_degree, m_height, m_width);
    lms.fit(initPoints);
    return lms.getCoeffs();
  }

private:
  unsigned int m_degree; //!< The degree of the polynomial.
  unsigned int m_height; //!< The height of the input image.
  unsigned int m_width; //!< The width of the input image.
};
//! [Average_functor]

//! [Likelihood_functor]
class vpTutoLikelihoodFunctor
{
public:
  /**
   * @brief Construct a new vp Likelihood Functor object
   *
   * \param[in] stdev The standard deviation of the likelihood function.
   * \param[in] height The height of the input image.
   * \param[in] width The width of the input image.
   */
  vpTutoLikelihoodFunctor(const double &stdev, const unsigned int &height, const unsigned int &width)
    : m_height(height)
    , m_width(width)
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
    unsigned int nbPoints = meas.size();

    // Generate a model from the coefficients stored in the particle state
    vpTutoParabolaModel model(coeffs, m_height, m_width);

    // Compute the residual between each measurement point and its equivalent in the model
    vpColVector residuals(nbPoints);
    for (unsigned int i = 0; i < nbPoints; ++i) {
      double squareError = tutorial::evaluate(meas[i], model);
      residuals[i] = squareError;
    }

    // Use Tukey M-estimator to be robust against outliers
    vpRobust Mestimator;
    vpColVector w(nbPoints, 1.);
    Mestimator.MEstimator(vpRobust::TUKEY, residuals, w);
    double sumError = w.hadamard(residuals).sum();

    // Compute the likelihood as a Gaussian function
    likelihood = std::exp(m_constantExpDenominator * sumError / w.sum()) * m_constantDenominator;
    likelihood = std::min(likelihood, 1.0); // Clamp to have likelihood <= 1.
    likelihood = std::max(likelihood, 0.); // Clamp to have likelihood >= 0.
    return likelihood;
  }
  //! [Likelihood_function]
private:
  double m_constantDenominator; //!< Denominator of the Gaussian function used for the likelihood computation.
  double m_constantExpDenominator; //!< Denominator of the exponential of the Gaussian function used for the likelihood computation.
  unsigned int m_height; //!< The height of the input image.
  unsigned int m_width; //!< The width of the input image.
};
//! [Likelihood_functor]
}
#endif

int main(const int argc, const char *argv[])
{
  tutorial::vpTutoCommonData data;
  int returnCode = data.init(argc, argv);
  if (returnCode != tutorial::vpTutoCommonData::SOFTWARE_CONTINUE) {
    return returnCode;
  }
  tutorial::vpTutoMeanSquareFitting lmsFitter(data.m_degree, data.m_I_orig.getHeight(), data.m_I_orig.getWidth());
  const unsigned int vertOffset = data.m_legendOffset.get_i();
  const unsigned int horOffset = data.m_ipLegend.get_j();
  const unsigned int legendLmsVert = data.m_I_orig.getHeight() - 3 * vertOffset;
  const unsigned int legendLmsHor = horOffset;
  const unsigned int legendPFVert = data.m_I_orig.getHeight() - 2 * vertOffset, legendPFHor = horOffset;

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
  vpParticleFilter<std::vector<vpImagePoint>>::vpProcessFunction processFunc = tutorial::fx;
  tutorial::vpTutoLikelihoodFunctor likelihoodFtor(sigmaLikelihood, data.m_I_orig.getHeight(), data.m_I_orig.getWidth());
  using std::placeholders::_1;
  using std::placeholders::_2;
  vpParticleFilter<std::vector<vpImagePoint>>::vpLikelihoodFunction likelihoodFunc = std::bind(&tutorial::vpTutoLikelihoodFunctor::likelihood, &likelihoodFtor, _1, _2);
  vpParticleFilter<std::vector<vpImagePoint>>::vpResamplingConditionFunction checkResamplingFunc = vpParticleFilter<std::vector<vpImagePoint>>::simpleResamplingCheck;
  vpParticleFilter<std::vector<vpImagePoint>>::vpResamplingFunction resamplingFunc = vpParticleFilter<std::vector<vpImagePoint>>::simpleImportanceResampling;
  tutorial::vpTutoAverageFunctor averageCpter(data.m_degree, data.m_I_orig.getHeight(), data.m_I_orig.getWidth());
  using std::placeholders::_3;
  vpParticleFilter<std::vector<vpImagePoint>>::vpFilterFunction meanFunc = std::bind(&tutorial::vpTutoAverageFunctor::averagePolynomials, &averageCpter, _1, _2, _3);
  //! [Init_functions]

  //! [Init_PF]
  // Initialize the PF
  vpParticleFilter<std::vector<vpImagePoint>> filter(nbParticles, stdevsPF, seedPF, nbThread);
  filter.init(X0, processFunc, likelihoodFunc, checkResamplingFunc, resamplingFunc, meanFunc);
  //! [Init_PF]

  //! [Init_plot]
#ifdef VISP_HAVE_DISPLAY
  unsigned int plotHeight = 350, plotWidth = 350;
  int plotXpos = data.m_legendOffset.get_u();
  int plotYpos = data.m_I_orig.getHeight() + 4. * data.m_legendOffset.get_v();
  vpPlot plot(1, plotHeight, plotWidth, plotXpos, plotYpos, "Root mean-square error");
  plot.initGraph(0, 2);
  plot.setLegend(0, 0, "LMS estimator");
  plot.setColor(0, 0, vpColor::gray);
  plot.setLegend(0, 1, "PF estimator");
  plot.setColor(0, 1, vpColor::red);
#endif
//! [Init_plot]

  bool run = true;
  unsigned int nbIter = 0;
  double  meanDtLMS = 0., meanDtPF = 0.;
  double  meanRootMeanSquareErrorLMS = 0., meanRootMeanSquareErrorPF = 0.;
  while (!data.m_grabber.end() && run) {
    std::cout << "Iter " << nbIter << std::endl;
    data.m_grabber.acquire(data.m_I_orig);

    tutorial::performSegmentationHSV(data);

    /// Extracting the skeleton of the mask
    std::vector<vpImagePoint> edgePoints = tutorial::extractSkeleton(data);

    /// Simulate sensor noise
    std::vector<vpImagePoint> noisyEdgePoints = tutorial::addSaltAndPepperNoise(edgePoints, data);

#ifdef VISP_HAVE_DISPLAY
    /// Initial display of the images
    vpDisplay::display(data.m_I_orig);
    vpDisplay::display(data.m_I_segmented);
    vpDisplay::display(data.m_IskeletonNoisy);
#endif

    /// Fit using least-square
    double tLms = vpTime::measureTimeMs();
    lmsFitter.fit(noisyEdgePoints);
    double dtLms = vpTime::measureTimeMs() - tLms;
    float lmsRootMeanSquareError = lmsFitter.evaluate(edgePoints);
    std::cout << "  [Least-Mean Square method] " << std::endl;
    std::cout << "    Coeffs = [" << lmsFitter.getCoeffs().transpose() << " ]" << std::endl;
    std::cout << "    Root Mean Square Error = " << lmsRootMeanSquareError << " pixels" << std::endl;
    std::cout << "    Fitting duration = " << dtLms << " ms" << std::endl;
    meanDtLMS += dtLms;
    meanRootMeanSquareErrorLMS += lmsRootMeanSquareError;

    /// Use the PF to filter the measurement
    double tPF = vpTime::measureTimeMs();
    //! [Perform_filtering]
    filter.filter(noisyEdgePoints, period);
    //! [Perform_filtering]
    double dtPF = vpTime::measureTimeMs() - tPF;

    //! [Get_filtered_state]
    vpColVector Xest = filter.computeFilteredState();
    //! [Get_filtered_state]

    //! [Evaluate_performances]
    float pfError = tutorial::evaluate(Xest, data.m_I_orig.getHeight(), data.m_I_orig.getWidth(), edgePoints);
    //! [Evaluate_performances]
    std::cout << "  [Particle Filter method] " << std::endl;
    std::cout << "    Coeffs = [" << Xest.transpose() << " ]" << std::endl;
    std::cout << "    Root Mean Square Error = " << pfError << " pixels" << std::endl;
    std::cout << "    Fitting duration = " << dtPF << " ms" << std::endl;
    meanDtPF += dtPF;
    meanRootMeanSquareErrorPF += pfError;

#ifdef VISP_HAVE_DISPLAY
    // Update image overlay
    lmsFitter.display<unsigned char>(data.m_IskeletonNoisy, vpColor::gray, legendLmsVert, legendLmsHor);
    tutorial::display(Xest, data.m_IskeletonNoisy, vpColor::red, legendPFVert, legendPFHor);

    // Update plot
    plot.plot(0, 0, nbIter, lmsRootMeanSquareError);
    plot.plot(0, 1, nbIter, pfError);
    // Display the images with overlayed info
    data.displayLegend(data.m_I_orig);
    vpDisplay::flush(data.m_I_orig);
    vpDisplay::flush(data.m_I_segmented);
    vpDisplay::flush(data.m_IskeletonNoisy);
    run = data.manageClicks(data.m_I_orig, data.m_stepbystep);
#endif
    ++nbIter;
  }

  double iterAsDouble = static_cast<double>(nbIter);

  std::cout << std::endl << std::endl << "-----[Statistics summary]-----" << std::endl;

  std::cout << "  [LMS method] " << std::endl;
  std::cout << "    Average Root Mean Square Error = " << meanRootMeanSquareErrorLMS / iterAsDouble << " pixels" << std::endl;
  std::cout << "    Average fitting duration = " << meanDtLMS / iterAsDouble << " ms" << std::endl;

  std::cout << "  [Particle Filter method] " << std::endl;
  std::cout << "    Average Root Mean Square Error = " << meanRootMeanSquareErrorPF / iterAsDouble << " pixels" << std::endl;
  std::cout << "    Average fitting duration = " << meanDtPF / iterAsDouble << " ms" << std::endl;

#ifdef VISP_HAVE_DISPLAY
  if (data.m_grabber.end() && (!data.m_stepbystep)) {
    /// Initial display of the images
    vpDisplay::display(data.m_I_orig);
    vpDisplay::displayText(data.m_I_orig, data.m_ipLegend, "End of sequence reached. Click to exit.", data.m_colorLegend);

    /// Update the display
    vpDisplay::flush(data.m_I_orig);

    /// Get the user input
    vpDisplay::getClick(data.m_I_orig, true);
  }
#endif
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
