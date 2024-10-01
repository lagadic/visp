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
 * Test Particle Filter functionalities.
 */

/*!
  \example testParticleFilter.cpp

  Test some vpParticleFilter functionalities.
  The aim is to fit a polynomial to input data.
*/
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
#include <limits>
#include <vector>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpParticleFilter.h>
#include <visp3/core/vpUniRand.h>

#ifdef VISP_HAVE_DISPLAY
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayFactory.h>
#endif

#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
bool opt_display = false; //!< If true, activate debug display

/*!
 * \brief Model of a parabola \f[v = \sum_{i = 0}^N a_i u^i \f] where \f[N\f] is the
 * degree of the polynomial.
 */
class vpParabolaModel
{
public:
  inline vpParabolaModel(const unsigned int &degree, const unsigned int &height, const unsigned int &width)
    : m_degree(degree)
    , m_height(height)
    , m_width(width)
    , m_coeffs(degree + 1, 0.)
  { }

  /**
   * \brief Construct a new vpParabolaModel object
   *
   * \param[in] coeffs The coefficients of the polynomial, where coeffs[0] = offset
   * and coeffs[m_degree] is the coefficient applied to the highest degree input.
   * \param[in] height The height of the input image.
   * \param[in] width The width of the input image.
   */
  inline vpParabolaModel(const vpColVector &coeffs, const unsigned int &height, const unsigned int &width)
    : m_degree(coeffs.size() - 1)
    , m_height(height)
    , m_width(width)
    , m_coeffs(coeffs)
  { }

  /**
   * \brief Construct a new vpParabolaModel object
   *
   * \param[in] coeffs The coefficients of the polynomial, where coeffs[0][0] = offset
   * and coeffs[m_degree][0] is the coefficient applied to the highest degree input.
   * \param[in] height The height of the input image.
   * \param[in] width The width of the input image.
   */
  inline vpParabolaModel(const vpMatrix &coeffs, const unsigned int &height, const unsigned int &width)
    : m_degree(coeffs.getRows() - 1)
    , m_height(height)
    , m_width(width)
    , m_coeffs(coeffs.getCol(0))
  { }

  inline vpParabolaModel(const vpParabolaModel &other)
  {
    *this = other;
  }

  /**
   * @brief Compute \f[v = \sum_{i = 0}^N a_i u^i \f]
   *
   * \param[in] u Input
   * \return float The corresponding v.
   */
  inline double eval(const double &u) const
  {
    double normalizedU = u / m_width;
    double v = 0.;
    for (unsigned int i = 0; i <= m_degree; ++i) {
      v += m_coeffs[i] * std::pow(normalizedU, i);
    }
    v *= m_height;
    return v;
  }

  /**
   * \brief Cast into a vpColVector
   *
   * \return coeffs a:=coeffs[0] b:=coeffs[1] c:=coeffs[2]
   */
  inline  vpColVector toVpColVector() const
  {
    return m_coeffs;
  }

  inline vpParabolaModel &operator=(const vpParabolaModel &other)
  {
    m_degree = other.m_degree;
    m_height = other.m_height;
    m_width = other.m_width;
    m_coeffs = other.m_coeffs;
    return *this;
  }

  /**
   * @brief Fill the matrices that form the linear system A X = b
   * where A contains the different powers of the u-coordinates,
   * X contains the model coefficients and b contains the
   * v-coordinates.
   *
   * \param[in] degree The highest degree of the polynomial.
   * \param[in] height The height of the input image.
   * \param[in] width The width of the input image.
   * \param[in] pts The points to use to interpolate the coefficients of the parabola.
   * \param[out] A The matrix that contains the different powers of the u-coordinates.
   * \param[out] b The matrix that contains the v-coordinates.
   * \return Fill
   */
  static void fillSystem(const unsigned int &degree, const unsigned int &height, const unsigned int&width, const std::vector< vpImagePoint> &pts, vpMatrix &A, vpMatrix &b)
  {
    const unsigned int nbPts = static_cast<unsigned int>(pts.size());
    const unsigned int nbCoeffs = degree + 1;
    std::vector<vpImagePoint> normalizedPts;
    for (const auto &pt: pts) {
      normalizedPts.push_back(vpImagePoint(pt.get_i() / height, pt.get_j() / width));
    }
    A.resize(nbPts, nbCoeffs, 1.);
    b.resize(nbPts, 1);
    for (unsigned int i = 0; i < nbPts; ++i) {
      double u = normalizedPts[i].get_u();
      double v = normalizedPts[i].get_v();
      for (unsigned int j = 0; j < nbCoeffs; ++j) {
        A[i][j] = std::pow(u, j);
      }
      b[i][0] = v;
    }
  }

#ifdef VISP_HAVE_DISPLAY
  /**
   * \brief Display the fitted parabola on the image.
   *
   * \tparam T Either unsigned char or vpRGBa.
   * \param[in] I The image on which we want to display the parabola model.
   * \param[in] color The color we want to use to display the parabola.
   */
  template<typename T>
  void display(const vpImage<T> &I, const vpColor &color, const std::string &legend,
               const unsigned int &vertPosLegend, const unsigned int &horPosLegend)
  {
    unsigned int width = I.getWidth();
    for (unsigned int u = 0; u < width; ++u) {
      int v = static_cast<int>(eval(u));
      vpDisplay::displayPoint(I, v, u, color, 1);
      vpDisplay::displayText(I, vertPosLegend, horPosLegend, legend, color);
    }
  }
#endif

private:
  unsigned int m_degree; /*!< The highest degree of the polynomial.*/
  unsigned int m_height; /*!< The height of the input image*/
  unsigned int m_width; /*!< The width of the input image*/
  vpColVector m_coeffs; /*!< The coefficient of the polynomial, where m_coeffs[0] is the offset and m_coeffs[m_degree] is the coefficient applied to the highest degree.*/
};

/**
 * \brief Compute the coefficients of the 2nd degree curve for the simulated data.
 * The polynomial is written as y = a x^2 + b x + c.
 *
 * \param[in] x0 Horizontal coordinate of the inflection point.
 * \param[in] y0 Vertical coordinate of the inflection point.
 * \param[in] x1 Horizontal coordinate of another point of the curve.
 * \param[in] y1 Vertical coordinate of another point of the curve.
 * \return vpColVector The coefficients such as v[0] = c ; v[1] = b ; v[2] = a
 */
vpColVector computeABC(const double &x0, const double &y0, const double &x1, const double &y1)
{
  double b = (y1 - y0)/(-0.5*(x1 * x1/x0) + x1 -0.5 * x0);
  double a = -b / (2. * x0);
  double c = y0 - 0.5 * b * x0;
  return vpColVector({ c, b, a });
}

/**
 * \brief Compute the coefficients of the 2nd degree curve for the simulated data.
 * The polynomial is written as y = a x^3 + b x^2 + c x + d.
 *
 * \param[in] x0 Horizontal coordinate of the inflection point.
 * \param[in] y0 Vertical coordinate of the inflection point.
 * \param[in] x1 Horizontal coordinate of another point of the curve.
 * \param[in] y1 Vertical coordinate of another point of the curve.
 * \return vpColVector The coefficients such as v[0] = d ; v[1] = c ; v[2] = b ; v[3] = a
 */
vpColVector computeABCD(const double &x0, const double &y0, const double &x1, const double &y1)
{
  double factorA = -2. / (3. * (x1 + x0));
  double factorC = -1. * ((-2. * std::pow(x0, 2))/(x1 + x0) + 2 * x0);
  double b = (y1 - y0)/(factorA * (std::pow(x1, 3) - std::pow(x0, 3)) + (std::pow(x1, 2) - std::pow(x0, 2)) + (x1 - x0) * factorC);
  double a = factorA * b;
  double c = factorC * b;
  double d = y0-(a * std::pow(x0, 3) + b * std::pow(x0, 2) + c * x0);
  return vpColVector({ d, c, b, a });
}

/**
 * \brief Compute the v-coordinate of an image point based on the u-coordinate and the polynomial used
 * for the simulation.
 * \param[in] x The u-coordinate of the image point.
 * \param[in] coeffs The coefficients of the polynomial.
 * \return The corresponding v-coordinate.
 */
double computeY(const double &x, const vpColVector &coeffs)
{
  double y = 0.;
  unsigned int nbCoeffs = coeffs.size();
  for (unsigned int i = 0; i < nbCoeffs; ++i) {
    y += coeffs[i] * std::pow(x, i);
  }
  return y;
}

/**
 * \brief Generate the polynomial points corresponding to the polynomial.
 *
 * \param[in] xmin The lowest x-coordinate to use to generate the data.
 * \param[in] xmax The highest x-coordinate to use to generate the data.
 * \param[in] step The step between the x-coordinates to use.
 * \param[in] coeffs The coefficients of the polynomial.
 * \return std::vector<vpImagePoint> Vector of points that represent the polynomial.
 */
std::vector<vpImagePoint> generateSimulatedImage(const double &xmin, const double &xmax, const double &step, const vpColVector &coeffs)
{
  std::vector<vpImagePoint> points;
  for (double x = xmin; x <= xmax; x += step) {
    double y = computeY(x, coeffs);
    vpImagePoint pt(y, x);
    points.push_back(pt);
  }
  return points;
}

#ifdef VISP_HAVE_DISPLAY
template<typename T>
void displayGeneratedImage(const vpImage<T> &I, const std::vector<vpImagePoint> &pts, const vpColor &color,
                           const std::string &legend, const unsigned int &vertOffset, const unsigned int &horOffset)
{
  unsigned int nbPts = static_cast<unsigned int>(pts.size());
  for (unsigned int i = 1; i < nbPts; ++i) {
    vpDisplay::displayPoint(I, pts[i], color, 1);
  }
  vpDisplay::displayText(I, vertOffset, horOffset, legend, color);
}
#endif

/**
 * \brief Compute the initial guess of the state for the Particle Filter initialization.
 *
 * \param[in] pts The points to use for the initialization of the Particle Filter.
 * \param[in] degree The degree of the polynomial to fit.
 * \param[in] height The maximum y-coordinate.
 * \param[in] width The maximum x-coordinate.
 * \return vpParabolaModel The fitter model.
 */
vpParabolaModel computeInitialGuess(const std::vector<vpImagePoint> &pts, const unsigned int &degree, const unsigned int&height, const unsigned int &width)
{
  vpMatrix A; // The matrix that contains the u^2, u and 1s
  vpMatrix X; // The matrix we want to estimate, that contains the a, b and c coefficients.
  vpMatrix b; // The matrix that contains the v values

  // Fill the matrices that form the system we want to solve
  vpParabolaModel::fillSystem(degree, height, width, pts, A, b);

  // Compute the parabola coefficients using the least-mean-square method.
  X = A.pseudoInverse() * b;
  vpParabolaModel model(X, height, width);
  return model;
}

/**
 * \brief Compute the square error between the parabola model and
 * the input points \b pts.
 *
 * \param[in] pts The input points.
 * \return double The square error.
 */
double evaluate(const std::vector<vpImagePoint> &pts, const vpParabolaModel &model)
{
  double rmse = 0.;
  size_t sizePts = pts.size();
  for (size_t i = 0; i < sizePts; ++i) {
    const vpImagePoint &pt = pts[i];
    double u = pt.get_u();
    double v = pt.get_v();
    double v_model = model.eval(u);
    double error = v - v_model;
    double squareError = error * error;
    rmse += squareError;
  }
  rmse = std::sqrt(rmse / static_cast<double>(pts.size()));
  return rmse;
}

/**
 * \brief Process function, we use a constant model.
 *
 * \param[in] coeffs The particle, that represents the polynomial coefficients.
 */
vpColVector fx(const vpColVector &coeffs, const double &/*dt*/)
{
  vpColVector updatedCoeffs = coeffs;
  return updatedCoeffs;
}

class vpAverageFunctor
{
public:
  vpAverageFunctor(const unsigned int &degree, const unsigned int &height, const unsigned int &width)
    : m_degree(degree)
    , m_height(height)
    , m_width(width)
  { }

  vpColVector averagePolynomials(const std::vector<vpColVector> &particles, const std::vector<double> &weights, const vpParticleFilter<std::vector<vpImagePoint>>::vpStateAddFunction &/**/)
  {
    const unsigned int nbParticles = static_cast<unsigned int>(particles.size());
    const double nbParticlesAsDOuble = static_cast<double>(nbParticles);
    const double sumWeight = std::accumulate(weights.begin(), weights.end(), 0.);
    const double nbPointsForAverage = 10. * nbParticlesAsDOuble;
    std::vector<vpImagePoint> initPoints;
    for (unsigned int i = 0; i < nbParticles; ++i) {
      double nbPoints = std::floor(weights[i] * nbPointsForAverage / sumWeight);
      if (nbPoints > 1.) {
        vpParabolaModel curve(particles[i], m_height, m_width);
        double widthAsDouble = static_cast<double>(m_width);
        double step = widthAsDouble / (nbPoints - 1.);
        for (double u = 0.; u < widthAsDouble; u += step) {
          double v = curve.eval(u);
          vpImagePoint pt(v, u);
          initPoints.push_back(pt);
        }
      }
      else if (nbPoints == 1.) {
        vpParabolaModel curve(particles[i], m_height, m_width);
        double u = static_cast<double>(m_width) / 2.;
        double v = curve.eval(u);
        vpImagePoint pt(v, u);
        initPoints.push_back(pt);
      }
    }
    vpMatrix A, X, b;
    vpParabolaModel::fillSystem(m_degree, m_height, m_width, initPoints, A, b);
    X = A.pseudoInverse() * b;
    return vpParabolaModel(X, m_height, m_width).toVpColVector();
  }

private:
  unsigned int m_degree; //!< The degree of the polynomial.
  unsigned int m_height; //!< The height of the input image.
  unsigned int m_width; //!< The width of the input image.
};

class vpLikelihoodFunctor
{
public:
  /**
   * @brief Construct a new vp Likelihood Functor object
   *
   * \param[in] stdev The standard deviation of the likelihood function.
   * \param[in] height The height of the input image.
   * \param[in] width The width of the input image.
   */
  vpLikelihoodFunctor(const double &stdev, const unsigned int &height, const unsigned int &width)
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
    unsigned int nbPoints = static_cast<unsigned int>(meas.size());
    vpParabolaModel model(coeffs, m_height, m_width);
    vpColVector residuals(nbPoints);
    double rmse = evaluate(meas, model);
    likelihood = std::exp(rmse * m_constantExpDenominator) * m_constantDenominator;
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
}

TEST_CASE("2nd-degree", "[vpParticleFilter][Polynomial interpolation]")
{
  /// ----- Simulation parameters -----
  const unsigned int width = 600; //!< The width of the simulated image
  const unsigned int height = 400; //!< The height of the simulated image
  const unsigned int degree = 2; //!< The degree of the polynomial in the simulated image
  const unsigned int nbInitPoints = 10; //!< Number of points to compute the initial guess of the PF state
  const uint64_t seedCurve = 4224; //!< The seed to generate the curve
  const uint64_t seedInitPoints = 2112; //!< The seed to choose the init points
  const unsigned int nbTestRepet = 10; //!< The number of times the test is repeated
  const unsigned int nbWarmUpIter = 10; //!< Number of iterations for the warmup loop
  const unsigned int nbEvalIter = 20; //!< Number of iterations for the evaluation loop
  const double dt = 0.040; //!< Simulated period of acquisition
  const int32_t seedShuffle = 4221; //!< The seed to shuffle the curve points

  /// ----- PF parameters -----
  // The maximum amplitude for the likelihood compute.
  // A particle whose "distance" with the measurements is greater than this value has a likelihood of 0
  const double ampliMaxLikelihood = 16.;
  const double sigmaLikelihood = ampliMaxLikelihood / 3.; //:< The corresponding standard deviation
  const unsigned int nbParticles = 300; //!< Number of particles used by the particle filter
  const double ratioAmpliMax(0.25); //!< Ratio of the initial guess values to use to add noise to the PF state
  const long seedPF = 4221; //!< Seed of the particle filter
  const int nbThreads = 1; //<! Number of threads to use for the PF
  vpUniRand rngCurvePoints(seedCurve);
  vpUniRand rngInitPoints(seedInitPoints);

  SECTION("Noise-free", "The init points are directly extracted from the curve points, without any additional noise")
  {
    const double maxToleratedError = 5.;
    double x0 = rngCurvePoints.uniform(0., width);
    double x1 = rngCurvePoints.uniform(0., width);
    double y0 = rngCurvePoints.uniform(0., height);
    double y1 = rngCurvePoints.uniform(0., height);
    vpColVector coeffs = computeABC(x0, y0, x1, y1);
    std::vector<vpImagePoint> curvePoints = generateSimulatedImage(0, width, 1., coeffs);

#ifdef VISP_HAVE_DISPLAY
    vpImage<vpRGBa> I(height, width);
    std::shared_ptr<vpDisplay> pDisplay;
    if (opt_display) {
      pDisplay = vpDisplayFactory::createDisplay(I);
    }
#endif

    for (unsigned int iter = 0; iter < nbTestRepet; ++iter) {
      // Randomly select the initialization points
      std::vector<vpImagePoint> suffledVector = vpUniRand::shuffleVector(curvePoints, seedShuffle);
      std::vector<vpImagePoint> initPoints;
      for (unsigned int j = 0; j < nbInitPoints; ++j) {
        initPoints.push_back(suffledVector[j]);
      }

      // Compute the initial model
      vpParabolaModel modelInitial = computeInitialGuess(initPoints, degree, height, width);
      vpColVector X0 = modelInitial.toVpColVector();

      // Initialize the Particle Filter
      std::vector<double> stdevsPF;
      for (unsigned int i = 0; i < degree + 1; ++i) {
        stdevsPF.push_back(ratioAmpliMax * X0[0] / 3.);
      }
      vpParticleFilter<std::vector<vpImagePoint>>::vpProcessFunction processFunc = fx;
      vpLikelihoodFunctor likelihoodFtor(sigmaLikelihood, height, width);
      using std::placeholders::_1;
      using std::placeholders::_2;
      vpParticleFilter<std::vector<vpImagePoint>>::vpLikelihoodFunction likelihoodFunc = std::bind(&vpLikelihoodFunctor::likelihood, &likelihoodFtor, _1, _2);
      vpParticleFilter<std::vector<vpImagePoint>>::vpResamplingConditionFunction checkResamplingFunc = vpParticleFilter<std::vector<vpImagePoint>>::simpleResamplingCheck;
      vpParticleFilter<std::vector<vpImagePoint>>::vpResamplingFunction resamplingFunc = vpParticleFilter<std::vector<vpImagePoint>>::simpleImportanceResampling;
      vpAverageFunctor averageCpter(degree, height, width);
      using std::placeholders::_3;
      vpParticleFilter<std::vector<vpImagePoint>>::vpFilterFunction meanFunc = std::bind(&vpAverageFunctor::averagePolynomials, &averageCpter, _1, _2, _3);
      vpParticleFilter<std::vector<vpImagePoint>> filter(nbParticles, stdevsPF, seedPF, nbThreads);
      filter.init(X0, processFunc, likelihoodFunc, checkResamplingFunc, resamplingFunc, meanFunc);

      for (unsigned int i = 0; i < nbWarmUpIter; ++i) {
        filter.filter(curvePoints, dt);
      }

      double meanError = 0.;
      for (unsigned int i = 0; i < nbEvalIter; ++i) {
        filter.filter(curvePoints, dt);
        vpColVector Xest = filter.computeFilteredState();
        vpParabolaModel model(Xest, height, width);
        double rmse = evaluate(curvePoints, model);
        meanError += rmse;

#ifdef VISP_HAVE_DISPLAY
        if (opt_display) {
          vpDisplay::display(I);
          displayGeneratedImage(I, curvePoints, vpColor::red, "GT", 20, 20);
          model.display(I, vpColor::blue, "Model", 40, 20);
          vpDisplay::flush(I);
          vpDisplay::getClick(I);
        }
#endif
      }
      meanError /= static_cast<double>(nbEvalIter);
      std::cout << "Mean(rmse) = " << meanError << std::endl;
      CHECK(meanError <= maxToleratedError);
    }
  }

  SECTION("Noisy", "Noise is added to the init points")
  {
    const double maxToleratedError = 12.;
    double x0 = rngCurvePoints.uniform(0., width);
    double x1 = rngCurvePoints.uniform(0., width);
    double y0 = rngCurvePoints.uniform(0., height);
    double y1 = rngCurvePoints.uniform(0., height);
    vpColVector coeffs = computeABC(x0, y0, x1, y1);
    std::vector<vpImagePoint> curvePoints = generateSimulatedImage(0, width, 1., coeffs);

#ifdef VISP_HAVE_DISPLAY
    vpImage<vpRGBa> I(height, width);
    std::shared_ptr<vpDisplay> pDisplay;
    if (opt_display) {
      pDisplay = vpDisplayFactory::createDisplay(I);
    }
#endif

    const double ampliMaxInitNoise = 24.;
    const double stdevInitNoise = ampliMaxInitNoise / 3.;
    vpGaussRand rngInitNoise(stdevInitNoise, 0., seedInitPoints);

    for (unsigned int iter = 0; iter < nbTestRepet; ++iter) {
      // Randomly select the initialization points
      std::vector<vpImagePoint> suffledVector = vpUniRand::shuffleVector(curvePoints, seedShuffle);
      std::vector<vpImagePoint> initPoints;
      for (unsigned int j = 0; j < nbInitPoints; ++j) {
        vpImagePoint noisyPt(suffledVector[j].get_i() + rngInitNoise(), suffledVector[j].get_j() + rngInitNoise());
        initPoints.push_back(noisyPt);
      }

      // Compute the initial model
      vpParabolaModel modelInitial = computeInitialGuess(initPoints, degree, height, width);
      vpColVector X0 = modelInitial.toVpColVector();

      // Initialize the Particle Filter
      std::vector<double> stdevsPF;
      for (unsigned int i = 0; i < degree + 1; ++i) {
        stdevsPF.push_back(ratioAmpliMax * X0[0] / 3.);
      }
      vpParticleFilter<std::vector<vpImagePoint>>::vpProcessFunction processFunc = fx;
      vpLikelihoodFunctor likelihoodFtor(sigmaLikelihood, height, width);
      using std::placeholders::_1;
      using std::placeholders::_2;
      vpParticleFilter<std::vector<vpImagePoint>>::vpLikelihoodFunction likelihoodFunc = std::bind(&vpLikelihoodFunctor::likelihood, &likelihoodFtor, _1, _2);
      vpParticleFilter<std::vector<vpImagePoint>>::vpResamplingConditionFunction checkResamplingFunc = vpParticleFilter<std::vector<vpImagePoint>>::simpleResamplingCheck;
      vpParticleFilter<std::vector<vpImagePoint>>::vpResamplingFunction resamplingFunc = vpParticleFilter<std::vector<vpImagePoint>>::simpleImportanceResampling;
      vpAverageFunctor averageCpter(degree, height, width);
      using std::placeholders::_3;
      vpParticleFilter<std::vector<vpImagePoint>>::vpFilterFunction meanFunc = std::bind(&vpAverageFunctor::averagePolynomials, &averageCpter, _1, _2, _3);
      vpParticleFilter<std::vector<vpImagePoint>> filter(nbParticles, stdevsPF, seedPF, nbThreads);
      filter.init(X0, processFunc, likelihoodFunc, checkResamplingFunc, resamplingFunc, meanFunc);

      for (unsigned int i = 0; i < nbWarmUpIter; ++i) {
        filter.filter(curvePoints, dt);
      }

      double meanError = 0.;
      for (unsigned int i = 0; i < nbEvalIter; ++i) {
        filter.filter(curvePoints, dt);
        vpColVector Xest = filter.computeFilteredState();
        vpParabolaModel model(Xest, height, width);
        double rmse = evaluate(curvePoints, model);
        meanError += rmse;

#ifdef VISP_HAVE_DISPLAY
        if (opt_display) {
          vpDisplay::display(I);
          displayGeneratedImage(I, curvePoints, vpColor::red, "GT", 20, 20);
          model.display(I, vpColor::blue, "Model", 40, 20);
          vpDisplay::flush(I);
          vpDisplay::getClick(I);
        }
#endif
      }
      meanError /= static_cast<double>(nbEvalIter);
      std::cout << "Mean(rmse) = " << meanError << std::endl;
      CHECK(meanError <= maxToleratedError);
    }
  }
}

TEST_CASE("3rd-degree", "[vpParticleFilter][Polynomial interpolation]")
{
/// ----- Simulation parameters -----
  const unsigned int width = 600; //!< The width of the simulated image
  const unsigned int height = 400; //!< The height of the simulated image
  const unsigned int degree = 3; //!< The degree of the polynomial in the simulated image
  const unsigned int nbInitPoints = 10; //!< Number of points to compute the initial guess of the PF state
  const uint64_t seedCurve = 4224; //!< The seed to generate the curve
  const uint64_t seedInitPoints = 2112; //!< The seed to choose the init points
  const unsigned int nbTestRepet = 10; //!< The number of times the test is repeated
  const unsigned int nbWarmUpIter = 10; //!< Number of iterations for the warmup loop
  const unsigned int nbEvalIter = 20; //!< Number of iterations for the evaluation loop
  const double dt = 0.040; //!< Simulated period of acquisition
  const int32_t seedShuffle = 4221; //!< The seed to shuffle the curve points

  /// ----- PF parameters -----
  // The maximum amplitude for the likelihood compute.
  // A particle whose "distance" with the measurements is greater than this value has a likelihood of 0
  const double ampliMaxLikelihood = 6.;
  const double sigmaLikelihood = ampliMaxLikelihood / 3.; //:< The corresponding standard deviation
  const unsigned int nbParticles = 300; //!< Number of particles used by the particle filter
  const double ratioAmpliMax(0.21); //!< Ratio of the initial guess values to use to add noise to the PF state
  const long seedPF = 4221; //!< Seed of the particle filter
  const int nbThreads = 1; //<! Number of threads to use for the PF
  vpUniRand rngCurvePoints(seedCurve);
  vpUniRand rngInitPoints(seedInitPoints);

  SECTION("Noise-free", "The init points are directly extracted from the curve points, without any additional noise")
  {
    const double maxToleratedError = 10.;
    double x0 = rngCurvePoints.uniform(0., width);
    double x1 = rngCurvePoints.uniform(0., width);
    double y0 = rngCurvePoints.uniform(0., height);
    double y1 = rngCurvePoints.uniform(0., height);
    vpColVector coeffs = computeABCD(x0, y0, x1, y1);
    std::vector<vpImagePoint> curvePoints = generateSimulatedImage(0, width, 1., coeffs);

#ifdef VISP_HAVE_DISPLAY
    vpImage<vpRGBa> I(height, width);
    std::shared_ptr<vpDisplay> pDisplay;
    if (opt_display) {
      pDisplay = vpDisplayFactory::createDisplay(I);
    }
#endif

    for (unsigned int iter = 0; iter < nbTestRepet; ++iter) {
      // Randomly select the initialization points
      std::vector<vpImagePoint> suffledVector = vpUniRand::shuffleVector(curvePoints, seedShuffle);
      std::vector<vpImagePoint> initPoints;
      for (unsigned int j = 0; j < nbInitPoints; ++j) {
        initPoints.push_back(suffledVector[j]);
      }

      // Compute the initial model
      vpParabolaModel modelInitial = computeInitialGuess(initPoints, degree, height, width);
      vpColVector X0 = modelInitial.toVpColVector();

      // Initialize the Particle Filter
      std::vector<double> stdevsPF;
      for (unsigned int i = 0; i < degree + 1; ++i) {
        stdevsPF.push_back(ratioAmpliMax * std::pow(0.1, i) * X0[0] / 3.);
      }
      vpParticleFilter<std::vector<vpImagePoint>>::vpProcessFunction processFunc = fx;
      vpLikelihoodFunctor likelihoodFtor(sigmaLikelihood, height, width);
      using std::placeholders::_1;
      using std::placeholders::_2;
      vpParticleFilter<std::vector<vpImagePoint>>::vpLikelihoodFunction likelihoodFunc = std::bind(&vpLikelihoodFunctor::likelihood, &likelihoodFtor, _1, _2);
      vpParticleFilter<std::vector<vpImagePoint>>::vpResamplingConditionFunction checkResamplingFunc = vpParticleFilter<std::vector<vpImagePoint>>::simpleResamplingCheck;
      vpParticleFilter<std::vector<vpImagePoint>>::vpResamplingFunction resamplingFunc = vpParticleFilter<std::vector<vpImagePoint>>::simpleImportanceResampling;
      vpAverageFunctor averageCpter(degree, height, width);
      using std::placeholders::_3;
      vpParticleFilter<std::vector<vpImagePoint>>::vpFilterFunction meanFunc = std::bind(&vpAverageFunctor::averagePolynomials, &averageCpter, _1, _2, _3);
      vpParticleFilter<std::vector<vpImagePoint>> filter(nbParticles, stdevsPF, seedPF, nbThreads);
      filter.init(X0, processFunc, likelihoodFunc, checkResamplingFunc, resamplingFunc, meanFunc);

      for (unsigned int i = 0; i < nbWarmUpIter; ++i) {
        filter.filter(curvePoints, dt);
      }

      double meanError = 0.;
      for (unsigned int i = 0; i < nbEvalIter; ++i) {
        filter.filter(curvePoints, dt);
        vpColVector Xest = filter.computeFilteredState();
        vpParabolaModel model(Xest, height, width);
        double rmse = evaluate(curvePoints, model);
        meanError += rmse;

#ifdef VISP_HAVE_DISPLAY
        if (opt_display) {
          vpDisplay::display(I);
          displayGeneratedImage(I, curvePoints, vpColor::red, "GT", 20, 20);
          model.display(I, vpColor::blue, "Model", 40, 20);
          vpDisplay::flush(I);
          vpDisplay::getClick(I);
        }
#endif
      }
      meanError /= static_cast<double>(nbEvalIter);
      std::cout << "Mean(rmse) = " << meanError << std::endl;
      CHECK(meanError <= maxToleratedError);
    }
  }



  SECTION("Noisy", "Noise is added to the init points")
  {
    const double maxToleratedError = 17.;
    double x0 = rngCurvePoints.uniform(0., width);
    double x1 = rngCurvePoints.uniform(0., width);
    double y0 = rngCurvePoints.uniform(0., height);
    double y1 = rngCurvePoints.uniform(0., height);
    vpColVector coeffs = computeABCD(x0, y0, x1, y1);
    std::vector<vpImagePoint> curvePoints = generateSimulatedImage(0, width, 1., coeffs);

#ifdef VISP_HAVE_DISPLAY
    vpImage<vpRGBa> I(height, width);
    std::shared_ptr<vpDisplay> pDisplay;
    if (opt_display) {
      pDisplay = vpDisplayFactory::createDisplay(I);
    }
#endif

    const double ampliMaxInitNoise = 1.5;
    const double stdevInitNoise = ampliMaxInitNoise / 3.;
    vpGaussRand rngInitNoise(stdevInitNoise, 0., seedInitPoints);

    for (unsigned int iter = 0; iter < nbTestRepet; ++iter) {
      // Randomly select the initialization points
      std::vector<vpImagePoint> suffledVector = vpUniRand::shuffleVector(curvePoints, seedShuffle);
      std::vector<vpImagePoint> initPoints;
      for (unsigned int j = 0; j < nbInitPoints * 4; ++j) {
        vpImagePoint noisyPt(suffledVector[j].get_i() + rngInitNoise(), suffledVector[j].get_j() + rngInitNoise());
        initPoints.push_back(noisyPt);
      }

      // Compute the initial model
      vpParabolaModel modelInitial = computeInitialGuess(initPoints, degree, height, width);
      vpColVector X0 = modelInitial.toVpColVector();

      // Initialize the Particle Filter
      std::vector<double> stdevsPF;
      for (unsigned int i = 0; i < degree + 1; ++i) {
        stdevsPF.push_back(ratioAmpliMax * std::pow(.05, i) * X0[0] / 3.);
      }
      vpParticleFilter<std::vector<vpImagePoint>>::vpProcessFunction processFunc = fx;
      vpLikelihoodFunctor likelihoodFtor(sigmaLikelihood * 2., height, width);
      using std::placeholders::_1;
      using std::placeholders::_2;
      vpParticleFilter<std::vector<vpImagePoint>>::vpLikelihoodFunction likelihoodFunc = std::bind(&vpLikelihoodFunctor::likelihood, &likelihoodFtor, _1, _2);
      vpParticleFilter<std::vector<vpImagePoint>>::vpResamplingConditionFunction checkResamplingFunc = vpParticleFilter<std::vector<vpImagePoint>>::simpleResamplingCheck;
      vpParticleFilter<std::vector<vpImagePoint>>::vpResamplingFunction resamplingFunc = vpParticleFilter<std::vector<vpImagePoint>>::simpleImportanceResampling;
      vpAverageFunctor averageCpter(degree, height, width);
      using std::placeholders::_3;
      vpParticleFilter<std::vector<vpImagePoint>>::vpFilterFunction meanFunc = std::bind(&vpAverageFunctor::averagePolynomials, &averageCpter, _1, _2, _3);
      vpParticleFilter<std::vector<vpImagePoint>> filter(nbParticles, stdevsPF, seedPF, nbThreads);
      filter.init(X0, processFunc, likelihoodFunc, checkResamplingFunc, resamplingFunc, meanFunc);

      for (unsigned int i = 0; i < nbWarmUpIter * 5; ++i) {
        filter.filter(curvePoints, dt);
      }

      double meanError = 0.;
      for (unsigned int i = 0; i < nbEvalIter; ++i) {
        filter.filter(curvePoints, dt);
        vpColVector Xest = filter.computeFilteredState();
        vpParabolaModel model(Xest, height, width);
        double rmse = evaluate(curvePoints, model);
        meanError += rmse;

#ifdef VISP_HAVE_DISPLAY
        if (opt_display) {
          vpDisplay::display(I);
          displayGeneratedImage(I, curvePoints, vpColor::red, "GT", 20, 20);
          model.display(I, vpColor::blue, "Model", 40, 20);
          vpDisplay::flush(I);
          vpDisplay::getClick(I);
        }
#endif
      }
      meanError /= static_cast<double>(nbEvalIter);
      std::cout << "Mean(rmse) = " << meanError << std::endl;
      CHECK(meanError <= maxToleratedError);
    }
  }
}

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance

  // Build a new parser on top of Catch's
  using namespace Catch::clara;
  auto cli = session.cli()         // Get Catch's composite command line parser
    | Opt(opt_display)   // bind variable to a new option, with a hint string
    ["--display"] // the option names it will respond to
    ("Activate debug display"); // description string for the help output

  // Now pass the new composite back to Catch so it uses that
  session.cli(cli);

  // Let Catch (using Clara) parse the command line
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();

  // numFailed is clamped to 255 as some unices only use the lower 8 bits.
  // This clamping has already been applied, so just return it here
  // You can also do any post run clean-up here
  return numFailed;
}

#else
#include <iostream>

int main() { return EXIT_SUCCESS; }
#endif
