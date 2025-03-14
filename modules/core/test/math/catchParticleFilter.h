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

  Header file for catchParticleFilter tests.
*/
#include <visp3/core/vpConfig.h>

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
#include <limits>
#include <vector>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpParticleFilter.h>
#include <visp3/core/vpUniRand.h>

#ifdef VISP_HAVE_DISPLAY
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayFactory.h>
#endif

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
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
  static void fillSystem(const unsigned int &degree, const unsigned int &height, const unsigned int &width, const std::vector< vpImagePoint> &pts, vpMatrix &A, vpMatrix &b)
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
  inline void display(const vpImage<T> &I, const vpColor &color, const std::string &legend,
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
inline vpColVector computeABC(const double &x0, const double &y0, const double &x1, const double &y1)
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
inline vpColVector computeABCD(const double &x0, const double &y0, const double &x1, const double &y1)
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
inline double computeY(const double &x, const vpColVector &coeffs)
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
inline std::vector<vpImagePoint> generateSimulatedImage(const double &xmin, const double &xmax, const double &step, const vpColVector &coeffs)
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
inline void displayGeneratedImage(const vpImage<T> &I, const std::vector<vpImagePoint> &pts, const vpColor &color,
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
inline vpParabolaModel computeInitialGuess(const std::vector<vpImagePoint> &pts, const unsigned int &degree, const unsigned int &height, const unsigned int &width)
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
inline double evaluate(const std::vector<vpImagePoint> &pts, const vpParabolaModel &model)
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
inline vpColVector fx(const vpColVector &coeffs, const double &/*dt*/)
{
  vpColVector updatedCoeffs = coeffs;
  return updatedCoeffs;
}

class vpAverageFunctor
{
public:
  inline vpAverageFunctor(const unsigned int &degree, const unsigned int &height, const unsigned int &width)
    : m_degree(degree)
    , m_height(height)
    , m_width(width)
  { }

  inline vpColVector averagePolynomials(const std::vector<vpColVector> &particles, const std::vector<double> &weights, const vpParticleFilter<std::vector<vpImagePoint>>::vpStateAddFunction &/**/)
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
      else if (vpMath::equal(nbPoints, 1.)) {
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
  inline vpLikelihoodFunctor(const double &stdev, const unsigned int &height, const unsigned int &width)
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
  inline double likelihood(const vpColVector &coeffs, const std::vector<vpImagePoint> &meas)
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
#endif
