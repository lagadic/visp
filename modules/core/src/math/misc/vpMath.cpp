/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * See http://visp.inria.fr for more information.
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
 * Simple mathematical function not available in the C math library (math.h).
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpMath.cpp
  \brief Provides simple Math computation that are not available in
  the C mathematics library (math.h)
*/

#include <cmath>
#include <functional>
#include <numeric>
#include <stdint.h>

#include <visp3/core/vpException.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>

#if defined(VISP_HAVE_FUNC__ISNAN)
#include <float.h>
#endif

#if !(defined(VISP_HAVE_FUNC_ISNAN) || defined(VISP_HAVE_FUNC_STD_ISNAN)) ||                                           \
    !(defined(VISP_HAVE_FUNC_ISINF) || defined(VISP_HAVE_FUNC_STD_ISINF))
#if defined _MSC_VER || defined __BORLANDC__
typedef __int64 int64;
typedef unsigned __int64 uint64;
#else
typedef int64_t int64;
typedef uint64_t uint64;
#endif

#ifndef DOXYGEN_SHOULD_SKIP_THIS
typedef union Cv64suf {
  //  int64 i; //Unused variable, should be harmless to comment it
  uint64 u;
  double f;
} Cv64suf;
#endif
#endif

const double vpMath::ang_min_sinc = 1.0e-8;
const double vpMath::ang_min_mc = 2.5e-4;

/*!
   Check whether a double number is not a number (NaN) or not.
   \param value : Double number to check.
   \return Return true if value is not a number.
 */
bool vpMath::isNaN(double value)
{
#if defined(VISP_HAVE_FUNC_ISNAN)
  return isnan(value);
#elif defined(VISP_HAVE_FUNC_STD_ISNAN)
  return std::isnan(value);
#elif defined(VISP_HAVE_FUNC__ISNAN)
  return (_isnan(value) != 0);
#else
#if 0
    //This trick should work for any compiler which claims to use IEEE floating point.
    //Do not work with g++ and -ffast-math option.
    return (value != value);
#else
  // Taken from OpenCV source code CvIsNan()
  Cv64suf ieee754;
  ieee754.f = value;
  return (((unsigned)(ieee754.u >> 32) & 0x7fffffff) + ((unsigned)ieee754.u != 0) > 0x7ff00000) != 0;
#endif
#endif
}
/*!
   Returns whether a double is an infinity value (either positive infinity or
   negative infinity). \param value : Double number to check. \return Return
   true if value is infinity.
 */
bool vpMath::isInf(double value)
{
#if defined(VISP_HAVE_FUNC_ISINF)
  return isinf(value);
#elif defined(VISP_HAVE_FUNC_STD_ISINF)
  return std::isinf(value);
#elif defined(VISP_HAVE_FUNC__FINITE)
  return !_finite(value);
#else
  // Taken from OpenCV source code CvIsInf()
  Cv64suf ieee754;
  ieee754.f = value;
  return ((unsigned)(ieee754.u >> 32) & 0x7fffffff) == 0x7ff00000 && (unsigned)ieee754.u == 0;
#endif
}

/*!
  Compute \f$ (1-cos(x))/x^2 \f$

  \param cosx : Value of cos(x).
  \param x : Value of x.

  \return \f$ (1-cosx)/x^2 \f$

*/
double vpMath::mcosc(double cosx, double x)
{
  if (fabs(x) < ang_min_mc)
    return 0.5;
  else
    return ((1.0 - cosx) / x / x);
}

/*!
  Compute \f$ (1-sinc(x))/x^2 \f$ with \f$ sinc(x) = sinx / x \f$.

  \param sinx : value of sin(x).
  \param x  : Value of x.

  \return \f$ (1-sinc(x))/x^2 \f$

*/
double vpMath::msinc(double sinx, double x)
{
  if (fabs(x) < ang_min_mc)
    return (1. / 6.0);
  else
    return ((1.0 - sinx / x) / x / x);
}

/*!
  Compute sinus cardinal \f$ \frac{sin(x)}{x} \f$.

  \param x : Value of x.

  \return Sinus cardinal.

*/
double vpMath::sinc(double x)
{
  if (fabs(x) < ang_min_sinc)
    return 1.0;
  else
    return sin(x) / x;
}
/*!
  Compute sinus cardinal \f$ \frac{sin(x)}{x}\f$.

  \param sinx : Value of sin(x).
  \param x : Value of x.

  \return Sinus cardinal.

*/
double vpMath::sinc(double sinx, double x)
{
  if (fabs(x) < ang_min_sinc)
    return 1.0;
  else
    return (sinx / x);
}

/*!
  Compute the mean value for a vector of double.

  \param v : Vector of double values.

  \return The mean value.
*/
double vpMath::getMean(const std::vector<double> &v)
{
  if (v.empty()) {
    throw vpException(vpException::notInitialized, "Empty vector !");
  }

  size_t size = v.size();

  double sum = std::accumulate(v.begin(), v.end(), 0.0);

  return sum / (double)size;
}

/*!
  Compute the median value for a vector of double.

  \param v : Vector of double values.

  \return The median value.
*/
double vpMath::getMedian(const std::vector<double> &v)
{
  if (v.empty()) {
    throw vpException(vpException::notInitialized, "Empty vector !");
  }

  std::vector<double> v_copy = v;
  size_t size = v_copy.size();

  size_t n = size / 2;
  std::nth_element(v_copy.begin(), v_copy.begin() + n, v_copy.end());
  double val_n = v_copy[n];

  if (size % 2 == 1) {
    return val_n;
  } else {
    std::nth_element(v_copy.begin(), v_copy.begin() + n - 1, v_copy.end());
    return 0.5 * (val_n + v_copy[n - 1]);
  }
}

/*!
  Compute the standard deviation value for a vector of double.

  \param v : Vector of double values.
  \param useBesselCorrection : If true, the Bessel correction is used
  (normalize by N-1).

  \return The standard deviation value.
*/
double vpMath::getStdev(const std::vector<double> &v, bool useBesselCorrection)
{
  if (v.empty()) {
    throw vpException(vpException::notInitialized, "Empty vector !");
  }

  double mean = getMean(v);

  std::vector<double> diff(v.size());
#if VISP_CXX_STANDARD > VISP_CXX_STANDARD_98
  std::transform(v.begin(), v.end(), diff.begin(), std::bind(std::minus<double>(), std::placeholders::_1, mean));
#else
  std::transform(v.begin(), v.end(), diff.begin(), std::bind2nd(std::minus<double>(), mean));
#endif

  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  double divisor = (double)v.size();
  if (useBesselCorrection && v.size() > 1) {
    divisor = divisor - 1;
  }

  return std::sqrt(sq_sum / divisor);
}

/*!
  Compute the line equation using least-squares fitting that minimizes the cost function:
  \f[
    \mathbf{E} = \sum_{i=1}^{n}\left ( ax_i + by_i - c \right )^2
  \f]

  \param imPts : Image points (size  >= 3).
  \param a : a coefficient.
  \param b : b coefficient.
  \param c : c coefficient.

  \return The mean distance error (point-to-line distance) between the points and the fitted line.
*/
double vpMath::lineFitting(const std::vector<vpImagePoint>& imPts, double& a, double& b, double& c)
{
  if (imPts.size() < 3) {
    throw vpException(vpException::dimensionError, "Number of image points must be greater or equal to 3.");
  }

  double x_mean = 0, y_mean = 0;
  for (size_t i = 0; i < imPts.size(); i++) {
    const vpImagePoint& imPt = imPts[i];
    x_mean += imPt.get_u();
    y_mean += imPt.get_v();
  }
  x_mean /= imPts.size();
  y_mean /= imPts.size();

  vpMatrix AtA(2, 2, 0.0);
  for (size_t i = 0; i < imPts.size(); i++) {
    const vpImagePoint& imPt = imPts[i];
    AtA[0][0] += (imPt.get_u() - x_mean)*(imPt.get_u() - x_mean);
    AtA[0][1] += (imPt.get_u() - x_mean)*(imPt.get_v() - y_mean);
    AtA[1][1] += (imPt.get_v() - y_mean)*(imPt.get_v() - y_mean);
  }
  AtA[1][0] = AtA[0][1];

  vpColVector eigenvalues;
  vpMatrix eigenvectors;
  AtA.eigenValues(eigenvalues, eigenvectors);

  a = eigenvectors[0][0];
  b = eigenvectors[1][0];
  c = a*x_mean + b*y_mean;

  double error = 0;
  for (size_t i = 0; i < imPts.size(); i++) {
    double x0 = imPts[i].get_u();
    double y0 = imPts[i].get_v();

    error += std::fabs(a*x0 + b*y0 - c);
  }

  return error / imPts.size();
}

/*!
  Compute the modified modulo:
    - modulo(11, 10) == 1 == 11 % 10
    - modulo(-1, 10) == 9

  \param a : The dividend.
  \param n : The divisor.

  \return The modified modulo of a mod n.
*/
int vpMath::modulo(int a, int n) { return ((a % n) + n) % n; }
