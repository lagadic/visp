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
 * Simple mathematical function not available in the C math library (math.h).
 */

/*!
 * \file vpMath.h
 * \brief Provides simple Math computation that are not available in
 * the C mathematics library (math.h)
 */

#ifndef VP_MATH_H
#define VP_MATH_H

#include <visp3/core/vpConfig.h>

#include <algorithm>
#include <climits>
#include <limits>
#if defined(_WIN32)
// Define _USE_MATH_DEFINES before including <math.h> to expose these macro
// definitions for common math constants.  These are placed under an #ifdef
// since these commonly-defined names are not part of the C or C++ standards
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif
#include <math.h>
#include <vector>

#if defined(VISP_HAVE_FUNC_ISNAN) || defined(VISP_HAVE_FUNC_STD_ISNAN) || defined(VISP_HAVE_FUNC_ISINF) ||             \
    defined(VISP_HAVE_FUNC_STD_ISINF) || defined(VISP_HAVE_FUNC_STD_ROUND)
#include <cmath>
#endif

#if defined(_WIN32) // Not defined in Microsoft math.h

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 (M_PI / 2.0)
#endif

#ifndef M_PI_4
#define M_PI_4 (M_PI / 4.0)
#endif

#endif

#ifndef M_PI_FLOAT
#define M_PI_FLOAT 3.14159265358979323846f
#endif

#ifndef M_PI_2_FLOAT
#define M_PI_2_FLOAT (M_PI_FLOAT / 2.0f)
#endif

#ifndef M_PI_4_FLOAT
#define M_PI_4_FLOAT (M_PI_FLOAT / 4.0f)
#endif

#include <visp3/core/vpException.h>
#include <visp3/core/vpImagePoint.h>

BEGIN_VISP_NAMESPACE

class vpPoint;
class vpHomogeneousMatrix;
class vpColVector;
class vpRotationVector;
class vpRxyzVector;
class vpTranslationVector;

/*!
 * \class vpMath
 * \ingroup group_core_math_tools
 * \brief Provides simple mathematics computation tools that are not
 * available in the C mathematics library (math.h)
*/
class VISP_EXPORT vpMath
{
public:
  /*!
   * Convert an angle in radians into degrees.
   *
   * \param rad : Angle in radians.
   * \return Angle converted in degrees.
   */
  static inline double deg(double rad) { return (rad * 180.0) / M_PI; }

  static vpColVector deg(const vpRotationVector &r);
  static vpColVector deg(const vpColVector &r);

  /*!
   * Convert an angle in degrees into radian.
   * \param deg : Angle in degrees.
   * \return Angle converted in radians.
   */
  static inline double rad(double deg) { return (deg * M_PI) / 180.0; }

  static vpColVector rad(const vpColVector &r);

  /*!
   * Convert angle between \f$-\pi\f$ and \f$\pi\f$.
   *
   * \param[in] theta The input angle we want to ensure it is in the interval \f$[-\pi ; \pi]\f$.
   * \return The corresponding angle in the interval \f$[-\pi ; \pi]\f$.
   */
  static float getAngleBetweenMinPiAndPi(const float &theta)
  {
    float theta1 = theta;
    if (theta1 > M_PI_FLOAT) {
      theta1 -= 2.0f * M_PI_FLOAT;
    }
    else if (theta1 <= -M_PI_FLOAT) {
      theta1 += 2.0f * M_PI_FLOAT;
    }
    return theta1;
  }

  /*!
   * Convert angle between \f$-\pi\f$ and \f$\pi\f$.
   *
   * \param[in] theta The input angle we want to ensure it is in the interval \f$[-\pi ; \pi]\f$.
   * \return The corresponding angle in the interval \f$[-\pi ; \pi]\f$.
   */
  static double getAngleBetweenMinPiAndPi(const double &theta)
  {
    double theta1 = theta;
    if (theta1 > M_PI) {
      theta1 -= 2.0 * M_PI;
    }
    else if (theta1 < -M_PI) {
      theta1 += 2.0 * M_PI;
    }
    return theta1;
  }

  /**
   * \brief Gives the rest of \b value divided by \b modulo when
   * the quotient can only be an integer.
   *
   * \param[in] value The value we want to know the rest in the "modulo" operation.
   * \param[in] modulo The divider.
   * \return float The rest as in a modulo operation.
   */
  static float modulo(const float &value, const float &modulo)
  {
    float quotient = std::floor(value / modulo);
    float rest = value - (quotient * modulo);
    return rest;
  }

  /**
   * \brief Gives the rest of \b value divided by \b modulo when
   * the quotient can only be an integer.
   *
   * \param[in] value The value we want to know the rest in the "modulo" operation.
   * \param[in] modulo The divider.
   * \return double The rest as in a modulo operation.
   */
  static double modulo(const double &value, const double &modulo)
  {
    double quotient = std::floor(value / modulo);
    double rest = value - (quotient * modulo);
    return rest;
  }

  /*!
    Compute x square value.
    \return Square value \f$ x^2 \f$.
  */
  static inline double sqr(double x) { return x * x; }

  //  factorial of x
  static inline double fact(unsigned int x);

  // combinaison
  static inline long double comb(unsigned int n, unsigned int p);

  /*!
    Clamp a value to boundaries.
    \param v : The value to clamp.
    \param lower, upper : The boundaries to clamp `v` to.

    Throw a vpException if the value of `lower` is greater than `upper`.
  */
  template <typename T> static inline T clamp(const T &v, const T &lower, const T &upper)
  {
    // Check if std:c++17 or higher.
    // Here we cannot use (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17) when ViSP
    // is used as a 3rdparty. See issue #1274
#if ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))
    return std::clamp(v, lower, upper);
#else
    if (upper < lower) {
      throw vpException(vpException::badValue, "clamp: lower bound is greater than upper bound");
    }
    return (v < lower) ? lower : (upper < v) ? upper : v;
#endif
  }

  //   round x to the nearest integer
  static inline int round(double x);

  //   return the sign of x (+-1)
  static inline int sign(double x);

  // test if a number equals 0 (with threshold value)
  static inline bool nul(double x, double threshold = 0.001);

  // test if two numbers are equals (with a user defined threshold)
  static inline bool equal(double x, double y, double threshold = 0.001);

  // test if a number is greater than another (with a user defined threshold)
  static inline bool greater(double x, double y, double threshold = 0.001);

  /*!
    Find the maximum between two numbers (or other).
    \param a : First number.
    \param b : Second number.
    \return The maximum of the two numbers.
  */
  template <class Type> static Type maximum(const Type &a, const Type &b) { return (a > b) ? a : b; }

  /*!
    Find the minimum between two numbers (or other).
    \param a : First number.
    \param b : Second number.
    \return The minimum of the two numbers.
  */
  template <class Type> static Type minimum(const Type &a, const Type &b) { return (a < b) ? a : b; }

  /*!
    Find the absolute value of a number (or other).
    \param x : The number.
    \return The absolute value of x
  */
  template <class Type> static Type abs(const Type &x) { return (x < 0) ? -x : x; }

  // sinus cardinal
  static double sinc(double x);
  static double sinc(double sinx, double x);
  static double mcosc(double cosx, double x);
  static double msinc(double sinx, double x);

  // sigmoid
  static inline double sigmoid(double x, double x0 = 0., double x1 = 1., double n = 12.);

  /*!
    Exchange two numbers.

    \param a First number to exchange.
    \param b Second number to exchange
  */
  template <class Type> static void swap(Type &a, Type &b)
  {
    Type tmp = b;
    b = a;
    a = tmp;
  }

  static bool isNaN(double value);
  static bool isNaN(float value);
  static bool isInf(double value);
  static bool isInf(float value);
  static bool isFinite(double value);
  static bool isFinite(float value);
  static bool isNumber(const std::string &str);

  static double lineFitting(const std::vector<vpImagePoint> &imPts, double &a, double &b, double &c);

  template <typename Tp> static inline Tp saturate(unsigned char v) { return Tp(v); }
  template <typename Tp> static inline Tp saturate(char v) { return Tp(v); }
  template <typename Tp> static inline Tp saturate(unsigned short v) { return Tp(v); }
  template <typename Tp> static inline Tp saturate(short v) { return Tp(v); }
  template <typename Tp> static inline Tp saturate(unsigned v) { return Tp(v); }
  template <typename Tp> static inline Tp saturate(int v) { return Tp(v); }
  template <typename Tp> static inline Tp saturate(float v) { return Tp(v); }
  template <typename Tp> static inline Tp saturate(double v) { return Tp(v); }

  static double getMean(const std::vector<double> &v);
  static double getMedian(const std::vector<double> &v);
  static double getStdev(const std::vector<double> &v, bool useBesselCorrection = false);

  static int modulo(int a, int n);

  static vpHomogeneousMatrix ned2ecef(double lonDeg, double latDeg, double radius);
  static vpHomogeneousMatrix enu2ecef(double lonDeg, double latDeg, double radius);
  static vpHomogeneousMatrix enu2ned(const vpHomogeneousMatrix &enu_M);

  /*!
    Similar to the NumPy linspace function: "Return evenly spaced numbers over a specified interval."
    Code from: https://stackoverflow.com/a/27030598

    \param start_in : The starting value of the sequence.
    \param end_in : The end value of the sequence.
    \param num_in : Number of samples to generate.

    \return Returns \e num_in evenly spaced samples, calculated over the interval [\e start_in, \e end_in].
  */
  template <typename T> static std::vector<double> linspace(T start_in, T end_in, unsigned int num_in)
  {
    std::vector<double> linspaced;

    double start = static_cast<double>(start_in);
    double end = static_cast<double>(end_in);
    double num = static_cast<double>(num_in);

    if (std::fabs(num) < std::numeric_limits<double>::epsilon()) {
      return linspaced;
    }
    if (std::fabs(num - 1) < std::numeric_limits<double>::epsilon()) {
      linspaced.push_back(start);
      return linspaced;
    }

    double delta = (end - start) / (num - 1);

    for (int i = 0; i < (num - 1); ++i) {
      linspaced.push_back(start + (delta * i));
    }
    linspaced.push_back(end); // I want to ensure that start and end
    // are exactly the same as the input
    return linspaced;
  }

  static std::vector<std::pair<double, double> > computeRegularPointsOnSphere(unsigned int maxPoints);
  static std::vector<vpHomogeneousMatrix>
    getLocalTangentPlaneTransformations(const std::vector<std::pair<double, double> > &lonlatVec, double radius,
      vpHomogeneousMatrix(*toECEF)(double lonDeg, double latDeg, double radius));

  static vpHomogeneousMatrix lookAt(const vpColVector &from, const vpColVector &to, vpColVector tmp);

private:
  static const double ang_min_sinc;
  static const double ang_min_mc;
};

// Begining of the inline functions definition

/*!
  Computes and returns x!
  \param x : parameter of factorial function.
*/
double vpMath::fact(unsigned int x)
{
  if ((x == 1) || (x == 0)) {
    return 1;
  }
  return x * fact(x - 1);
}

/*!
  Computes the number of combination of p elements inside n elements.

  \param n : total number of elements.
  \param p : requested number of elements.

  \return Combination number \f$ n! / ((n-p)! p!) \f$
*/
long double vpMath::comb(unsigned int n, unsigned int p)
{
  if (n == p) {
    return 1;
  }
  return fact(n) / (fact(n - p) * fact(p));
}

/*!
  Round x to the nearest integer.

  \param x : Value to round.

  \return Nearest integer of x.
*/
int vpMath::round(double x)
{
#if defined(VISP_HAVE_FUNC_STD_ROUND)
  return static_cast<int>(std::round(x));
#elif defined(VISP_HAVE_FUNC_ROUND)
  //:: to design the global namespace and avoid to call recursively
  // vpMath::round
  return static_cast<int>(::round(x));
#else
  return (x > 0.0) ? (static_cast<int>(floor(x + 0.5))) : (static_cast<int>(ceil(x - 0.5)));
#endif
}

/*!
  Return the sign of x.

  \param x : Value to test.
  \return -1 if x is negative, +1 if positive and 0 if zero.
*/
int vpMath::sign(double x)
{
  if (fabs(x) < std::numeric_limits<double>::epsilon()) {
    return 0;
  }
  else {
    if (x < 0) {
      return -1;
    }
    else {
      return 1;
    }
  }
}

/*!
  Compares \f$ | x | \f$ to `threshold`.
  \param x : Value to test.
  \param threshold : Tolerance threshold
  \return true if \f$ | x | <\f$ `threshold`.
*/
bool vpMath::nul(double x, double threshold) { return (fabs(x) < threshold); }

/*!
  Compares \f$ | x - y | \f$ to `threshold`.
  \param x : x value.
  \param y : y value.
  \param threshold : Tolerance threshold.
  \return true if \f$ | x - y | <\f$ `threshold`.
*/
bool vpMath::equal(double x, double y, double threshold) { return (nul(x - y, threshold)); }

/*!
  Compares \f$ x \f$ to \f$ y -\f$ `threshold`.
  \param x : x value.
  \param y : y value.
  \param threshold : Tolerance threshold.
  \return true if \f$ x > y -\f$ `threshold`.
*/
bool vpMath::greater(double x, double y, double threshold) { return (x > (y - threshold)); }

/*!

  Sigmoid function between [x0,x1] with \f$ s(x)=0 if x\le x0\f$ and \f$ s(x)=1
  if x \ge x1 \f$
  \param x : Value of x.
  \param x0 : Lower bound (default 0).
  \param x1 : Upper bound (default 1).
  \param n : Degree of the exponential (default 12).

  \return Sigmoid value \f$1/(1+exp(-n*((x-x0)/(x1-x0)-0.5)))\f$
 */
double vpMath::sigmoid(double x, double x0, double x1, double n)
{
  if (x < x0) {
    return 0.;
  }
  else if (x > x1) {
    return 1.;
  }
  double l0 = 1. / (1. + exp(0.5 * n));
  double l1 = 1. / (1. + exp(-0.5 * n));
  return ((1. / (1. + exp(-n * (((x - x0) / (x1 - x0)) - 0.5)))) - l0) / (l1 - l0);
}

// unsigned char
template <> inline unsigned char vpMath::saturate<unsigned char>(char v)
{
  // On big endian arch like powerpc, char implementation is unsigned
  // with CHAR_MIN=0, CHAR_MAX=255 and SCHAR_MIN=-128, SCHAR_MAX=127
  // leading to (int)(char -127) = 129.
  // On little endian arch, CHAR_MIN=-127 and CHAR_MAX=128 leading to
  // (int)(char -127) = -127.
  if (std::numeric_limits<char>::is_signed) {
    return static_cast<unsigned char>(std::max<int>(static_cast<int>(v), 0));
  }
  else {
    return static_cast<unsigned char>(static_cast<unsigned int>(v) > SCHAR_MAX ? 0 : v);
  }
}

template <> inline unsigned char vpMath::saturate<unsigned char>(unsigned short v)
{
  return static_cast<unsigned char>(std::min<unsigned int>(static_cast<unsigned int>(v), static_cast<unsigned int>(UCHAR_MAX)));
}

template <> inline unsigned char vpMath::saturate<unsigned char>(int v)
{
  return static_cast<unsigned char>(static_cast<unsigned int>(v) <= UCHAR_MAX ? v : v > 0 ? UCHAR_MAX : 0);
}

template <> inline unsigned char vpMath::saturate<unsigned char>(short v)
{
  return saturate<unsigned char>(static_cast<int>(v));
}

template <> inline unsigned char vpMath::saturate<unsigned char>(unsigned int v)
{
  return static_cast<unsigned char>(std::min<unsigned int>(v, static_cast<unsigned int>(UCHAR_MAX)));
}

template <> inline unsigned char vpMath::saturate<unsigned char>(float v)
{
  int iv = vpMath::round(static_cast<double>(v));
  return saturate<unsigned char>(iv);
}

template <> inline unsigned char vpMath::saturate<unsigned char>(double v)
{
  int iv = vpMath::round(v);
  return saturate<unsigned char>(iv);
}

// char
template <> inline char vpMath::saturate<char>(unsigned char v)
{
  return static_cast<char>(std::min<int>(static_cast<int>(v), SCHAR_MAX));
}

template <> inline char vpMath::saturate<char>(unsigned short v)
{
  return static_cast<char>(std::min<unsigned int>(static_cast<unsigned int>(v), static_cast<unsigned int>(SCHAR_MAX)));
}

template <> inline char vpMath::saturate<char>(int v)
{
  return static_cast<char>(static_cast<unsigned int>(v - SCHAR_MIN) <= static_cast<unsigned int>(UCHAR_MAX) ? v : v > 0 ? SCHAR_MAX : SCHAR_MIN);
}

template <> inline char vpMath::saturate<char>(short v)
{
  return saturate<char>(static_cast<int>(v));
}

template <> inline char vpMath::saturate<char>(unsigned int v)
{
  return static_cast<char>(std::min<unsigned int>(v, static_cast<unsigned int>(SCHAR_MAX)));
}

template <> inline char vpMath::saturate<char>(float v)
{
  int iv = vpMath::round(v);
  return saturate<char>(iv);
}

template <> inline char vpMath::saturate<char>(double v)
{
  int iv = vpMath::round(v);
  return saturate<char>(iv);
}

// unsigned short
template <> inline unsigned short vpMath::saturate<unsigned short>(char v)
{
  // On big endian arch like powerpc, char implementation is unsigned
  // with CHAR_MIN=0, CHAR_MAX=255 and SCHAR_MIN=-128, SCHAR_MAX=127
  // leading to (int)(char -127) = 129.
  // On little endian arch, CHAR_MIN=-127 and CHAR_MAX=128 leading to
  // (int)(char -127) = -127.
  if (std::numeric_limits<char>::is_signed) {
    return static_cast<unsigned short>(std::max<int>(static_cast<int>(v), 0));
  }
  else {
    return static_cast<unsigned short>(static_cast<unsigned int>(v) > SCHAR_MAX ? 0 : v);
  }
}

template <> inline unsigned short vpMath::saturate<unsigned short>(short v)
{
  return static_cast<unsigned short>(std::max<int>(static_cast<int>(v), 0));
}

template <> inline unsigned short vpMath::saturate<unsigned short>(int v)
{
  return static_cast<unsigned short>(static_cast<unsigned int>(v) <= static_cast<unsigned int>(USHRT_MAX) ? v : v > 0 ? USHRT_MAX : 0);
}

template <> inline unsigned short vpMath::saturate<unsigned short>(unsigned int v)
{
  return static_cast<unsigned short>(std::min<unsigned int>(v, static_cast<unsigned int>(USHRT_MAX)));
}

template <> inline unsigned short vpMath::saturate<unsigned short>(float v)
{
  int iv = vpMath::round(static_cast<double>(v));
  return vpMath::saturate<unsigned short>(iv);
}

template <> inline unsigned short vpMath::saturate<unsigned short>(double v)
{
  int iv = vpMath::round(v);
  return vpMath::saturate<unsigned short>(iv);
}

// short
template <> inline short vpMath::saturate<short>(unsigned short v)
{
  return static_cast<short>(std::min<int>(static_cast<int>(v), SHRT_MAX));
}
template <> inline short vpMath::saturate<short>(int v)
{
  return static_cast<short>(static_cast<unsigned int>(v - SHRT_MIN) <= static_cast<unsigned int>(USHRT_MAX) ? v : v > 0 ? SHRT_MAX : SHRT_MIN);
}
template <> inline short vpMath::saturate<short>(unsigned int v)
{
  return static_cast<short>(std::min<unsigned int>(v, static_cast<unsigned int>(SHRT_MAX)));
}
template <> inline short vpMath::saturate<short>(float v)
{
  int iv = vpMath::round(static_cast<double>(v));
  return vpMath::saturate<short>(iv);
}
template <> inline short vpMath::saturate<short>(double v)
{
  int iv = vpMath::round(v);
  return vpMath::saturate<short>(iv);
}

// int
template <> inline int vpMath::saturate<int>(float v)
{
  return vpMath::round(static_cast<double>(v));
}

template <> inline int vpMath::saturate<int>(double v)
{
  return vpMath::round(v);
}

// unsigned int
// (Comment from OpenCV) we intentionally do not clip negative numbers, to
// make -1 become 0xffffffff etc.
template <> inline unsigned int vpMath::saturate<unsigned int>(float v)
{
  return static_cast<unsigned int>(vpMath::round(static_cast<double>(v)));
}

template <> inline unsigned int vpMath::saturate<unsigned int>(double v)
{
  return static_cast<unsigned int>(vpMath::round(v));
}
END_VISP_NAMESPACE
#endif
