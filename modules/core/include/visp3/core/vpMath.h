/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
  \file vpMath.h
  \brief Provides simple Math computation that are not available in
  the C mathematics library (math.h)
*/

#ifndef vpMATH_HH
#define vpMATH_HH

#include <visp3/core/vpConfig.h>

#include <algorithm>
#include <climits>
#include <limits>
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

/*!
  \class vpMath
  \ingroup group_core_math_tools
  \brief Provides simple mathematics computation tools that are not
  available in the C mathematics library (math.h)

  \author Eric Marchand   (Eric.Marchand@irisa.fr) Irisa / Inria Rennes
*/

class VISP_EXPORT vpMath
{
public:
  /*!
    Convert an angle in radians into degrees.

    \param rad : Angle in radians.
    \return Angle converted in degrees.
  */
  static inline double deg(double rad) { return (rad * 180.0) / M_PI; }

  /*!
    Convert an angle in degrees into radian.
    \param deg : Angle in degrees.
    \return Angle converted in radian.
  */
  static inline double rad(double deg) { return (deg * M_PI) / 180.0; }

  /*!
    Compute x square value.
    \return Square value \f$ x^2 \f$.
  */
  static inline double sqr(double x) { return x * x; }

  //  factorial of x
  static inline double fact(unsigned int x);

  // combinaison
  static inline long double comb(unsigned int n, unsigned int p);

  //   round x to the nearest integer
  static inline int round(const double x);

  //   return the sign of x (+-1)
  static inline int(sign)(double x);

  // test if a number equals 0 (with threshold value)
  static inline bool nul(double x, double s = 0.001);

  // test if two numbers are equals (with a user defined threshold)
  static inline bool equal(double x, double y, double s = 0.001);

  // test if a number is greater than another (with a user defined threshold)
  static inline bool greater(double x, double y, double s = 0.001);

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

  static bool isNaN(const double value);
  static bool isInf(const double value);

  template <typename _Tp> static inline _Tp saturate(unsigned char v) { return _Tp(v); }
  template <typename _Tp> static inline _Tp saturate(char v) { return _Tp(v); }
  template <typename _Tp> static inline _Tp saturate(unsigned short v) { return _Tp(v); }
  template <typename _Tp> static inline _Tp saturate(short v) { return _Tp(v); }
  template <typename _Tp> static inline _Tp saturate(unsigned v) { return _Tp(v); }
  template <typename _Tp> static inline _Tp saturate(int v) { return _Tp(v); }
  template <typename _Tp> static inline _Tp saturate(float v) { return _Tp(v); }
  template <typename _Tp> static inline _Tp saturate(double v) { return _Tp(v); }

  static double getMean(const std::vector<double> &v);
  static double getMedian(const std::vector<double> &v);
  static double getStdev(const std::vector<double> &v, const bool useBesselCorrection = false);

  static int modulo(const int a, const int n);

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
  if ((x == 1) || (x == 0))
    return 1;
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
  if (n == p)
    return 1;
  return fact(n) / (fact(n - p) * fact(p));
}

/*!
  Round x to the nearest integer.

  \param x : Value to round.

  \return Nearest integer of x.

*/
int vpMath::round(const double x)
{
#if defined(VISP_HAVE_FUNC_ROUND)
  //:: to design the global namespace and avoid to call recursively
  // vpMath::round
  return (int)::round(x);
#elif defined(VISP_HAVE_FUNC_STD_ROUND)
  return (int)std::round(x);
#else
  return (x > 0.0) ? ((int)floor(x + 0.5)) : ((int)ceil(x - 0.5));
#endif
}

/*!
  Return the sign of x.

  \return -1 if x is negative, +1 if positive and 0 if zero.

*/
int
#ifndef DOXYGEN_SHOULD_SKIP_THIS
    (
#endif
        vpMath::sign
#ifndef DOXYGEN_SHOULD_SKIP_THIS
        )
#endif
        (double x)
{
  if (fabs(x) < std::numeric_limits<double>::epsilon())
    return 0;
  else {
    if (x < 0)
      return -1;
    else
      return 1;
  }
}

/*!
  Compares  \f$ | x | \f$ to \f$ s \f$.
  \param x : Value to test.
  \param s : Tolerance threshold
  \return true if \f$ | x | < s \f$.

*/
bool vpMath::nul(double x, double s) { return (fabs(x) < s); }

/*!
  Compares  \f$ | x - y | \f$ to \f$ s \f$.
  \param x : x value.
  \param y : y value.
  \param s : Tolerance threshold.
  \return true if \f$ | x - y | < s \f$.
*/
bool vpMath::equal(double x, double y, double s) { return (nul(x - y, s)); }

/*!
  Compares  \f$ x \f$ to \f$ y - s \f$.
  \param x : x value.
  \param y : y value.
  \param s : Tolerance threshold.
  \return true if \f$ x > y - s \f$.
*/
bool vpMath::greater(double x, double y, double s) { return (x > (y - s)); }

/*!

 Sigmoid function between [x0,x1] with \f$ s(x)=0 if x\le x0\f$ and \f$ s(x)=1
if x \ge x1 \f$ \param x : Value of x. \param x0 : Lower bound (default 0).
 \param x1 : Upper bound (default 1).
  \param n : Degree of the exponential (default 12).

\return Sigmoid value \f$1/(1+exp(-n*((x-x0)/(x1-x0)-0.5)))\f$
 */
double vpMath::sigmoid(double x, double x0, double x1, double n)
{
  if (x < x0)
    return 0.;
  else if (x > x1)
    return 1.;
  double l0 = 1. / (1. + exp(0.5 * n));
  double l1 = 1. / (1. + exp(-0.5 * n));
  return (1. / (1. + exp(-n * ((x - x0) / (x1 - x0) - 0.5))) - l0) / (l1 - l0);
}

// unsigned char
template <> inline unsigned char vpMath::saturate<unsigned char>(char v)
{
  // On big endian arch like powerpc, char implementation is unsigned
  // with CHAR_MIN=0, CHAR_MAX=255 and SCHAR_MIN=-128, SCHAR_MAX=127
  // leading to (int)(char -127) = 129.
  // On little endian arch, CHAR_MIN=-127 and CHAR_MAX=128 leading to
  // (int)(char -127) = -127.
  if (std::numeric_limits<char>::is_signed)
    return (unsigned char)(((std::max))((int)v, 0));
  else
    return (unsigned char)((unsigned int)v > SCHAR_MAX ? 0 : v);
}

template <> inline unsigned char vpMath::saturate<unsigned char>(unsigned short v)
{
  return (unsigned char)((std::min))((unsigned int)v, (unsigned int)UCHAR_MAX);
}

template <> inline unsigned char vpMath::saturate<unsigned char>(int v)
{
  return (unsigned char)((unsigned int)v <= UCHAR_MAX ? v : v > 0 ? UCHAR_MAX : 0);
}

template <> inline unsigned char vpMath::saturate<unsigned char>(short v) { return saturate<unsigned char>((int)v); }

template <> inline unsigned char vpMath::saturate<unsigned char>(unsigned int v)
{
  return (unsigned char)((std::min))(v, (unsigned int)UCHAR_MAX);
}

template <> inline unsigned char vpMath::saturate<unsigned char>(float v)
{
  int iv = vpMath::round(v);
  return saturate<unsigned char>(iv);
}

template <> inline unsigned char vpMath::saturate<unsigned char>(double v)
{
  int iv = vpMath::round(v);
  return saturate<unsigned char>(iv);
}

// char
template <> inline char vpMath::saturate<char>(unsigned char v) { return (char)((std::min))((int)v, SCHAR_MAX); }

template <> inline char vpMath::saturate<char>(unsigned short v)
{
  return (char)((std::min))((unsigned int)v, (unsigned int)SCHAR_MAX);
}

template <> inline char vpMath::saturate<char>(int v)
{
  return (char)((unsigned int)(v - SCHAR_MIN) <= (unsigned int)UCHAR_MAX ? v : v > 0 ? SCHAR_MAX : SCHAR_MIN);
}

template <> inline char vpMath::saturate<char>(short v) { return saturate<char>((int)v); }

template <> inline char vpMath::saturate<char>(unsigned int v)
{
  return (char)((std::min))(v, (unsigned int)SCHAR_MAX);
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
  if (std::numeric_limits<char>::is_signed)
    return (unsigned char)(((std::max))((int)v, 0));
  else
    return (unsigned char)((unsigned int)v > SCHAR_MAX ? 0 : v);
}

template <> inline unsigned short vpMath::saturate<unsigned short>(short v)
{
  return (unsigned short)((std::max))((int)v, 0);
}

template <> inline unsigned short vpMath::saturate<unsigned short>(int v)
{
  return (unsigned short)((unsigned int)v <= (unsigned int)USHRT_MAX ? v : v > 0 ? USHRT_MAX : 0);
}

template <> inline unsigned short vpMath::saturate<unsigned short>(unsigned int v)
{
  return (unsigned short)((std::min))(v, (unsigned int)USHRT_MAX);
}

template <> inline unsigned short vpMath::saturate<unsigned short>(float v)
{
  int iv = vpMath::round(v);
  return vpMath::saturate<unsigned short>(iv);
}

template <> inline unsigned short vpMath::saturate<unsigned short>(double v)
{
  int iv = vpMath::round(v);
  return vpMath::saturate<unsigned short>(iv);
}

// short
template <> inline short vpMath::saturate<short>(unsigned short v) { return (short)((std::min))((int)v, SHRT_MAX); }
template <> inline short vpMath::saturate<short>(int v)
{
  return (short)((unsigned int)(v - SHRT_MIN) <= (unsigned int)USHRT_MAX ? v : v > 0 ? SHRT_MAX : SHRT_MIN);
}
template <> inline short vpMath::saturate<short>(unsigned int v)
{
  return (short)((std::min))(v, (unsigned int)SHRT_MAX);
}
template <> inline short vpMath::saturate<short>(float v)
{
  int iv = vpMath::round(v);
  return vpMath::saturate<short>(iv);
}
template <> inline short vpMath::saturate<short>(double v)
{
  int iv = vpMath::round(v);
  return vpMath::saturate<short>(iv);
}

// int
template <> inline int vpMath::saturate<int>(float v) { return vpMath::round(v); }

template <> inline int vpMath::saturate<int>(double v) { return vpMath::round(v); }

// unsigned int
// (Comment from OpenCV) we intentionally do not clip negative numbers, to
// make -1 become 0xffffffff etc.
template <> inline unsigned int vpMath::saturate<unsigned int>(float v) { return (unsigned int)vpMath::round(v); }

template <> inline unsigned int vpMath::saturate<unsigned int>(double v) { return (unsigned int)vpMath::round(v); }

#endif
