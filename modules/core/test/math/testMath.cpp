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
 * Test some core functionalities.
 */

 /*!
   \example testMath.cpp

   Test some vpMath functionalities.
 */

#include <cfloat>
#include <iostream>
#include <limits>

#include <visp3/core/vpMath.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpRxyzVector.h>

#include <visp3/core/vpHomogeneousMatrix.h>

#if defined _MSC_VER && _MSC_VER >= 1200
#pragma warning(disable : 4723)

 // 4723 : potential divide by 0
#endif

#ifdef _WIN32
#ifndef NAN
// https://msdn.microsoft.com/en-us/library/w22adx1s%28v=vs.120%29.aspx
// http://tdistler.com/2011/03/24/how-to-define-nan-not-a-number-on-windows
static const unsigned long __nan[2] = { 0xffffffff, 0x7fffffff };
#define NAN (*(const float *)__nan)
#endif
#endif

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  // Test isNaN
  if (vpMath::isNaN(0.0)) {
    std::cerr << "Fail: IsNaN(0.0)=" << vpMath::isNaN(0.0) << " / should be false" << std::endl;
    return EXIT_FAILURE;
  }

  double num = 1.0, den = 0.0;
  if (vpMath::isNaN(num / den)) {
    std::cerr << "Fail: IsNaN(1.0/0.0)=" << vpMath::isNaN(num / den) << " / should be false" << std::endl;
    return EXIT_FAILURE;
  }

  if (!vpMath::isNaN(NAN)) {
    std::cerr << "Fail: IsNaN(NAN)=" << vpMath::isNaN(NAN) << " / should be true" << std::endl;
    return EXIT_FAILURE;
  }

  num = 0.0;
  if (!vpMath::isNaN(num / den)) {
    std::cerr << "Fail: IsNaN(0.0/0.0)=" << vpMath::isNaN(num / den) << " / should be true" << std::endl;
    return EXIT_FAILURE;
  }

  if (!vpMath::isNaN(std::numeric_limits<double>::quiet_NaN())) {
    std::cerr << "Fail: IsNaN(quiet_NaN)=" << vpMath::isNaN(std::numeric_limits<double>::quiet_NaN())
      << " / should be true" << std::endl;
    return EXIT_FAILURE;
  }

  if (!vpMath::isNaN(std::numeric_limits<float>::quiet_NaN())) {
    std::cerr << "Fail: IsNaN(quiet_NaN)=" << vpMath::isNaN(std::numeric_limits<float>::quiet_NaN())
      << " / should be true" << std::endl;
    return EXIT_FAILURE;
  }

  if (!vpMath::isNaN(std::numeric_limits<double>::signaling_NaN())) {
    std::cerr << "Fail: IsNaN(signaling_NaN)=" << vpMath::isNaN(std::numeric_limits<double>::signaling_NaN())
      << " / should be true" << std::endl;
    return EXIT_FAILURE;
  }

  if (!vpMath::isNaN(std::numeric_limits<float>::signaling_NaN())) {
    std::cerr << "Fail: IsNaN(signaling_NaN)=" << vpMath::isNaN(std::numeric_limits<float>::signaling_NaN())
      << " / should be true" << std::endl;
    return EXIT_FAILURE;
  }

  if (vpMath::isNaN(std::numeric_limits<double>::infinity())) {
    std::cerr << "Fail: IsNaN(infinity)=" << vpMath::isNaN(std::numeric_limits<double>::infinity())
      << " / should be false" << std::endl;
    return EXIT_FAILURE;
  }

  if (vpMath::isNaN(std::numeric_limits<float>::infinity())) {
    std::cerr << "Fail: IsNaN(infinity)=" << vpMath::isNaN(std::numeric_limits<float>::infinity())
      << " / should be false" << std::endl;
    return EXIT_FAILURE;
  }

  if (vpMath::isNaN(1.0 / std::numeric_limits<double>::epsilon())) {
    std::cerr << "Fail: IsNaN(1.0/epsilon)=" << vpMath::isNaN(1.0 / std::numeric_limits<double>::epsilon())
      << " / should be false" << std::endl;
    return EXIT_FAILURE;
  }

  if (!vpMath::isNaN(std::numeric_limits<double>::infinity() - std::numeric_limits<double>::infinity())) {
    std::cerr << "Fail: IsNaN(infinity - infinity)="
      << vpMath::isNaN(std::numeric_limits<double>::infinity() - std::numeric_limits<double>::infinity())
      << " / should be true" << std::endl;
    return EXIT_FAILURE;
  }

  float a = 0.0f, b = 0.0f;
  if (!vpMath::isNaN(a / b)) {
    std::cerr << "Fail: IsNaN(0.0f/0.0f)=" << vpMath::isNaN(a / b) << " / should be true" << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "vpMath::isNaN is Ok !" << std::endl;

  // Test isInf
  if (vpMath::isInf(NAN)) {
    std::cerr << "Fail: vpMath::isInf(NAN)=" << vpMath::isInf(NAN) << " / should be false" << std::endl;
    return EXIT_FAILURE;
  }

  if (!vpMath::isInf(1.0 / a)) {
    std::cerr << "Fail: vpMath::isInf(1.0/0.0)=" << vpMath::isInf(1.0 / a) << " / should be true" << std::endl;
    return EXIT_FAILURE;
  }

  if (vpMath::isInf(0.0)) {
    std::cerr << "Fail: vpMath::isInf(0.0)=" << vpMath::isInf(0.0) << " / should be false" << std::endl;
    return EXIT_FAILURE;
  }

  if (!vpMath::isInf(exp(800.))) {
    std::cerr << "Fail: vpMath::isInf(exp(800.))=" << vpMath::isInf(exp(800.)) << " / should be true" << std::endl;
    return EXIT_FAILURE;
  }

  if (vpMath::isInf(DBL_MIN / 2.0)) {
    std::cerr << "Fail: vpMath::isInf(DBL_MIN/2.0)=" << vpMath::isInf(DBL_MIN / 2.0) << " / should be false"
      << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "vpMath::isInf is Ok !" << std::endl;

  // Test isfinite
  if (vpMath::isFinite(NAN)) {
    std::cerr << "Fail: vpMath::isFinite(NAN)=" << vpMath::isFinite(NAN) << " / should be false" << std::endl;
    return EXIT_FAILURE;
  }

  if (vpMath::isFinite(1.0 / a)) {
    std::cerr << "Fail: vpMath::isFinite(1.0/0.0)=" << vpMath::isFinite(1.0 / a) << " / should be false" << std::endl;
    return EXIT_FAILURE;
  }

  if (!vpMath::isFinite(0.0)) {
    std::cerr << "Fail: vpMath::isFinite(0.0)=" << vpMath::isFinite(0.0) << " / should be true" << std::endl;
    return EXIT_FAILURE;
  }

  if (vpMath::isFinite(exp(800.))) {
    std::cerr << "Fail: vpMath::isFinite(exp(800.))=" << vpMath::isFinite(exp(800.)) << " / should be false" << std::endl;
    return EXIT_FAILURE;
  }

  if (!vpMath::isFinite(DBL_MIN / 2.0)) {
    std::cerr << "Fail: vpMath::isFinite(DBL_MIN/2.0)=" << vpMath::isFinite(DBL_MIN / 2.0) << " / should be true"
      << std::endl;
    return EXIT_FAILURE;
  }

  if (!vpMath::isFinite(std::numeric_limits<float>::max())) {
    std::cerr << "Fail: vpMath::isFinite(DBL_MAX)=" << vpMath::isFinite(std::numeric_limits<float>::max()) << " / should be true"
      << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "vpMath::isFinite is Ok !" << std::endl;

  // Test isNumber
  if (!vpMath::isNumber("123")) {
    std::cerr << "Fail: vpMath::isNumber(\"123\")=" << vpMath::isNumber("123") << " / should be false" << std::endl;
    return EXIT_FAILURE;
  }
  if (vpMath::isNumber("string")) {
    std::cerr << "Fail: vpMath::isNumber(\"string\")=" << vpMath::isNumber("string") << " / should be true" << std::endl;
    return EXIT_FAILURE;
  }
  if (vpMath::isNumber("123string")) {
    std::cerr << "Fail: vpMath::isNumber(\"123string\")=" << vpMath::isNumber("123string") << " / should be true" << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "vpMath::isNumber is Ok !" << std::endl;

  // Test round
  if (vpMath::round(2.3) != 2) {
    std::cerr << "Fail: vpMath::round(2.3)=" << vpMath::round(2.3) << " / should be 2" << std::endl;
    return EXIT_FAILURE;
  }

  if (vpMath::round(3.8) != 4) {
    std::cerr << "Fail: vpMath::round(3.8)=" << vpMath::round(3.8) << " / should be 4" << std::endl;
    return EXIT_FAILURE;
  }

  if (vpMath::round(5.5) != 6) {
    std::cerr << "Fail: vpMath::round(5.5)=" << vpMath::round(5.5) << " / should be 6" << std::endl;
    return EXIT_FAILURE;
  }

  if (vpMath::round(-2.3) != -2) {
    std::cerr << "Fail: vpMath::round(-2.3)=" << vpMath::round(-2.3) << " / should be -2" << std::endl;
    return EXIT_FAILURE;
  }

  if (vpMath::round(-3.8) != -4) {
    std::cerr << "Fail: vpMath::round(-3.8)=" << vpMath::round(-3.8) << " / should be -4" << std::endl;
    return EXIT_FAILURE;
  }

  if (vpMath::round(-5.5) != -6) {
    std::cerr << "Fail: vpMath::round(-5.5)=" << vpMath::round(-5.5) << " / should be -6" << std::endl;
    return EXIT_FAILURE;
  }

  if (vpMath::round(0.0) != 0) {
    std::cerr << "Fail: vpMath::round(0.0)=" << vpMath::round(0.0) << " / should be 0" << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "vpMath::round is Ok !" << std::endl;

  // Test saturate functions
  // unsigned char
  char char_value = -127;
  unsigned char uchar_value = vpMath::saturate<unsigned char>(char_value);
  if (uchar_value != 0) {
    std::cerr << "Fail: vpMath::saturate<unsigned char>(-127)=" << uchar_value << " / should be 0" << std::endl;
    return EXIT_FAILURE;
  }

  unsigned short ushort_value = 60000;
  uchar_value = vpMath::saturate<unsigned char>(ushort_value);
  if (uchar_value != UCHAR_MAX) {
    std::cerr << "Fail: vpMath::saturate<unsigned char>(60000)=" << uchar_value << " / should be " << UCHAR_MAX
      << std::endl;
    return EXIT_FAILURE;
  }

  int int_value = 70000;
  uchar_value = vpMath::saturate<unsigned char>(int_value);
  if (uchar_value != UCHAR_MAX) {
    std::cerr << "Fail: vpMath::saturate<unsigned char>(70000)=" << uchar_value << " / should be " << UCHAR_MAX
      << std::endl;
    return EXIT_FAILURE;
  }

  int_value = -70000;
  uchar_value = vpMath::saturate<unsigned char>(int_value);
  if (uchar_value != 0) {
    std::cerr << "Fail: vpMath::saturate<unsigned char>(-70000)=" << uchar_value << " / should be 0" << std::endl;
    return EXIT_FAILURE;
  }

  short short_value = 30000;
  uchar_value = vpMath::saturate<unsigned char>(short_value);
  if (uchar_value != UCHAR_MAX) {
    std::cerr << "Fail: vpMath::saturate<unsigned char>(30000)=" << uchar_value << " / should be " << UCHAR_MAX
      << std::endl;
    return EXIT_FAILURE;
  }

  short_value = -30000;
  uchar_value = vpMath::saturate<unsigned char>(short_value);
  if (uchar_value != 0) {
    std::cerr << "Fail: vpMath::saturate<unsigned char>(-30000)=" << uchar_value << " / should be 0" << std::endl;
    return EXIT_FAILURE;
  }

  unsigned int uint_value = 10000;
  uchar_value = vpMath::saturate<unsigned char>(uint_value);
  if (uchar_value != UCHAR_MAX) {
    std::cerr << "Fail: vpMath::saturate<unsigned char>(10000)=" << uchar_value << " / should be " << UCHAR_MAX
      << std::endl;
    return EXIT_FAILURE;
  }

  float float_value = 10000.1f;
  uchar_value = vpMath::saturate<unsigned char>(float_value);
  if (uchar_value != UCHAR_MAX) {
    std::cerr << "Fail: vpMath::saturate<unsigned char>(10000.1f)=" << uchar_value << " / should be " << UCHAR_MAX
      << std::endl;
    return EXIT_FAILURE;
  }

  float_value = -10000.1f;
  uchar_value = vpMath::saturate<unsigned char>(float_value);
  if (uchar_value != 0) {
    std::cerr << "Fail: vpMath::saturate<unsigned char>(-10000.1f)=" << uchar_value << " / should be 0" << std::endl;
    return EXIT_FAILURE;
  }

  double double_value = 10000.1;
  uchar_value = vpMath::saturate<unsigned char>(double_value);
  if (uchar_value != UCHAR_MAX) {
    std::cerr << "Fail: vpMath::saturate<unsigned char>(10000.0)=" << uchar_value << " / should be " << UCHAR_MAX
      << std::endl;
    return EXIT_FAILURE;
  }

  double_value = -10000.1;
  uchar_value = vpMath::saturate<unsigned char>(double_value);
  if (uchar_value != 0) {
    std::cerr << "Fail: vpMath::saturate<unsigned char>(-10000.0)=" << uchar_value << " / should be 0" << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "vpMath::saturate<unsigned char>() is Ok !" << std::endl;

  // char
  uchar_value = 255;
  char_value = vpMath::saturate<char>(uchar_value);
  if (char_value != SCHAR_MAX) {
    std::cerr << "Fail: vpMath::saturate<char>(255)=" << char_value << " / should be " << SCHAR_MAX << std::endl;
    return EXIT_FAILURE;
  }

  ushort_value = 60000;
  char_value = vpMath::saturate<char>(ushort_value);
  if (char_value != SCHAR_MAX) {
    std::cerr << "Fail: vpMath::saturate<char>(60000)=" << char_value << " / should be " << SCHAR_MAX << std::endl;
    return EXIT_FAILURE;
  }

  int_value = 70000;
  char_value = vpMath::saturate<char>(int_value);
  if (char_value != SCHAR_MAX) {
    std::cerr << "Fail: vpMath::saturate<char>(70000)=" << char_value << " / should be " << SCHAR_MAX << std::endl;
    return EXIT_FAILURE;
  }

  int_value = -70000;
  char_value = vpMath::saturate<char>(int_value);
  if (char_value != (char)SCHAR_MIN) {
    std::cerr << "Fail: vpMath::saturate<char>(-70000)=" << char_value << " / should be " << SCHAR_MIN << std::endl;
    return EXIT_FAILURE;
  }

  short_value = 30000;
  char_value = vpMath::saturate<char>(short_value);
  if (char_value != SCHAR_MAX) {
    std::cerr << "Fail: vpMath::saturate<char>(30000)=" << char_value << " / should be " << SCHAR_MAX << std::endl;
    return EXIT_FAILURE;
  }

  short_value = -30000;
  char_value = vpMath::saturate<char>(short_value);
  if (char_value != (char)SCHAR_MIN) {
    std::cerr << "Fail: vpMath::saturate<char>(-30000)=" << char_value << " / should be " << SCHAR_MIN << std::endl;
    return EXIT_FAILURE;
  }

  uint_value = 10000;
  char_value = vpMath::saturate<char>(uint_value);
  if (char_value != SCHAR_MAX) {
    std::cerr << "Fail: vpMath::saturate<char>(10000)=" << char_value << " / should be " << SCHAR_MAX << std::endl;
    return EXIT_FAILURE;
  }

  float_value = 10000.1f;
  char_value = vpMath::saturate<char>(float_value);
  if (char_value != SCHAR_MAX) {
    std::cerr << "Fail: vpMath::saturate<char>(10000.1f)=" << char_value << " / should be " << SCHAR_MAX << std::endl;
    return EXIT_FAILURE;
  }

  float_value = -10000.1f;
  char_value = vpMath::saturate<char>(float_value);
  if (char_value != (char)SCHAR_MIN) {
    std::cerr << "Fail: vpMath::saturate<char>(-10000.1f)=" << char_value << " / should be " << SCHAR_MIN << std::endl;
    return EXIT_FAILURE;
  }

  double_value = 10000.1;
  char_value = vpMath::saturate<char>(double_value);
  if (char_value != SCHAR_MAX) {
    std::cerr << "Fail: vpMath::saturate<char>(10000.1)=" << char_value << " / should be " << SCHAR_MAX << std::endl;
    return EXIT_FAILURE;
  }

  double_value = -10000.1;
  char_value = vpMath::saturate<char>(double_value);
  if (char_value != (char)SCHAR_MIN) {
    std::cerr << "Fail: vpMath::saturate<char>(-10000.1)=" << char_value << " / should be " << SCHAR_MIN << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "vpMath::saturate<char>() is Ok !" << std::endl;

  // unsigned short
  char_value = -127;
  ushort_value = vpMath::saturate<unsigned short>(char_value);
  if (ushort_value != 0) {
    std::cerr << "Fail: vpMath::saturate<unsigned short>(-127)=" << ushort_value << " / should be 0" << std::endl;
    return EXIT_FAILURE;
  }

  short_value = -30000;
  ushort_value = vpMath::saturate<unsigned short>(short_value);
  if (ushort_value != 0) {
    std::cerr << "Fail: vpMath::saturate<unsigned short>(-30000)=" << ushort_value << " / should be 0" << std::endl;
    return EXIT_FAILURE;
  }

  int_value = 70000;
  ushort_value = vpMath::saturate<unsigned short>(int_value);
  if (ushort_value != USHRT_MAX) {
    std::cerr << "Fail: vpMath::saturate<unsigned short>(70000)=" << ushort_value << " / should be " << USHRT_MAX
      << std::endl;
    return EXIT_FAILURE;
  }

  int_value = -70000;
  ushort_value = vpMath::saturate<unsigned short>(int_value);
  if (ushort_value != 0) {
    std::cerr << "Fail: vpMath::saturate<unsigned short>(-70000)=" << ushort_value << " / should be 0" << std::endl;
    return EXIT_FAILURE;
  }

  uint_value = 70000;
  ushort_value = vpMath::saturate<unsigned short>(uint_value);
  if (ushort_value != USHRT_MAX) {
    std::cerr << "Fail: vpMath::saturate<unsigned short>(70000)=" << ushort_value << " / should be " << USHRT_MAX
      << std::endl;
    return EXIT_FAILURE;
  }

  float_value = 70000.1f;
  ushort_value = vpMath::saturate<unsigned short>(float_value);
  if (ushort_value != USHRT_MAX) {
    std::cerr << "Fail: vpMath::saturate<unsigned short>(70000.1f)=" << ushort_value << " / should be " << USHRT_MAX
      << std::endl;
    return EXIT_FAILURE;
  }

  float_value = -10000.1f;
  ushort_value = vpMath::saturate<unsigned short>(float_value);
  if (ushort_value != 0) {
    std::cerr << "Fail: vpMath::saturate<unsigned short>(-10000.1f)=" << ushort_value << " / should be 0" << std::endl;
    return EXIT_FAILURE;
  }

  double_value = 70000.1;
  ushort_value = vpMath::saturate<unsigned short>(double_value);
  if (ushort_value != USHRT_MAX) {
    std::cerr << "Fail: vpMath::saturate<unsigned short>(70000.1)=" << ushort_value << " / should be " << USHRT_MAX
      << std::endl;
    return EXIT_FAILURE;
  }

  double_value = -10000.1;
  ushort_value = vpMath::saturate<unsigned short>(double_value);
  if (ushort_value != 0) {
    std::cerr << "Fail: vpMath::saturate<unsigned short>(-10000.1)=" << ushort_value << " / should be 0" << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "vpMath::saturate<unsigned short>() is Ok !" << std::endl;

  // short
  ushort_value = 60000;
  short_value = vpMath::saturate<short>(ushort_value);
  if (short_value != SHRT_MAX) {
    std::cerr << "Fail: vpMath::saturate<short>(60000)=" << short_value << " / should be " << SHRT_MAX << std::endl;
    return EXIT_FAILURE;
  }

  int_value = 70000;
  short_value = vpMath::saturate<short>(int_value);
  if (short_value != SHRT_MAX) {
    std::cerr << "Fail: vpMath::saturate<short>(70000)=" << short_value << " / should be " << SHRT_MAX << std::endl;
    return EXIT_FAILURE;
  }

  int_value = -70000;
  short_value = vpMath::saturate<short>(int_value);
  if (short_value != SHRT_MIN) {
    std::cerr << "Fail: vpMath::saturate<short>(-70000)=" << short_value << " / should be " << SHRT_MIN << std::endl;
    return EXIT_FAILURE;
  }

  uint_value = 70000;
  short_value = vpMath::saturate<short>(uint_value);
  if (short_value != SHRT_MAX) {
    std::cerr << "Fail: vpMath::saturate<short>(70000)=" << short_value << " / should be " << SHRT_MAX << std::endl;
    return EXIT_FAILURE;
  }

  float_value = 70000.1f;
  short_value = vpMath::saturate<short>(float_value);
  if (short_value != SHRT_MAX) {
    std::cerr << "Fail: vpMath::saturate<short>(70000.1f)=" << short_value << " / should be " << SHRT_MAX << std::endl;
    return EXIT_FAILURE;
  }

  float_value = -70000.1f;
  short_value = vpMath::saturate<short>(float_value);
  if (short_value != SHRT_MIN) {
    std::cerr << "Fail: vpMath::saturate<short>(-70000.1f)=" << short_value << " / should be " << SHRT_MIN << std::endl;
    return EXIT_FAILURE;
  }

  double_value = 70000.1;
  short_value = vpMath::saturate<short>(double_value);
  if (short_value != SHRT_MAX) {
    std::cerr << "Fail: vpMath::saturate<short>(70000.1)=" << short_value << " / should be " << SHRT_MAX << std::endl;
    return EXIT_FAILURE;
  }

  double_value = -70000.1;
  short_value = vpMath::saturate<short>(double_value);
  if (short_value != SHRT_MIN) {
    std::cerr << "Fail: vpMath::saturate<short>(70000.1)=" << short_value << " / should be " << SHRT_MIN << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "vpMath::saturate<short>() is Ok !" << std::endl;

  // Test mean, median and standard deviation against Matlab with rng(0) and
  // rand(10,1)*10
  std::vector<double> vectorOfDoubles(10);
  vectorOfDoubles[0] = 8.1472;
  vectorOfDoubles[1] = 9.0579;
  vectorOfDoubles[2] = 1.2699;
  vectorOfDoubles[3] = 9.1338;
  vectorOfDoubles[4] = 6.3236;
  vectorOfDoubles[5] = 0.9754;
  vectorOfDoubles[6] = 2.7850;
  vectorOfDoubles[7] = 5.4688;
  vectorOfDoubles[8] = 9.5751;
  vectorOfDoubles[9] = 9.6489;

  double res = vpMath::getMean(vectorOfDoubles);
  if (!vpMath::equal(res, 6.2386, 0.001)) {
    std::cerr << "Problem with vpMath::getMean()=" << res << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "vpMath::getMean() is Ok !" << std::endl;

  res = vpMath::getStdev(vectorOfDoubles);
  if (!vpMath::equal(res, 3.2810, 0.001)) {
    std::cerr << "Problem with vpMath::getStdev()=" << res << std::endl;
    return EXIT_FAILURE;
  }

  res = vpMath::getStdev(vectorOfDoubles, true);
  if (!vpMath::equal(res, 3.4585, 0.001)) {
    std::cerr << "Problem with vpMath::getStdev() with Bessel correction=" << res << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "vpMath::getStdev() is Ok !" << std::endl;

  res = vpMath::getMedian(vectorOfDoubles);
  if (!vpMath::equal(res, 7.2354, 0.001)) {
    std::cerr << "Problem with vpMath::getMedian()=" << res << std::endl;
    return EXIT_FAILURE;
  }

  // Test median with odd number of elements
  vectorOfDoubles.push_back(1.5761);
  res = vpMath::getMedian(vectorOfDoubles);
  if (!vpMath::equal(res, 6.3236, 0.001)) {
    std::cerr << "Problem with vpMath::getMedian()=" << res << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "vpMath::getMedian() is Ok !" << std::endl;

  // Test clamp
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  {
    const double lower { -10. }, upper { +10. };
    std::vector<double> testing_values { 5., -15., 15. };
    std::vector<double> expected_values { 5., -10., 10. };

    for (size_t i = 0u; i < testing_values.size(); i++) {
      const double clamp_val = vpMath::clamp(testing_values.at(i), lower, upper);
      if (!vpMath::equal(clamp_val, expected_values.at(i), 0.001)) {
        std::cerr << "Problem with vpMath::clamp()=" << clamp_val << std::endl;
        return EXIT_FAILURE;
      }
    }
    std::cout << "vpMath::clamp() is Ok !" << std::endl;
  }
#endif

  // Test vpMath::deg() and vpMath::rad()
  {
    vpColVector rxyz_deg_truth(3);
    rxyz_deg_truth[0] = 10;
    rxyz_deg_truth[1] = 20;
    rxyz_deg_truth[2] = -30;
    vpColVector rxyz_rad_truth(3);
    rxyz_rad_truth[0] = vpMath::rad(rxyz_deg_truth[0]);
    rxyz_rad_truth[1] = vpMath::rad(rxyz_deg_truth[1]);
    rxyz_rad_truth[2] = vpMath::rad(rxyz_deg_truth[2]);

    {
      vpRxyzVector rxyz(rxyz_rad_truth);
      vpColVector rxyz_deg = vpMath::deg(rxyz);
      for (unsigned int i = 0u; i < rxyz_deg_truth.size(); i++) {
        if (!vpMath::equal(rxyz_deg[i], rxyz_deg_truth[i], 0.001)) {
          std::cerr << "Problem with vpMath::deg(vpRotationVector) " << i << ": " << rxyz_deg[i] << std::endl;
          return EXIT_FAILURE;
        }
      }
      std::cout << "vpMath::deg(vpRxyzVector) is Ok !" << std::endl;
    }
    {
      vpColVector rxyz_deg = vpMath::deg(rxyz_rad_truth);
      for (unsigned int i = 0u; i < rxyz_deg_truth.size(); i++) {
        if (!vpMath::equal(rxyz_deg[i], rxyz_deg_truth[i], 0.001)) {
          std::cerr << "Problem with vpMath::deg(vpColVector) " << i << ": " << rxyz_deg[i] << std::endl;
          return EXIT_FAILURE;
        }
      }
      std::cout << "vpMath::deg(vpColVector) is Ok !" << std::endl;
    }
    {
      vpColVector rxyz_rad = vpMath::rad(rxyz_deg_truth);
      for (unsigned int i = 0u; i < rxyz_deg_truth.size(); i++) {
        if (!vpMath::equal(rxyz_rad[i], rxyz_rad_truth[i], 0.001)) {
          std::cerr << "Problem with vpMath::rad(vpColVector) " << i << ": " << rxyz_rad[i] << std::endl;
          return EXIT_FAILURE;
        }
      }
      std::cout << "vpMath::rad(vpColVector) is Ok !" << std::endl;
    }
  }

  std::cout << "Test succeed" << std::endl;
  return EXIT_SUCCESS;
}
