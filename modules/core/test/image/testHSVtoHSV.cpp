/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 * Test vpImageConvert::convert() function for HSV to HSV.
 */
/*!
  \example testHSVtoHSV.cpp

  \brief Test vpImageConvert::convert() function for HSV to HSV.
*/
#include <iostream>
#include <limits>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/core/vpHSV.h>

#include "hsvUtils.h"

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

#ifndef DOXYGEN_SHOULD_SKIP_THIS
/**
 * \brief Check if the computed HSV value corresponds to the ground-truth.
 *
 * \tparam useFullScale True if vpHSV uses unsigned char and the full range [0; 255], false if vpHSV uses unsigned char and the limited range [0; vpHSV<unsigned char, false>::maxHueUsingLimitedRange].
 * \param[in] hsv_computed Computed vpRGBa value.
 * \param[in] rgb_truth vpRGBa value  that was used to compute rgb_computed.
 * \param[in] hsv_truth The HSVground-truth value.
 * \return true If hsv_computed and hsv_truth are equal.
 * \return false Otherwise
*/
template<bool useFullScale >
bool test_hsv(const vpHSV<unsigned char, useFullScale> &hsv_computed,
              const vpHSV<unsigned char, useFullScale> &hsv_truth)
{
  // Compare HSV values
  if ((hsv_computed.H != hsv_truth.H) ||
      (hsv_computed.S != hsv_truth.S) ||
      (hsv_computed.V != hsv_truth.V)) {

    std::cout << "Expected hsv value: ("
      << static_cast<int>(hsv_truth.H) << ","
      << static_cast<int>(hsv_truth.S) << ","
      << static_cast<int>(hsv_truth.V) << ") converted value: ("
      << static_cast<int>(hsv_computed.H) << ","
      << static_cast<int>(hsv_computed.S) << ","
      << static_cast<int>(hsv_computed.V) << ")" << std::endl;
    return false;
  }

  return true;
}

/**
 * \brief Check if the computed HSV value corresponds to the ground-truth.
 *
 * \tparam Type The type of the vpHSV channels.
 * \tparam useFullScale True if vpHSV uses unsigned char and the full range [0; 255], false if vpHSV uses unsigned char and the limited range [0; vpHSV<unsigned char, false>::maxHueUsingLimitedRange].
 * \param[in] hsv_computed Computed vpRGBa value.
 * \param[in] rgb_truth vpRGBa value  that was used to compute rgb_computed.
 * \param[in] hsv_truth The HSVground-truth value.
 * \return true If hsv_computed and hsv_truth are equal.
 * \return false Otherwise
 */
template<typename Type, bool useFullScale >
bool test_hsv(const vpHSV<Type, useFullScale> &hsv_computed,
              const vpHSV<Type, useFullScale> &hsv_truth)
{
  // Compare HSV values
  if ((!vpMath::equal(hsv_computed.H, hsv_truth.H)) ||
      (!vpMath::equal(hsv_computed.S, hsv_truth.S)) ||
      (!vpMath::equal(hsv_computed.V, hsv_truth.V))) {

    std::cout << "Expected hsv value: ("
      << static_cast<int>(hsv_truth.H) << ","
      << static_cast<int>(hsv_truth.S) << ","
      << static_cast<int>(hsv_truth.V) << ") converted value: ("
      << static_cast<int>(hsv_computed.H) << ","
      << static_cast<int>(hsv_computed.S) << ","
      << static_cast<int>(hsv_computed.V) << ")" << std::endl;
    return false;
  }

  return true;
}
#endif

int main()
{
  bool isSuccess = true;

  std::cout << std::endl << "----- Testing single pixel HSV to HSV conversions -----" << std::endl;
  vpHSV<unsigned char, false> hsvucff(vpHSV<float>(1., 1., 1.));
  isSuccess = isSuccess && test_hsv(hsvucff, vpHSV<unsigned char, false>(vpHSV<unsigned char, false>::maxHueUsingLimitedRange, 255, 255));

  vpHSV<unsigned char, true > hsvucft(vpHSV<float>(1., 1., 1.));
  isSuccess = isSuccess && test_hsv(hsvucft, vpHSV<unsigned char, true>(255, 255, 255));

  vpHSV<unsigned char, false> hsvucdf(vpHSV<double>(1., 1., 1.));
  isSuccess = isSuccess && test_hsv(hsvucdf, vpHSV<unsigned char, false>(vpHSV<unsigned char, false>::maxHueUsingLimitedRange, 255, 255));

  vpHSV<unsigned char, true > hsvucdt(vpHSV<double>(1., 1., 1.));
  isSuccess = isSuccess && test_hsv(hsvucdt, vpHSV<unsigned char, true>(255, 255, 255));

  vpHSV<float> hsvfucf(vpHSV<unsigned char, false>(vpHSV<unsigned char, false>::maxHueUsingLimitedRange, 255, 255));
  isSuccess = isSuccess && test_hsv(hsvfucf, vpHSV<float>(1., 1., 1.));

  vpHSV<float> hsvfuct(vpHSV<unsigned char, true >(255, 255, 255));
  isSuccess = isSuccess && test_hsv(hsvfuct, vpHSV<float>(1., 1., 1.));

  vpHSV<double> hsvducf(vpHSV<unsigned char, false>(vpHSV<unsigned char, false>::maxHueUsingLimitedRange, 255, 255));
  isSuccess = isSuccess && test_hsv(hsvducf, vpHSV<double>(1., 1., 1.));

  vpHSV<double> hsvduct(vpHSV<unsigned char, true >(255, 255, 255));
  isSuccess = isSuccess && test_hsv(hsvduct, vpHSV<double>(1., 1., 1.));

  std::cout << std::endl << "----- Testing vpImageConvert(HSV, HSV) conversions -----" << std::endl;
  {
    std::cout << std::endl << "\t UCHAR -> floating types" << std::endl;
    vpImage<vpHSV<unsigned char, false>> Iucf(480, 640, vpHSV<unsigned char, false>(vpHSV<unsigned char, false>::maxHueUsingLimitedRange, 255, 255));
    vpImage<vpHSV<float>> Ihff, Ihff_truth(480, 640, vpHSV<float>(1., 1., 1.));
    vpImageConvert::convert(Iucf, Ihff);
    bool localSuccess = vpHSVTests::areAlmostEqual(Ihff, "Ihff", Ihff_truth, "Ihff_truth");
    if (!localSuccess) {
      std::cerr << "vpImageConvert(HSV<uchar, false>, HSV<float>) failed!" << std::endl;
    }
    isSuccess = isSuccess && localSuccess;

    vpImage<vpHSV<unsigned char, true>> Iuct(480, 640, vpHSV<unsigned char, true>(255, 255, 255));
    vpImage<vpHSV<float>> Ihft, Ihft_truth(480, 640, vpHSV<float>(1., 1., 1.));
    vpImageConvert::convert(Iuct, Ihft);
    localSuccess = vpHSVTests::areAlmostEqual(Ihft, "Ihft", Ihft_truth, "Ihft_truth");
    if (!localSuccess) {
      std::cerr << "vpImageConvert(HSV<uchar, true>, HSV<float>) failed!" << std::endl;
    }
    isSuccess = isSuccess && localSuccess;

    vpImage<vpHSV<double>> Ihdf, Ihdf_truth(480, 640, vpHSV<double>(1., 1., 1.));
    vpImageConvert::convert(Iucf, Ihdf);
    localSuccess = vpHSVTests::areAlmostEqual(Ihdf, "Ihdf", Ihdf_truth, "Ihdf_truth");
    if (!localSuccess) {
      std::cerr << "vpImageConvert(HSV<uchar, false>, HSV<double>) failed!" << std::endl;
    }
    isSuccess = isSuccess && localSuccess;

    vpImage<vpHSV<double>> Ihdt, Ihdt_truth(480, 640, vpHSV<double>(1., 1., 1.));
    vpImageConvert::convert(Iuct, Ihdt);
    localSuccess = vpHSVTests::areAlmostEqual(Ihdt, "Ihdt", Ihdt_truth, "Ihdt_truth");
    if (!localSuccess) {
      std::cerr << "vpImageConvert(HSV<uchar, true>, HSV<double>) failed!" << std::endl;
    }
    isSuccess = isSuccess && localSuccess;
  }

  {
    std::cout << std::endl << "\t floating types -> UCHAR" << std::endl;
    vpImage<vpHSV<float>> Ihff(480, 640, vpHSV<float>(1., 1., 1.));
    vpImage<vpHSV<unsigned char, false>> Iucf, Iucf_truth(480, 640, vpHSV<unsigned char, false>(vpHSV<unsigned char, false>::maxHueUsingLimitedRange, 255, 255));
    vpImageConvert::convert(Ihff, Iucf);
    bool localSuccess = vpHSVTests::areAlmostEqual(Iucf, "Iucf", Iucf_truth, "Iucf_truth");
    if (!localSuccess) {
      std::cerr << "vpImageConvert(HSV<float>, HSV<uchar, false>) failed!" << std::endl;
    }
    isSuccess = isSuccess && localSuccess;

    vpImage<vpHSV<float>> Ihft(480, 640, vpHSV<float>(1., 1., 1.));
    vpImage<vpHSV<unsigned char, true>> Iuct, Iuct_truth(480, 640, vpHSV<unsigned char, true>(255, 255, 255));
    vpImageConvert::convert(Ihft, Iuct);
    localSuccess = vpHSVTests::areAlmostEqual(Iuct, "Iuct", Iuct_truth, "Iuct_truth");
    if (!localSuccess) {
      std::cerr << "vpImageConvert(HSV<float>, HSV<uchar, true>) failed!" << std::endl;
    }
    isSuccess = isSuccess && localSuccess;

    vpImage<vpHSV<double>> Ihdf(480, 640, vpHSV<double>(1., 1., 1.));
    vpImageConvert::convert(Ihdf, Iucf);
    localSuccess = vpHSVTests::areAlmostEqual(Iucf, "Iucf", Iucf_truth, "Iucf_truth");
    if (!localSuccess) {
      std::cerr << "vpImageConvert(HSV<double>, HSV<uchar, false>) failed!" << std::endl;
    }
    isSuccess = isSuccess && localSuccess;

    vpImage<vpHSV<double>> Ihdt(480, 640, vpHSV<double>(1., 1., 1.));
    vpImageConvert::convert(Ihdt, Iuct);
    localSuccess = vpHSVTests::areAlmostEqual(Iuct, "Iuct", Iuct_truth, "Iuct_truth");
    if (!localSuccess) {
      std::cerr << "vpImageConvert(HSV<double>, HSV<uchar, true>) failed!" << std::endl;
    }
    isSuccess = isSuccess && localSuccess;
  }

  {
    std::cout << std::endl << "\t floating types -> floating types" << std::endl;
    vpImage<vpHSV<float>> Ifloat_truth(480, 640, vpHSV<float>(1., 1., 1.));
    vpImage<vpHSV<double>> Idouble, Idouble_truth(480, 640, vpHSV<double>(1., 1., 1.));
    vpImageConvert::convert(Ifloat_truth, Idouble);
    bool localSuccess = vpHSVTests::areAlmostEqual(Idouble, "Idouble", Idouble_truth, "Idouble_truth");
    if (!localSuccess) {
      std::cerr << "vpImageConvert(HSV<float>, HSV<double>) failed!" << std::endl;
    }
    isSuccess = isSuccess && localSuccess;

    vpImage<vpHSV<float>> Ifloat;
    vpImageConvert::convert(Idouble_truth, Ifloat);
    localSuccess = vpHSVTests::areAlmostEqual(Ifloat, "Ifloat", Ifloat_truth, "Ifloat_truth");
    if (!localSuccess) {
      std::cerr << "vpImageConvert(HSV<double>, HSV<float>) failed!" << std::endl;
    }
    isSuccess = isSuccess && localSuccess;
  }

  if (isSuccess) {
    std::cout << "All tests were successful !" << std::endl;
    return EXIT_SUCCESS;
  }
  std::cerr << "ERROR: Something went wrong !" << std::endl;
  return EXIT_FAILURE;
}

#else
int main()
{
  std::cout << "vpHSV class is not available, please use CXX 11 standard" << std::endl;
  return EXIT_SUCCESS;
}
#endif
