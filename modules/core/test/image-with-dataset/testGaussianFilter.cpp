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
 * Test Gaussian filter.
 */

/*!
  \example testGaussianFilter.cpp

  \brief Test Gaussian filter.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_SIMDLIB) && defined(VISP_HAVE_CATCH2) && (VISP_HAVE_DATASET_VERSION >= 0x030400)
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>
#include <visp3/core/vpGaussianFilter.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

TEST_CASE("Test vpGaussianFilter (unsigned char)")
{
  const std::string filepath = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Klimt/Klimt.pgm");
  vpImage<unsigned char> I;
  vpImageIo::read(I, filepath);

  std::vector<float> sigmas = { 0.5f, 2.0f, 5.0f, 7.0f };
  for (auto sigma : sigmas) {
    vpGaussianFilter gaussianFilter(I.getWidth(), I.getHeight(), sigma);

    vpImage<unsigned char> I_blurred;
    gaussianFilter.apply(I, I_blurred);

    vpImage<unsigned char> I_blurred_ref;
    const std::string filepath_ref = vpIoTools::createFilePath(
        vpIoTools::getViSPImagesDataPath(), "Gaussian-filter/Klimt_gray_Gaussian_blur_sigma=%.1f.png");
    char buffer[FILENAME_MAX];
    snprintf(buffer, FILENAME_MAX, filepath_ref.c_str(), sigma);
    const std::string filename = buffer;
    vpImageIo::read(I_blurred_ref, filename);

    vpImage<unsigned char> I_diff;
    vpImageTools::imageDifferenceAbsolute(I_blurred, I_blurred_ref, I_diff);
    vpImage<double> I_diff_dbl;
    vpImageConvert::convert(I_diff, I_diff_dbl);
    std::cout << "sigma: " << sigma << " ; I_diff_dbl: " << I_diff_dbl.getMeanValue() << std::endl;
    const double threshold = 1.5;
    CHECK(I_diff_dbl.getMeanValue() < threshold);
  }
}

TEST_CASE("Test vpGaussianFilter (vpRGBa)")
{
  const std::string filepath = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Klimt/Klimt.ppm");
  vpImage<vpRGBa> I;
  vpImageIo::read(I, filepath);

  std::vector<float> sigmas = { 0.5f, 2.0f, 5.0f, 7.0f };
  for (auto sigma : sigmas) {
    vpGaussianFilter gaussianFilter(I.getWidth(), I.getHeight(), sigma);

    vpImage<vpRGBa> I_blurred;
    gaussianFilter.apply(I, I_blurred);

    vpImage<vpRGBa> I_blurred_ref;
    const std::string filepath_ref = vpIoTools::createFilePath(
        vpIoTools::getViSPImagesDataPath(), "Gaussian-filter/Klimt_RGB_Gaussian_blur_sigma=%.1f.png");
    char buffer[FILENAME_MAX];
    snprintf(buffer, FILENAME_MAX, filepath_ref.c_str(), sigma);
    const std::string filename = buffer;
    vpImageIo::read(I_blurred_ref, filename);

    vpImage<vpRGBa> I_diff;
    vpImageTools::imageDifferenceAbsolute(I_blurred, I_blurred_ref, I_diff);
    vpImage<unsigned char> I_diff_R, I_diff_G, I_diff_B;
    vpImageConvert::split(I_diff, &I_diff_R, &I_diff_G, &I_diff_B);

    vpImage<double> I_diff_R_dbl, I_diff_G_dbl, I_diff_B_dbl;
    vpImageConvert::convert(I_diff_R, I_diff_R_dbl);
    vpImageConvert::convert(I_diff_G, I_diff_G_dbl);
    vpImageConvert::convert(I_diff_B, I_diff_B_dbl);

    std::cout << "sigma: " << sigma << " ; I_diff_R_dbl: " << I_diff_R_dbl.getMeanValue()
      << " ; I_diff_G_dbl: " << I_diff_G_dbl.getMeanValue()
      << " ; I_diff_B_dbl: " << I_diff_B_dbl.getMeanValue() << std::endl;
    const double threshold = 1.5;
    CHECK(I_diff_R_dbl.getMeanValue() < threshold);
    CHECK(I_diff_G_dbl.getMeanValue() < threshold);
    CHECK(I_diff_B_dbl.getMeanValue() < threshold);
  }
}

TEST_CASE("Test vpGaussianFilter (vpRGBa + deinterleave)")
{
  const std::string filepath = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Klimt/Klimt.ppm");
  vpImage<vpRGBa> I;
  vpImageIo::read(I, filepath);

  std::vector<float> sigmas = { 0.5f, 2.0f, 5.0f, 7.0f };
  for (auto sigma : sigmas) {
    const bool deinterleave = true;
    vpGaussianFilter gaussianFilter(I.getWidth(), I.getHeight(), sigma, deinterleave);

    vpImage<vpRGBa> I_blurred;
    gaussianFilter.apply(I, I_blurred);

    vpImage<vpRGBa> I_blurred_ref;
    const std::string filepath_ref = vpIoTools::createFilePath(
        vpIoTools::getViSPImagesDataPath(), "Gaussian-filter/Klimt_RGB_Gaussian_blur_sigma=%.1f.png");
    char buffer[FILENAME_MAX];
    snprintf(buffer, FILENAME_MAX, filepath_ref.c_str(), sigma);
    const std::string filename = buffer;
    vpImageIo::read(I_blurred_ref, filename);

    vpImage<vpRGBa> I_diff;
    vpImageTools::imageDifferenceAbsolute(I_blurred, I_blurred_ref, I_diff);
    vpImage<unsigned char> I_diff_R, I_diff_G, I_diff_B;
    vpImageConvert::split(I_diff, &I_diff_R, &I_diff_G, &I_diff_B);

    vpImage<double> I_diff_R_dbl, I_diff_G_dbl, I_diff_B_dbl;
    vpImageConvert::convert(I_diff_R, I_diff_R_dbl);
    vpImageConvert::convert(I_diff_G, I_diff_G_dbl);
    vpImageConvert::convert(I_diff_B, I_diff_B_dbl);

    std::cout << "sigma: " << sigma << " ; I_diff_R_dbl: " << I_diff_R_dbl.getMeanValue()
      << " ; I_diff_G_dbl: " << I_diff_G_dbl.getMeanValue()
      << " ; I_diff_B_dbl: " << I_diff_B_dbl.getMeanValue() << std::endl;
    const double threshold = 1.5;
    CHECK(I_diff_R_dbl.getMeanValue() < threshold);
    CHECK(I_diff_G_dbl.getMeanValue() < threshold);
    CHECK(I_diff_B_dbl.getMeanValue() < threshold);
  }
}

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance

  // Let Catch (using Clara) parse the command line
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();

  // numFailed is clamped to 255 as some unices only use the lower 8 bits.
  // This clamping has already been applied, so just return it here
  // You can also do any post run clean-up here
  return numFailed;
}
#else
int main() { return EXIT_SUCCESS; }
#endif
