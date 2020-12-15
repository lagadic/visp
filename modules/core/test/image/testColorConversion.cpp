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
 * Test image conversion.
 *
 *****************************************************************************/

/*!
  \example testColorConversion.cpp

  \brief Test color conversion.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2)
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>
#include <visp3/core/vpImageConvert.h>
#include "common.hpp"

static const double maxMeanPixelError = 1.0;
static const unsigned int width = 223, height = 151;

TEST_CASE("Gray to RGBa conversion", "[image_conversion]") {
  vpImage<unsigned char> gray(height, width);
  common_tools::fill(gray);

  vpImage<vpRGBa> rgba_ref(height, width);
  common_tools::grayToRGBaRef(gray.bitmap, reinterpret_cast<unsigned char*>(rgba_ref.bitmap), gray.getSize());

  vpImage<vpRGBa> rgba;
  vpImageConvert::convert(gray, rgba);
  CHECK((rgba == rgba_ref));
}

TEST_CASE("RGBa to Gray conversion", "[image_conversion]") {
  vpImage<vpRGBa> rgba(height, width);
  common_tools::fill(rgba);

  vpImage<unsigned char> gray_ref(height, width);
  common_tools::RGBaToGrayRef(reinterpret_cast<unsigned char*>(rgba.bitmap), gray_ref.bitmap, rgba.getSize());

  vpImage<unsigned char> gray(height, width);
  vpImageConvert::convert(rgba, gray);
  double error = 0;
  CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
  std::cout << "RGBa to Gray conversion, mean error: " << error << std::endl;
}

TEST_CASE("RGB to Gray conversion", "[image_conversion]") {
  std::vector<unsigned char> rgb(height*width*3);
  common_tools::fill(rgb);

  vpImage<unsigned char> gray_ref(height, width);
  common_tools::RGBToGrayRef(rgb.data(), gray_ref.bitmap, gray_ref.getWidth(), gray_ref.getHeight(), false);

  vpImage<unsigned char> gray(height, width);
  vpImageConvert::RGBToGrey(rgb.data(), gray.bitmap, width, height, false);
  double error = 0;
  CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
  std::cout << "RGB to Gray conversion 1, mean error: " << error << std::endl;

  vpImage<unsigned char> gray2(height, width);
  vpImageConvert::RGBToGrey(rgb.data(), gray2.bitmap, width*height);
  CHECK(common_tools::almostEqual(gray_ref, gray2, maxMeanPixelError, error));
  std::cout << "RGB to Gray conversion 2, mean error: " << error << std::endl;
}

TEST_CASE("RGB <==> RGBa conversion", "[image_conversion]") {
  vpImage<vpRGBa> rgba_ref(height, width);
  common_tools::fill(rgba_ref);

  std::vector<unsigned char> rgb(height*width*3);
  vpImageConvert::RGBaToRGB(reinterpret_cast<unsigned char*>(rgba_ref.bitmap), rgb.data(), rgba_ref.getSize());

  vpImage<vpRGBa> rgba(height, width);
  vpImageConvert::RGBToRGBa(rgb.data(), reinterpret_cast<unsigned char*>(rgba.bitmap), rgba_ref.getSize());

  CHECK((rgba == rgba_ref));
}

#if VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11
TEST_CASE("BGR to Gray conversion", "[image_conversion]") {
  vpImage<vpRGBa> rgba_ref(height, width);
  common_tools::fill(rgba_ref);

  vpImage<unsigned char> gray_ref;
  vpImageConvert::convert(rgba_ref, gray_ref);

  std::vector<unsigned char> bgr;
  common_tools::RGBaToBGR(rgba_ref, bgr);

  vpImage<unsigned char> gray(gray_ref.getHeight(), gray_ref.getWidth());
  vpImageConvert::BGRToGrey(bgr.data(), gray.bitmap, gray.getWidth(), gray.getHeight());

  double error = 0;
  CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
  std::cout << "BGR to Gray conversion, mean error: " << error << std::endl;
}
#endif

TEST_CASE("BGRa to Gray conversion", "[image_conversion]") {
  vpImage<vpRGBa> rgba_ref(height, width);
  common_tools::fill(rgba_ref);

  vpImage<unsigned char> gray_ref;
  vpImageConvert::convert(rgba_ref, gray_ref);

  std::vector<unsigned char> bgra;
  common_tools::RGBaToBGRa(rgba_ref, bgra);

  vpImage<unsigned char> gray(gray_ref.getHeight(), gray_ref.getWidth());
  vpImageConvert::BGRaToGrey(bgra.data(), gray.bitmap, gray.getWidth(), gray.getHeight());

  double error = 0;
  CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
  std::cout << "BGRa to Gray conversion, mean error: " << error << std::endl;
}

TEST_CASE("BGRa to RGBa conversion", "[image_conversion]") {
  vpImage<vpRGBa> rgba_ref(height, width);
  common_tools::fill(rgba_ref);

  std::vector<unsigned char> bgra_ref;
  common_tools::RGBaToBGRa(rgba_ref, bgra_ref);

  vpImage<vpRGBa> rgba(rgba_ref.getHeight(), rgba_ref.getWidth());
  vpImageConvert::BGRaToRGBa(bgra_ref.data(), reinterpret_cast<unsigned char*>(rgba.bitmap), rgba.getWidth(), rgba.getHeight());

  double error = 0;
  CHECK(common_tools::almostEqual(rgba_ref, rgba, maxMeanPixelError, error));
  std::cout << "BGRa to RGBa conversion, mean error: " << error << std::endl;
}

TEST_CASE("Split <==> Merge conversion", "[image_conversion]") {
  vpImage<vpRGBa> rgba_ref(height, width);
  common_tools::fill(rgba_ref);

  vpImage<unsigned char> R, G, B, A;
  vpImageConvert::split(rgba_ref, &R, &G, &B, &A);

  vpImage<vpRGBa> rgba;
  vpImageConvert::merge(&R, &G, &B, &A, rgba);

  CHECK((rgba == rgba_ref));
}

#if VISP_HAVE_OPENCV_VERSION >= 0x020100
TEST_CASE("OpenCV Mat <==> vpImage conversion", "[image_conversion]") {

  SECTION("CV_8UC3 to vpRGBa")
  {
    cv::Mat img(height, width, CV_8UC3);
    common_tools::fill(img);

    vpImage<vpRGBa> rgba_ref(height, width);
    common_tools::BGRToRGBaRef(img.data, reinterpret_cast<unsigned char*>(rgba_ref.bitmap), img.cols, img.rows, false);

    vpImage<vpRGBa> rgba;
    vpImageConvert::convert(img, rgba);
    CHECK((rgba_ref == rgba));
  }

  SECTION("CV_8UC1 to vpRGBa")
  {
    cv::Mat img(height, width, CV_8UC1);
    common_tools::fill(img);

    vpImage<vpRGBa> rgba_ref(height, width);
    common_tools::grayToRGBaRef(img.data, reinterpret_cast<unsigned char*>(rgba_ref.bitmap), height*width);

    vpImage<vpRGBa> rgba;
    vpImageConvert::convert(img, rgba);
    CHECK((rgba_ref == rgba));
  }

  SECTION("CV_8UC3 to unsigned char")
  {
    cv::Mat img(height, width, CV_8UC3);
    common_tools::fill(img);

    vpImage<unsigned char> gray_ref(height, width);
    common_tools::BGRToGrayRef(img.data, gray_ref.bitmap, img.cols, img.rows, false);

    vpImage<unsigned char> gray;
    vpImageConvert::convert(img, gray);
    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "BGR to Gray conversion, mean error: " << error << std::endl;
  }

  SECTION("CV_8UC1 to unsigned char")
  {
    cv::Mat img(height, width, CV_8UC1);
    common_tools::fill(img);

    vpImage<unsigned char> gray;
    vpImageConvert::convert(img, gray);

    REQUIRE(gray.getHeight() == height);
    REQUIRE(gray.getWidth() == width);

    for (int i = 0; i < img.rows; i++) {
      for (int j = 0; j < img.cols; j++) {
        REQUIRE(img.at<uchar>(i, j) == gray[i][j]);
      }
    }
  }
}
#endif

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
int main()
{
  return 0;
}
#endif
