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
 * Test image conversion.
 */

/*!
  \example testColorConversion.cpp

  \brief Test color conversion.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2)
#define CATCH_CONFIG_RUNNER
#include "common.hpp"
#include <catch.hpp>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpEndian.h>
#include <visp3/io/vpImageIo.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

static const double maxMeanPixelError = 1.5; // conversion to gray produce an error = 1.0
static const unsigned int width = 223, height = 151;

TEST_CASE("Gray to RGBa conversion", "[image_conversion]")
{
  SECTION("Image 1x16 (SSE41 aligned=true)")
  {
    unsigned int h = 1, w = 16;
    vpImage<unsigned char> gray(h, w);
    common_tools::fill(gray);

    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::grayToRGBaRef(gray.bitmap, reinterpret_cast<unsigned char *>(rgba_ref.bitmap), gray.getSize());

    vpImage<vpRGBa> rgba;
    vpImageConvert::convert(gray, rgba);
    CHECK((rgba == rgba_ref));
  }
  SECTION("Image 1x17 (SSE41 aligned=false)")
  {
    unsigned int h = 1, w = 17;
    vpImage<unsigned char> gray(h, w);
    common_tools::fill(gray);

    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::grayToRGBaRef(gray.bitmap, reinterpret_cast<unsigned char *>(rgba_ref.bitmap), gray.getSize());

    vpImage<vpRGBa> rgba;
    vpImageConvert::convert(gray, rgba);
    CHECK((rgba == rgba_ref));
  }
  SECTION("Image 1x32 (AVX2 aligned=true)")
  {
    unsigned int h = 1, w = 32;
    vpImage<unsigned char> gray(h, w);
    common_tools::fill(gray);

    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::grayToRGBaRef(gray.bitmap, reinterpret_cast<unsigned char *>(rgba_ref.bitmap), gray.getSize());

    vpImage<vpRGBa> rgba;
    vpImageConvert::convert(gray, rgba);
    CHECK((rgba == rgba_ref));
  }
  SECTION("Image 1x33 (AVX2 aligned=false)")
  {
    unsigned int h = 1, w = 33;
    vpImage<unsigned char> gray(h, w);
    common_tools::fill(gray);

    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::grayToRGBaRef(gray.bitmap, reinterpret_cast<unsigned char *>(rgba_ref.bitmap), gray.getSize());

    vpImage<vpRGBa> rgba;
    vpImageConvert::convert(gray, rgba);
    CHECK((rgba == rgba_ref));
  }
  SECTION("Image 4x64 (general aligned = true")
  {
    unsigned int h = 4, w = 64;
    vpImage<unsigned char> gray(h, w);
    common_tools::fill(gray);

    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::grayToRGBaRef(gray.bitmap, reinterpret_cast<unsigned char *>(rgba_ref.bitmap), gray.getSize());

    vpImage<vpRGBa> rgba;
    vpImageConvert::convert(gray, rgba);
    CHECK((rgba == rgba_ref));
  }
  SECTION("Image 5x65 (general aligned = false")
  {
    unsigned int h = 5, w = 65;
    vpImage<unsigned char> gray(h, w);
    common_tools::fill(gray);

    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::grayToRGBaRef(gray.bitmap, reinterpret_cast<unsigned char *>(rgba_ref.bitmap), gray.getSize());

    vpImage<vpRGBa> rgba;
    vpImageConvert::convert(gray, rgba);
    CHECK((rgba == rgba_ref));
  }
}

TEST_CASE("RGBa to Gray conversion", "[image_conversion]")
{
  SECTION("Image 1x16 (SSE41 aligned=true)")
  {
    unsigned int h = 1, w = 16;
    vpImage<vpRGBa> rgba(h, w);
    common_tools::fill(rgba);

    vpImage<unsigned char> gray_ref(h, w);
    common_tools::RGBaToGrayRef(reinterpret_cast<unsigned char *>(rgba.bitmap), gray_ref.bitmap, rgba.getSize());

    vpImage<unsigned char> gray(h, w);
    vpImageConvert::convert(rgba, gray);
    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "RGBa to Gray conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 1x17 (SSE41 aligned=false)")
  {
    unsigned int h = 1, w = 17;
    vpImage<vpRGBa> rgba(h, w);
    common_tools::fill(rgba);

    vpImage<unsigned char> gray_ref(h, w);
    common_tools::RGBaToGrayRef(reinterpret_cast<unsigned char *>(rgba.bitmap), gray_ref.bitmap, rgba.getSize());

    vpImage<unsigned char> gray(h, w);
    vpImageConvert::convert(rgba, gray);
    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "RGBa to Gray conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 1x32 (AVX2 aligned=true)")
  {
    unsigned int h = 1, w = 32;
    vpImage<vpRGBa> rgba(h, w);
    common_tools::fill(rgba);

    vpImage<unsigned char> gray_ref(h, w);
    common_tools::RGBaToGrayRef(reinterpret_cast<unsigned char *>(rgba.bitmap), gray_ref.bitmap, rgba.getSize());

    vpImage<unsigned char> gray(h, w);
    vpImageConvert::convert(rgba, gray);
    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "RGBa to Gray conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 1x33 (AVX2 aligned=false)")
  {
    unsigned int h = 1, w = 33;
    vpImage<vpRGBa> rgba(h, w);
    common_tools::fill(rgba);

    vpImage<unsigned char> gray_ref(h, w);
    common_tools::RGBaToGrayRef(reinterpret_cast<unsigned char *>(rgba.bitmap), gray_ref.bitmap, rgba.getSize());

    vpImage<unsigned char> gray(h, w);
    vpImageConvert::convert(rgba, gray);
    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "RGBa to Gray conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 4x64 (general aligned = true")
  {
    unsigned int h = 4, w = 64;
    vpImage<vpRGBa> rgba(h, w);
    common_tools::fill(rgba);

    vpImage<unsigned char> gray_ref(h, w);
    common_tools::RGBaToGrayRef(reinterpret_cast<unsigned char *>(rgba.bitmap), gray_ref.bitmap, rgba.getSize());

    vpImage<unsigned char> gray(h, w);
    vpImageConvert::convert(rgba, gray);
    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "RGBa to Gray conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 5x65 (general aligned = false")
  {
    unsigned int h = 5, w = 65;
    vpImage<vpRGBa> rgba(h, w);
    common_tools::fill(rgba);

    vpImage<unsigned char> gray_ref(h, w);
    common_tools::RGBaToGrayRef(reinterpret_cast<unsigned char *>(rgba.bitmap), gray_ref.bitmap, rgba.getSize());

    vpImage<unsigned char> gray(h, w);
    vpImageConvert::convert(rgba, gray);
    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "RGBa to Gray conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
}

TEST_CASE("RGB to Gray conversion", "[image_conversion]")
{
  SECTION("Image 1x16 (SSE41 aligned=true)")
  {
    unsigned int h = 1, w = 16;
    std::vector<unsigned char> rgb(h * w * 3);
    common_tools::fill(rgb);

    vpImage<unsigned char> gray_ref(h, w);
    common_tools::RGBToGrayRef(rgb.data(), gray_ref.bitmap, gray_ref.getWidth(), gray_ref.getHeight(), false);

    vpImage<unsigned char> gray(h, w);
    vpImageConvert::RGBToGrey(rgb.data(), gray.bitmap, w, h, false);
    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "RGB to Gray conversion 1, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;

    vpImage<unsigned char> gray2(h, w);
    vpImageConvert::RGBToGrey(rgb.data(), gray2.bitmap, w * h);
    CHECK(common_tools::almostEqual(gray_ref, gray2, maxMeanPixelError, error));
    std::cout << "RGB to Gray conversion 2, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 1x17 (SSE41 aligned=false)")
  {
    unsigned int h = 1, w = 17;
    std::vector<unsigned char> rgb(h * w * 3);
    common_tools::fill(rgb);

    vpImage<unsigned char> gray_ref(h, w);
    common_tools::RGBToGrayRef(rgb.data(), gray_ref.bitmap, gray_ref.getWidth(), gray_ref.getHeight(), false);

    vpImage<unsigned char> gray(h, w);
    vpImageConvert::RGBToGrey(rgb.data(), gray.bitmap, w, h, false);
    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "RGB to Gray conversion 1, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;

    vpImage<unsigned char> gray2(h, w);
    vpImageConvert::RGBToGrey(rgb.data(), gray2.bitmap, w * h);
    CHECK(common_tools::almostEqual(gray_ref, gray2, maxMeanPixelError, error));
    std::cout << "RGB to Gray conversion 2, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 1x32 (AVX2 aligned=true)")
  {
    unsigned int h = 1, w = 32;
    std::vector<unsigned char> rgb(h * w * 3);
    common_tools::fill(rgb);

    vpImage<unsigned char> gray_ref(h, w);
    common_tools::RGBToGrayRef(rgb.data(), gray_ref.bitmap, gray_ref.getWidth(), gray_ref.getHeight(), false);

    vpImage<unsigned char> gray(h, w);
    vpImageConvert::RGBToGrey(rgb.data(), gray.bitmap, w, h, false);
    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "RGB to Gray conversion 1, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
    vpImage<unsigned char> gray2(h, w);
    vpImageConvert::RGBToGrey(rgb.data(), gray2.bitmap, w * h);
    CHECK(common_tools::almostEqual(gray_ref, gray2, maxMeanPixelError, error));
    std::cout << "RGB to Gray conversion 2, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 1x33 (AVX2 aligned=false)")
  {
    unsigned int h = 1, w = 33;
    std::vector<unsigned char> rgb(h * w * 3);
    common_tools::fill(rgb);

    vpImage<unsigned char> gray_ref(h, w);
    common_tools::RGBToGrayRef(rgb.data(), gray_ref.bitmap, gray_ref.getWidth(), gray_ref.getHeight(), false);

    vpImage<unsigned char> gray(h, w);
    vpImageConvert::RGBToGrey(rgb.data(), gray.bitmap, w, h, false);
    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "RGB to Gray conversion 1, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;

    vpImage<unsigned char> gray2(h, w);
    vpImageConvert::RGBToGrey(rgb.data(), gray2.bitmap, w * h);
    CHECK(common_tools::almostEqual(gray_ref, gray2, maxMeanPixelError, error));
    std::cout << "RGB to Gray conversion 2, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 4x64 (general aligned = true")
  {
    unsigned int h = 4, w = 64;
    std::vector<unsigned char> rgb(h * w * 3);
    common_tools::fill(rgb);

    vpImage<unsigned char> gray_ref(h, w);
    common_tools::RGBToGrayRef(rgb.data(), gray_ref.bitmap, gray_ref.getWidth(), gray_ref.getHeight(), false);

    vpImage<unsigned char> gray(h, w);
    vpImageConvert::RGBToGrey(rgb.data(), gray.bitmap, w, h, false);
    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "RGB to Gray conversion 1, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;

    vpImage<unsigned char> gray2(h, w);
    vpImageConvert::RGBToGrey(rgb.data(), gray2.bitmap, w * h);
    CHECK(common_tools::almostEqual(gray_ref, gray2, maxMeanPixelError, error));
    std::cout << "RGB to Gray conversion 2, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 5x65 (general aligned = false")
  {
    unsigned int h = 5, w = 65;
    std::vector<unsigned char> rgb(h * w * 3);
    common_tools::fill(rgb);

    vpImage<unsigned char> gray_ref(h, w);
    common_tools::RGBToGrayRef(rgb.data(), gray_ref.bitmap, gray_ref.getWidth(), gray_ref.getHeight(), false);

    vpImage<unsigned char> gray(h, w);
    vpImageConvert::RGBToGrey(rgb.data(), gray.bitmap, w, h, false);
    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "RGB to Gray conversion 1, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;

    vpImage<unsigned char> gray2(h, w);
    vpImageConvert::RGBToGrey(rgb.data(), gray2.bitmap, w * h);
    CHECK(common_tools::almostEqual(gray_ref, gray2, maxMeanPixelError, error));
    std::cout << "RGB to Gray conversion 2, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
}

TEST_CASE("RGB <==> RGBa conversion", "[image_conversion]")
{
  SECTION("Image 1x16 (SSE41 aligned=true)")
  {
    unsigned int h = 1, w = 16;
    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::fill(rgba_ref);

    std::vector<unsigned char> rgb(h * w * 3);
    vpImageConvert::RGBaToRGB(reinterpret_cast<unsigned char *>(rgba_ref.bitmap), rgb.data(), rgba_ref.getSize());

    vpImage<vpRGBa> rgba(h, w);
    vpImageConvert::RGBToRGBa(rgb.data(), reinterpret_cast<unsigned char *>(rgba.bitmap), rgba_ref.getSize());

    CHECK((rgba == rgba_ref));
  }
  SECTION("Image 1x17 (SSE41 aligned=false)")
  {
    unsigned int h = 1, w = 17;
    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::fill(rgba_ref);

    std::vector<unsigned char> rgb(h * w * 3);
    vpImageConvert::RGBaToRGB(reinterpret_cast<unsigned char *>(rgba_ref.bitmap), rgb.data(), rgba_ref.getSize());

    vpImage<vpRGBa> rgba(h, w);
    vpImageConvert::RGBToRGBa(rgb.data(), reinterpret_cast<unsigned char *>(rgba.bitmap), rgba_ref.getSize());

    CHECK((rgba == rgba_ref));
  }
  SECTION("Image 1x32 (AVX2 aligned=true)")
  {
    unsigned int h = 1, w = 32;
    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::fill(rgba_ref);

    std::vector<unsigned char> rgb(h * w * 3);
    vpImageConvert::RGBaToRGB(reinterpret_cast<unsigned char *>(rgba_ref.bitmap), rgb.data(), rgba_ref.getSize());

    vpImage<vpRGBa> rgba(h, w);
    vpImageConvert::RGBToRGBa(rgb.data(), reinterpret_cast<unsigned char *>(rgba.bitmap), rgba_ref.getSize());

    CHECK((rgba == rgba_ref));
  }
  SECTION("Image 1x33 (AVX2 aligned=false)")
  {
    unsigned int h = 1, w = 33;
    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::fill(rgba_ref);

    std::vector<unsigned char> rgb(h * w * 3);
    vpImageConvert::RGBaToRGB(reinterpret_cast<unsigned char *>(rgba_ref.bitmap), rgb.data(), rgba_ref.getSize());

    vpImage<vpRGBa> rgba(h, w);
    vpImageConvert::RGBToRGBa(rgb.data(), reinterpret_cast<unsigned char *>(rgba.bitmap), rgba_ref.getSize());

    CHECK((rgba == rgba_ref));
  }
  SECTION("Image 4x64 (general aligned = true")
  {
    unsigned int h = 4, w = 64;
    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::fill(rgba_ref);

    std::vector<unsigned char> rgb(h * w * 3);
    vpImageConvert::RGBaToRGB(reinterpret_cast<unsigned char *>(rgba_ref.bitmap), rgb.data(), rgba_ref.getSize());

    vpImage<vpRGBa> rgba(h, w);
    vpImageConvert::RGBToRGBa(rgb.data(), reinterpret_cast<unsigned char *>(rgba.bitmap), rgba_ref.getSize());

    CHECK((rgba == rgba_ref));
  }
  SECTION("Image 5x65 (general aligned = false")
  {
    unsigned int h = 5, w = 65;
    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::fill(rgba_ref);

    std::vector<unsigned char> rgb(h * w * 3);
    vpImageConvert::RGBaToRGB(reinterpret_cast<unsigned char *>(rgba_ref.bitmap), rgb.data(), rgba_ref.getSize());

    vpImage<vpRGBa> rgba(h, w);
    vpImageConvert::RGBToRGBa(rgb.data(), reinterpret_cast<unsigned char *>(rgba.bitmap), rgba_ref.getSize());

    CHECK((rgba == rgba_ref));
  }
}

TEST_CASE("BGR to Gray conversion", "[image_conversion]")
{
  SECTION("Image 1x16 (SSE41 aligned=true)")
  {
    unsigned int h = 1, w = 16;
    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::fill(rgba_ref);

    vpImage<unsigned char> gray_ref;
    vpImageConvert::convert(rgba_ref, gray_ref);

    std::vector<unsigned char> bgr;
    common_tools::RGBaToBGR(rgba_ref, bgr);

    vpImage<unsigned char> gray(gray_ref.getHeight(), gray_ref.getWidth());
    vpImageConvert::BGRToGrey(bgr.data(), gray.bitmap, gray.getWidth(), gray.getHeight());

    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "BGR to Gray conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 1x17 (SSE41 aligned=false)")
  {
    unsigned int h = 1, w = 17;
    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::fill(rgba_ref);

    vpImage<unsigned char> gray_ref;
    vpImageConvert::convert(rgba_ref, gray_ref);

    std::vector<unsigned char> bgr;
    common_tools::RGBaToBGR(rgba_ref, bgr);

    vpImage<unsigned char> gray(gray_ref.getHeight(), gray_ref.getWidth());
    vpImageConvert::BGRToGrey(bgr.data(), gray.bitmap, gray.getWidth(), gray.getHeight());

    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "BGR to Gray conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 1x32 (AVX2 aligned=true)")
  {
    unsigned int h = 1, w = 32;
    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::fill(rgba_ref);

    vpImage<unsigned char> gray_ref;
    vpImageConvert::convert(rgba_ref, gray_ref);

    std::vector<unsigned char> bgr;
    common_tools::RGBaToBGR(rgba_ref, bgr);

    vpImage<unsigned char> gray(gray_ref.getHeight(), gray_ref.getWidth());
    vpImageConvert::BGRToGrey(bgr.data(), gray.bitmap, gray.getWidth(), gray.getHeight());

    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "BGR to Gray conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 1x33 (AVX2 aligned=false)")
  {
    unsigned int h = 1, w = 33;
    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::fill(rgba_ref);

    vpImage<unsigned char> gray_ref;
    vpImageConvert::convert(rgba_ref, gray_ref);

    std::vector<unsigned char> bgr;
    common_tools::RGBaToBGR(rgba_ref, bgr);

    vpImage<unsigned char> gray(gray_ref.getHeight(), gray_ref.getWidth());
    vpImageConvert::BGRToGrey(bgr.data(), gray.bitmap, gray.getWidth(), gray.getHeight());

    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "BGR to Gray conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 4x64 (general aligned = true")
  {
    unsigned int h = 4, w = 64;
    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::fill(rgba_ref);

    vpImage<unsigned char> gray_ref;
    vpImageConvert::convert(rgba_ref, gray_ref);

    std::vector<unsigned char> bgr;
    common_tools::RGBaToBGR(rgba_ref, bgr);

    vpImage<unsigned char> gray(gray_ref.getHeight(), gray_ref.getWidth());
    vpImageConvert::BGRToGrey(bgr.data(), gray.bitmap, gray.getWidth(), gray.getHeight());

    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "BGR to Gray conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 5x65 (general aligned = false")
  {
    unsigned int h = 5, w = 65;
    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::fill(rgba_ref);

    vpImage<unsigned char> gray_ref;
    vpImageConvert::convert(rgba_ref, gray_ref);

    std::vector<unsigned char> bgr;
    common_tools::RGBaToBGR(rgba_ref, bgr);

    vpImage<unsigned char> gray(gray_ref.getHeight(), gray_ref.getWidth());
    vpImageConvert::BGRToGrey(bgr.data(), gray.bitmap, gray.getWidth(), gray.getHeight());

    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "BGR to Gray conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
}

TEST_CASE("BGRa to Gray conversion", "[image_conversion]")
{
  SECTION("Image 1x16 (SSE41 aligned=true)")
  {
    unsigned int h = 1, w = 16;
    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::fill(rgba_ref);

    vpImage<unsigned char> gray_ref;
    vpImageConvert::convert(rgba_ref, gray_ref);

    std::vector<unsigned char> bgra;
    common_tools::RGBaToBGRa(rgba_ref, bgra);

    vpImage<unsigned char> gray(gray_ref.getHeight(), gray_ref.getWidth());
    vpImageConvert::BGRaToGrey(bgra.data(), gray.bitmap, gray.getWidth(), gray.getHeight());

    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "BGRa to Gray conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 1x17 (SSE41 aligned=false)")
  {
    unsigned int h = 1, w = 17;
    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::fill(rgba_ref);

    vpImage<unsigned char> gray_ref;
    vpImageConvert::convert(rgba_ref, gray_ref);

    std::vector<unsigned char> bgra;
    common_tools::RGBaToBGRa(rgba_ref, bgra);

    vpImage<unsigned char> gray(gray_ref.getHeight(), gray_ref.getWidth());
    vpImageConvert::BGRaToGrey(bgra.data(), gray.bitmap, gray.getWidth(), gray.getHeight());

    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "BGRa to Gray conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 1x32 (AVX2 aligned=true)")
  {
    unsigned int h = 1, w = 32;
    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::fill(rgba_ref);

    vpImage<unsigned char> gray_ref;
    vpImageConvert::convert(rgba_ref, gray_ref);

    std::vector<unsigned char> bgra;
    common_tools::RGBaToBGRa(rgba_ref, bgra);

    vpImage<unsigned char> gray(gray_ref.getHeight(), gray_ref.getWidth());
    vpImageConvert::BGRaToGrey(bgra.data(), gray.bitmap, gray.getWidth(), gray.getHeight());

    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "BGRa to Gray conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 1x33 (AVX2 aligned=false)")
  {
    unsigned int h = 1, w = 33;
    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::fill(rgba_ref);

    vpImage<unsigned char> gray_ref;
    vpImageConvert::convert(rgba_ref, gray_ref);

    std::vector<unsigned char> bgra;
    common_tools::RGBaToBGRa(rgba_ref, bgra);

    vpImage<unsigned char> gray(gray_ref.getHeight(), gray_ref.getWidth());
    vpImageConvert::BGRaToGrey(bgra.data(), gray.bitmap, gray.getWidth(), gray.getHeight());

    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "BGRa to Gray conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 4x64 (general aligned = true")
  {
    unsigned int h = 4, w = 64;
    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::fill(rgba_ref);

    vpImage<unsigned char> gray_ref;
    vpImageConvert::convert(rgba_ref, gray_ref);

    std::vector<unsigned char> bgra;
    common_tools::RGBaToBGRa(rgba_ref, bgra);

    vpImage<unsigned char> gray(gray_ref.getHeight(), gray_ref.getWidth());
    vpImageConvert::BGRaToGrey(bgra.data(), gray.bitmap, gray.getWidth(), gray.getHeight());

    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "BGRa to Gray conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 5x65 (general aligned = false")
  {
    unsigned int h = 5, w = 65;
    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::fill(rgba_ref);

    vpImage<unsigned char> gray_ref;
    vpImageConvert::convert(rgba_ref, gray_ref);

    std::vector<unsigned char> bgra;
    common_tools::RGBaToBGRa(rgba_ref, bgra);

    vpImage<unsigned char> gray(gray_ref.getHeight(), gray_ref.getWidth());
    vpImageConvert::BGRaToGrey(bgra.data(), gray.bitmap, gray.getWidth(), gray.getHeight());

    double error = 0;
    CHECK(common_tools::almostEqual(gray_ref, gray, maxMeanPixelError, error));
    std::cout << "BGRa to Gray conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
}

TEST_CASE("BGRa to RGBa conversion", "[image_conversion]")
{
  SECTION("Image 1x16 (SSE41 aligned=true)")
  {
    unsigned int h = 1, w = 16;
    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::fill(rgba_ref);

    std::vector<unsigned char> bgra_ref;
    common_tools::RGBaToBGRa(rgba_ref, bgra_ref);

    vpImage<vpRGBa> rgba(rgba_ref.getHeight(), rgba_ref.getWidth());
    vpImageConvert::BGRaToRGBa(bgra_ref.data(), reinterpret_cast<unsigned char *>(rgba.bitmap), rgba.getWidth(),
                               rgba.getHeight());
    double error = 0;
    CHECK(common_tools::almostEqual(rgba_ref, rgba, maxMeanPixelError, error));
    std::cout << "BGRa to RGBa conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 1x17 (SSE41 aligned=false)")
  {
    unsigned int h = 1, w = 17;
    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::fill(rgba_ref);

    std::vector<unsigned char> bgra_ref;
    common_tools::RGBaToBGRa(rgba_ref, bgra_ref);

    vpImage<vpRGBa> rgba(rgba_ref.getHeight(), rgba_ref.getWidth());
    vpImageConvert::BGRaToRGBa(bgra_ref.data(), reinterpret_cast<unsigned char *>(rgba.bitmap), rgba.getWidth(),
                               rgba.getHeight());
    double error = 0;
    CHECK(common_tools::almostEqual(rgba_ref, rgba, maxMeanPixelError, error));
    std::cout << "BGRa to RGBa conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 1x32 (AVX2 aligned=true)")
  {
    unsigned int h = 1, w = 32;
    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::fill(rgba_ref);

    std::vector<unsigned char> bgra_ref;
    common_tools::RGBaToBGRa(rgba_ref, bgra_ref);

    vpImage<vpRGBa> rgba(rgba_ref.getHeight(), rgba_ref.getWidth());
    vpImageConvert::BGRaToRGBa(bgra_ref.data(), reinterpret_cast<unsigned char *>(rgba.bitmap), rgba.getWidth(),
                               rgba.getHeight());
    double error = 0;
    CHECK(common_tools::almostEqual(rgba_ref, rgba, maxMeanPixelError, error));
    std::cout << "BGRa to RGBa conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 1x33 (AVX2 aligned=false)")
  {
    unsigned int h = 1, w = 33;
    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::fill(rgba_ref);

    std::vector<unsigned char> bgra_ref;
    common_tools::RGBaToBGRa(rgba_ref, bgra_ref);

    vpImage<vpRGBa> rgba(rgba_ref.getHeight(), rgba_ref.getWidth());
    vpImageConvert::BGRaToRGBa(bgra_ref.data(), reinterpret_cast<unsigned char *>(rgba.bitmap), rgba.getWidth(),
                               rgba.getHeight());
    double error = 0;
    CHECK(common_tools::almostEqual(rgba_ref, rgba, maxMeanPixelError, error));
    std::cout << "BGRa to RGBa conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 4x64 (general aligned = true")
  {
    unsigned int h = 4, w = 64;
    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::fill(rgba_ref);

    std::vector<unsigned char> bgra_ref;
    common_tools::RGBaToBGRa(rgba_ref, bgra_ref);

    vpImage<vpRGBa> rgba(rgba_ref.getHeight(), rgba_ref.getWidth());
    vpImageConvert::BGRaToRGBa(bgra_ref.data(), reinterpret_cast<unsigned char *>(rgba.bitmap), rgba.getWidth(),
                               rgba.getHeight());
    double error = 0;
    CHECK(common_tools::almostEqual(rgba_ref, rgba, maxMeanPixelError, error));
    std::cout << "BGRa to RGBa conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
  SECTION("Image 5x65 (general aligned = false")
  {
    unsigned int h = 5, w = 65;
    vpImage<vpRGBa> rgba_ref(h, w);
    common_tools::fill(rgba_ref);

    std::vector<unsigned char> bgra_ref;
    common_tools::RGBaToBGRa(rgba_ref, bgra_ref);

    vpImage<vpRGBa> rgba(rgba_ref.getHeight(), rgba_ref.getWidth());
    vpImageConvert::BGRaToRGBa(bgra_ref.data(), reinterpret_cast<unsigned char *>(rgba.bitmap), rgba.getWidth(),
                               rgba.getHeight());
    double error = 0;
    CHECK(common_tools::almostEqual(rgba_ref, rgba, maxMeanPixelError, error));
    std::cout << "BGRa to RGBa conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
  }
}

TEST_CASE("Split <==> Merge conversion", "[image_conversion]")
{
  vpImage<vpRGBa> rgba_ref(height, width);
  common_tools::fill(rgba_ref);

  vpImage<unsigned char> R, G, B, A;
  vpImageConvert::split(rgba_ref, &R, &G, &B, &A);

  vpImage<vpRGBa> rgba;
  vpImageConvert::merge(&R, &G, &B, &A, rgba);

  CHECK((rgba == rgba_ref));
}

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
TEST_CASE("OpenCV Mat <==> vpImage conversion", "[image_conversion]")
{
  SECTION("CV_8UC3 to vpRGBa")
  {
    cv::Mat img(height, width, CV_8UC3);
    common_tools::fill(img);

    vpImage<vpRGBa> rgba_ref(height, width);
    common_tools::BGRToRGBaRef(img.data, reinterpret_cast<unsigned char *>(rgba_ref.bitmap), img.cols, img.rows, false);

    vpImage<vpRGBa> rgba;
    vpImageConvert::convert(img, rgba);
    CHECK((rgba_ref == rgba));
  }

  SECTION("CV_8UC1 to vpRGBa")
  {
    cv::Mat img(height, width, CV_8UC1);
    common_tools::fill(img);

    vpImage<vpRGBa> rgba_ref(height, width);
    common_tools::grayToRGBaRef(img.data, reinterpret_cast<unsigned char *>(rgba_ref.bitmap), height * width);

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
    std::cout << "BGR to Gray conversion, mean error: " << error << " max allowed: " << maxMeanPixelError << std::endl;
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

  SECTION("CV_16UC1 to uint16_t")
  {
    // Test when data in cv::Mat is continuous
    unsigned int w = 3, h = 3;
    cv::Mat img = (cv::Mat_<uint16_t>(h, w) << 65, 650, 6500, 65000, 60000, 6000, 600, 60, 6);
    vpImage<uint16_t> gray16;
    vpImageConvert::convert(img, gray16);

    REQUIRE(gray16.getHeight() == h);
    REQUIRE(gray16.getWidth() == w);

    for (int i = 0; i < img.rows; i++) {
      for (int j = 0; j < img.cols; j++) {
        REQUIRE(img.at<uint16_t>(i, j) == gray16[i][j]);
      }
    }

    // Test when data in cv::Mat is discontinuous
    cv::Mat img_col1 = img.col(1);
    vpImage<uint16_t> gray16_col1;
    vpImageConvert::convert(img_col1, gray16_col1);

    REQUIRE(gray16_col1.getHeight() == h);
    REQUIRE(gray16_col1.getWidth() == 1);

    for (int i = 0; i < img_col1.rows; i++) {
      for (int j = 0; j < img_col1.cols; j++) {
        REQUIRE(img_col1.at<uint16_t>(i, j) == gray16_col1[i][j]);
      }
    }
  }
}
#endif

#if (VISP_HAVE_DATASET_VERSION >= 0x030500)
void col2im(const std::vector<uint8_t> &buffer, vpImage<uint8_t> &I_Bayer_8U)
{
  for (unsigned int i = 0; i < I_Bayer_8U.getHeight(); i++) {
    for (unsigned int j = 0; j < I_Bayer_8U.getWidth(); j++) {
      I_Bayer_8U[i][j] = buffer[j * I_Bayer_8U.getHeight() + i];
    }
  }
}

static void col2im(const std::vector<uint16_t> &buffer, vpImage<uint16_t> &I_Bayer_16U)
{
  for (unsigned int i = 0; i < I_Bayer_16U.getHeight(); i++) {
    for (unsigned int j = 0; j < I_Bayer_16U.getWidth(); j++) {
      I_Bayer_16U[i][j] = buffer[j * I_Bayer_16U.getHeight() + i];
    }
  }
}

static void convertTo(const vpImage<uint16_t> &I_RGBA_16U, vpImage<vpRGBa> &I_RGBA_8U, int divisor = 1 << (12 - 8))
{
  for (unsigned int i = 0; i < I_RGBA_8U.getHeight(); i++) {
    for (unsigned int j = 0; j < I_RGBA_8U.getWidth(); j++) {
      I_RGBA_8U[i][j] = vpRGBa(
          vpMath::saturate<unsigned char>(I_RGBA_16U[0][(i * I_RGBA_8U.getWidth() + j) * 4 + 0] / (float)divisor),
          vpMath::saturate<unsigned char>(I_RGBA_16U[0][(i * I_RGBA_8U.getWidth() + j) * 4 + 1] / (float)divisor),
          vpMath::saturate<unsigned char>(I_RGBA_16U[0][(i * I_RGBA_8U.getWidth() + j) * 4 + 2] / (float)divisor));
    }
  }
}

static double computePSNR(const vpImage<vpRGBa> &I_RGBA_8U, const vpImage<vpRGBa> &I_RGBA_8U_ref)
{
  double mse = 0;
  for (unsigned int i = 0; i < I_RGBA_8U.getHeight(); i++) {
    for (unsigned int j = 0; j < I_RGBA_8U.getWidth(); j++) {
      vpColVector err = I_RGBA_8U[i][j] - I_RGBA_8U_ref[i][j];
      mse += vpMath::sqr(err[0]) + vpMath::sqr(err[1]) + vpMath::sqr(err[2]);
    }
  }
  mse /= I_RGBA_8U.getHeight() * I_RGBA_8U.getWidth() * 3;

  return 10 * std::log10(255 * 255 / mse);
}

static bool readBinaryFile(const std::string &filename, std::vector<uint16_t> &buffer)
{
  std::FILE *f = std::fopen(filename.c_str(), "rb");
  CHECK(f != nullptr);
  if (f == nullptr) {
    return false;
  }

  size_t sread = std::fread(&buffer[0], sizeof buffer[0], buffer.size(), f);
  REQUIRE(sread == buffer.size());

#ifdef VISP_BIG_ENDIAN
  std::vector<uint16_t> tmp = buffer;
  for (size_t i = 0; i < tmp.size(); i++) {
    buffer[i] = vpEndian::swap16bits(tmp[i]);
  }
#endif
  std::fclose(f);
  return true;
}

static bool readBinaryFile(const std::string &filename, std::vector<uint8_t> &buffer)
{
  std::FILE *f = std::fopen(filename.c_str(), "rb");
  CHECK(f != nullptr);
  if (f == nullptr) {
    return false;
  }

  size_t sread = std::fread(&buffer[0], sizeof buffer[0], buffer.size(), f);
  REQUIRE(sread == buffer.size());

  std::fclose(f);
  return true;
}

TEST_CASE("Bayer conversion", "[image_conversion]")
{
  // Load original Klimt image
  vpImage<vpRGBa> I_RGBA_8U_ref;
  vpImageIo::read(I_RGBA_8U_ref, vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Klimt/Klimt.ppm"));

  vpImage<vpRGBa> I_RGBA_8U(I_RGBA_8U_ref.getHeight(), I_RGBA_8U_ref.getWidth());
  int height = I_RGBA_8U_ref.getHeight(), width = I_RGBA_8U_ref.getWidth();
  const double min_PSNR_bilinear = 21, min_PSNR_Malvar = 24;

  SECTION("16-bit")
  {
    std::vector<uint16_t> buffer(height * width);
    vpImage<uint16_t> I_Bayer_16U(height, width);
    vpImage<uint16_t> I_RGBA_16U(1, I_Bayer_16U.getHeight() * I_Bayer_16U.getWidth() * 4);

    SECTION("BGGR")
    {
      const std::string filename =
        vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Bayer/Klimt_Bayer_560x558_BGGR_12bits.raw");
      if (readBinaryFile(filename, buffer)) {
        col2im(buffer, I_Bayer_16U);

        SECTION("Bilinear")
        {
          vpImageConvert::demosaicBGGRToRGBaBilinear(I_Bayer_16U.bitmap, I_RGBA_16U.bitmap, I_Bayer_16U.getWidth(),
                                                    I_Bayer_16U.getHeight());

          convertTo(I_RGBA_16U, I_RGBA_8U);
          double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
          std::cout << "16-bit - BGGR - Bilinear - PSNR: " << PSNR << " min required: " << min_PSNR_bilinear << std::endl;
          CHECK(PSNR >= min_PSNR_bilinear);
        }

        SECTION("Malvar")
        {
          vpImageConvert::demosaicBGGRToRGBaMalvar(I_Bayer_16U.bitmap, I_RGBA_16U.bitmap, I_Bayer_16U.getWidth(),
                                                  I_Bayer_16U.getHeight());

          convertTo(I_RGBA_16U, I_RGBA_8U);
          double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
          std::cout << "16-bit - BGGR - Malvar - PSNR: " << PSNR << " min required: " << min_PSNR_Malvar << std::endl;
          CHECK(PSNR >= min_PSNR_Malvar);
        }
      }
    }

    SECTION("GBRG")
    {
      const std::string filename =
        vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Bayer/Klimt_Bayer_560x558_GBRG_12bits.raw");
      if (readBinaryFile(filename, buffer)) {
        col2im(buffer, I_Bayer_16U);

        SECTION("Bilinear")
        {
          vpImageConvert::demosaicGBRGToRGBaBilinear(I_Bayer_16U.bitmap, I_RGBA_16U.bitmap, I_Bayer_16U.getWidth(),
                                                    I_Bayer_16U.getHeight());

          convertTo(I_RGBA_16U, I_RGBA_8U);
          double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
          std::cout << "16-bit - GBRG - Bilinear - PSNR: " << PSNR << " min required: " << min_PSNR_bilinear << std::endl;
          CHECK(PSNR >= min_PSNR_bilinear);
        }

        SECTION("Malvar")
        {
          vpImageConvert::demosaicGBRGToRGBaMalvar(I_Bayer_16U.bitmap, I_RGBA_16U.bitmap, I_Bayer_16U.getWidth(),
                                                  I_Bayer_16U.getHeight());

          convertTo(I_RGBA_16U, I_RGBA_8U);
          double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
          std::cout << "16-bit - GBRG - Malvar - PSNR: " << PSNR << " min required: " << min_PSNR_Malvar << std::endl;
          CHECK(PSNR >= min_PSNR_Malvar);
        }
      }
    }

    SECTION("GRBG")
    {
      const std::string filename =
        vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Bayer/Klimt_Bayer_560x558_GRBG_12bits.raw");
      if (readBinaryFile(filename, buffer)) {
        col2im(buffer, I_Bayer_16U);

        SECTION("Bilinear")
        {
          vpImageConvert::demosaicGRBGToRGBaBilinear(I_Bayer_16U.bitmap, I_RGBA_16U.bitmap, I_Bayer_16U.getWidth(),
                                                    I_Bayer_16U.getHeight());

          convertTo(I_RGBA_16U, I_RGBA_8U);
          double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
          std::cout << "16-bit - GRBG - Bilinear - PSNR: " << PSNR << " min required: " << min_PSNR_bilinear << std::endl;
          CHECK(PSNR >= min_PSNR_bilinear);
        }

        SECTION("Malvar")
        {
          vpImageConvert::demosaicGRBGToRGBaMalvar(I_Bayer_16U.bitmap, I_RGBA_16U.bitmap, I_Bayer_16U.getWidth(),
                                                  I_Bayer_16U.getHeight());

          convertTo(I_RGBA_16U, I_RGBA_8U);
          double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
          std::cout << "16-bit - GRBG - Malvar - PSNR: " << PSNR << " min required: " << min_PSNR_Malvar << std::endl;
          CHECK(PSNR >= min_PSNR_Malvar);
        }
      }
    }

    SECTION("RGGB")
    {
      const std::string filename =
        vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Bayer/Klimt_Bayer_560x558_RGGB_12bits.raw");
      if (readBinaryFile(filename, buffer)) {
        col2im(buffer, I_Bayer_16U);

        SECTION("Bilinear")
        {
          vpImageConvert::demosaicRGGBToRGBaBilinear(I_Bayer_16U.bitmap, I_RGBA_16U.bitmap, I_Bayer_16U.getWidth(),
                                                    I_Bayer_16U.getHeight());

          convertTo(I_RGBA_16U, I_RGBA_8U);
          double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
          std::cout << "16-bit - RGGB - Bilinear - PSNR: " << PSNR << " min required: " << min_PSNR_bilinear << std::endl;
          CHECK(PSNR >= min_PSNR_bilinear);
        }

        SECTION("Malvar")
        {
          vpImageConvert::demosaicRGGBToRGBaMalvar(I_Bayer_16U.bitmap, I_RGBA_16U.bitmap, I_Bayer_16U.getWidth(),
                                                  I_Bayer_16U.getHeight());

          convertTo(I_RGBA_16U, I_RGBA_8U);
          double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
          std::cout << "16-bit - RGGB - Malvar - PSNR: " << PSNR << " min required: " << min_PSNR_Malvar << std::endl;
          CHECK(PSNR >= min_PSNR_Malvar);
        }
      }
    }
  }

  SECTION("8-bit")
  {
    std::vector<uint8_t> buffer(height * width);
    vpImage<uint8_t> I_Bayer_8U(height, width);
    vpImage<vpRGBa> I_RGBA_8U(I_Bayer_8U.getHeight(), I_Bayer_8U.getWidth());

    SECTION("BGGR")
    {
      const std::string filename =
        vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Bayer/Klimt_Bayer_560x558_BGGR_08bits.raw");

      if (readBinaryFile(filename, buffer)) {
        col2im(buffer, I_Bayer_8U);

        SECTION("Bilinear")
        {
          vpImageConvert::demosaicBGGRToRGBaBilinear(I_Bayer_8U.bitmap, reinterpret_cast<uint8_t *>(I_RGBA_8U.bitmap),
                                                    I_Bayer_8U.getWidth(), I_Bayer_8U.getHeight());

          double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
          std::cout << "8-bit - BGGR - Bilinear - PSNR: " << PSNR <<" min required: " << min_PSNR_bilinear <<  std::endl;
          CHECK(PSNR >= min_PSNR_bilinear);
        }

        SECTION("Malvar")
        {
          vpImageConvert::demosaicBGGRToRGBaMalvar(I_Bayer_8U.bitmap, reinterpret_cast<uint8_t *>(I_RGBA_8U.bitmap),
                                                  I_Bayer_8U.getWidth(), I_Bayer_8U.getHeight());

          double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
          std::cout << "8-bit - BGGR - Malvar - PSNR: " << PSNR << " min required: " << min_PSNR_Malvar << std::endl;
          CHECK(PSNR >= min_PSNR_Malvar);
        }
      }
    }

    SECTION("GBRG")
    {
      const std::string filename =
        vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Bayer/Klimt_Bayer_560x558_GBRG_08bits.raw");

      if (readBinaryFile(filename, buffer)) {
        col2im(buffer, I_Bayer_8U);

        SECTION("Bilinear")
        {
          vpImageConvert::demosaicGBRGToRGBaBilinear(I_Bayer_8U.bitmap, reinterpret_cast<uint8_t *>(I_RGBA_8U.bitmap),
                                                    I_Bayer_8U.getWidth(), I_Bayer_8U.getHeight());

          double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
          std::cout << "8-bit - GBRG - Bilinear - PSNR: " << PSNR << " min required: " << min_PSNR_bilinear << std::endl;
          CHECK(PSNR >= min_PSNR_bilinear);
        }

        SECTION("Malvar")
        {
          vpImageConvert::demosaicGBRGToRGBaMalvar(I_Bayer_8U.bitmap, reinterpret_cast<uint8_t *>(I_RGBA_8U.bitmap),
                                                  I_Bayer_8U.getWidth(), I_Bayer_8U.getHeight());

          double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
          std::cout << "8-bit - GBRG - Malvar - PSNR: " << PSNR << " min required: " << min_PSNR_Malvar << std::endl;
          CHECK(PSNR >= min_PSNR_Malvar);
        }
      }
    }

    SECTION("GRBG")
    {
      const std::string filename =
        vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Bayer/Klimt_Bayer_560x558_GRBG_08bits.raw");

      if (readBinaryFile(filename, buffer)) {
        col2im(buffer, I_Bayer_8U);

        SECTION("Bilinear")
        {
          vpImageConvert::demosaicGRBGToRGBaBilinear(I_Bayer_8U.bitmap, reinterpret_cast<uint8_t *>(I_RGBA_8U.bitmap),
                                                    I_Bayer_8U.getWidth(), I_Bayer_8U.getHeight());

          double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
          std::cout << "8-bit - GRBG - Bilinear - PSNR: " << PSNR << " min required: " << min_PSNR_bilinear << std::endl;
          CHECK(PSNR >= min_PSNR_bilinear);
        }

        SECTION("Malvar")
        {
          vpImageConvert::demosaicGRBGToRGBaMalvar(I_Bayer_8U.bitmap, reinterpret_cast<uint8_t *>(I_RGBA_8U.bitmap),
                                                  I_Bayer_8U.getWidth(), I_Bayer_8U.getHeight());

          double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
          std::cout << "8-bit - GRBG - Malvar - PSNR: " << PSNR << " min required: " << min_PSNR_Malvar << std::endl;
          CHECK(PSNR >= min_PSNR_Malvar);
        }
      }
    }

    SECTION("RGGB")
    {
      const std::string filename =
        vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Bayer/Klimt_Bayer_560x558_RGGB_08bits.raw");

      if (readBinaryFile(filename, buffer)) {
        col2im(buffer, I_Bayer_8U);

        SECTION("Bilinear")
        {
          vpImageConvert::demosaicRGGBToRGBaBilinear(I_Bayer_8U.bitmap, reinterpret_cast<uint8_t *>(I_RGBA_8U.bitmap),
                                                    I_Bayer_8U.getWidth(), I_Bayer_8U.getHeight());

          double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
          std::cout << "8-bit - RGGB - Bilinear - PSNR: " << PSNR << " min required: " << min_PSNR_bilinear << std::endl;
          CHECK(PSNR >= min_PSNR_bilinear);
        }

        SECTION("Malvar")
        {
          vpImageConvert::demosaicRGGBToRGBaMalvar(I_Bayer_8U.bitmap, reinterpret_cast<uint8_t *>(I_RGBA_8U.bitmap),
                                                  I_Bayer_8U.getWidth(), I_Bayer_8U.getHeight());

          double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
          std::cout << "8-bit - RGGB - Malvar - PSNR: " << PSNR << " min required: " << min_PSNR_Malvar << std::endl;
          CHECK(PSNR >= min_PSNR_Malvar);
        }
      }
    }
  }
}
#endif

template<typename Type>
bool test_hsv(const std::vector<Type> &hue, const std::vector<Type> &saturation,
              const std::vector<Type> &value, const std::vector< std::vector<unsigned char> > &rgb_truth,
              const std::vector< std::vector<double> > &hsv_truth, size_t step, size_t size, double max_range)
{
  // Compare HSV values
  for (size_t i = 0; i < size; ++i) {
    if (((hue[i]*max_range) != static_cast<Type>(hsv_truth[i][0])) ||
        ((saturation[i]*max_range) != static_cast<Type>(hsv_truth[i][1])) ||
        ((value[i]*max_range) != static_cast<Type>(hsv_truth[i][2]))) {
      if (step == 3) {
        std::cout << "Error in rgb to hsv conversion for rgb (";
      }
      else {
        std::cout << "Error in rgba to hsv conversion for rgba (";
      }
      std::cout << static_cast<int>(rgb_truth[i][0]) << ","
        << static_cast<int>(rgb_truth[i][1]) << ","
        << static_cast<int>(rgb_truth[i][2]) << "): Expected hsv value: ("
        << static_cast<int>(hsv_truth[i][0]) << ","
        << static_cast<int>(hsv_truth[i][1]) << ","
        << static_cast<int>(hsv_truth[i][2]) << ") converted value: ("
        << static_cast<int>(hue[i]) << ","
        << static_cast<int>(saturation[i]) << ","
        << static_cast<int>(value[i]) << ")" << std::endl;
      return false;
    }
  }
  return true;
}

bool test_rgb(const std::vector<unsigned char> &rgb, const std::vector< std::vector<unsigned char> > rgb_truth,
              const std::vector< std::vector<double> > &hsv_truth, size_t step, size_t size, double epsilon = 0.)
{
  // Compare RGB values
  if (epsilon > 0.) {
    for (size_t i = 0; i < size; ++i) {
      if ((!vpMath::equal(rgb[i*step], rgb_truth[i][0], epsilon)) ||
          (!vpMath::equal(rgb[i*step+1], rgb_truth[i][1], epsilon)) ||
          (!vpMath::equal(rgb[i*step+2], rgb_truth[i][2], epsilon))) {
        std::cout << "Error in hsv to rgb conversion for hsv ("
          << static_cast<int>(hsv_truth[i][0]) << ","
          << static_cast<int>(hsv_truth[i][1]) << ","
          << static_cast<int>(hsv_truth[i][2]) << "): Expected rgb value: ("
          << static_cast<int>(rgb_truth[i][0]) << ","
          << static_cast<int>(rgb_truth[i][1]) << ","
          << static_cast<int>(rgb_truth[i][2]) << ") converted value: ("
          << static_cast<int>(rgb[i*step]) << ","
          << static_cast<int>(rgb[(i*step)+1]) << ","
          << static_cast<int>(rgb[(i*step)+2]) << ") epsilon: " << epsilon << std::endl;
        return false;
      }
    }
  }
  else {
    for (size_t i = 0; i < size; ++i) {
      if ((rgb[i*step] != rgb_truth[i][0]) || (rgb[i*step+1] != rgb_truth[i][1]) || (rgb[i*step+2] != rgb_truth[i][2])) {
        std::cout << "Error in hsv to rgb conversion for hsv ("
          << static_cast<int>(hsv_truth[i][0]) << ","
          << static_cast<int>(hsv_truth[i][1]) << ","
          << static_cast<int>(hsv_truth[i][2]) << "): Expected rgb value: ("
          << static_cast<int>(rgb_truth[i][0]) << ","
          << static_cast<int>(rgb_truth[i][1]) << ","
          << static_cast<int>(rgb_truth[i][2]) << ") converted value: ("
          << static_cast<int>(rgb[i*step]) << ","
          << static_cast<int>(rgb[(i*step)+1]) << ","
          << static_cast<int>(rgb[(i*step)+2]) << ")" << std::endl;
        return false;
      }
    }
  }

  return true;
}

TEST_CASE("RGB to HSV conversion", "[image_conversion]")
{
  std::vector< std::vector<unsigned char> > rgb_truth;
  rgb_truth.push_back({ 0, 0, 0 });
  rgb_truth.push_back({ 255, 255, 255 });
  rgb_truth.push_back({ 255, 0, 0 });
  rgb_truth.push_back({ 0, 255, 0 });
  rgb_truth.push_back({ 0, 0, 255 });
  rgb_truth.push_back({ 255, 255, 0 });
  rgb_truth.push_back({ 0, 255, 255 });
  rgb_truth.push_back({ 255, 0, 255 });
  rgb_truth.push_back({ 128, 128, 128 });
  rgb_truth.push_back({ 128, 128, 0 });
  rgb_truth.push_back({ 128, 0, 0 });
  rgb_truth.push_back({ 0, 128, 0 });
  rgb_truth.push_back({ 0, 128, 128 });
  rgb_truth.push_back({ 0, 0, 128 });
  rgb_truth.push_back({ 128, 0, 128 });

  double h_max;
  bool h_full;

  for (size_t test = 0; test < 2; ++test) {
    if (test == 0) {
      h_max = 255;
      h_full = true;
    }
    else {
      h_max = 180;
      h_full = false;
    }
    std::vector< std::vector<double> > hsv_truth;
    // See https://www.rapidtables.com/convert/color/hsv-to-rgb.html
    hsv_truth.push_back({ 0, 0, 0 });
    hsv_truth.push_back({ 0, 0, 255 });
    hsv_truth.push_back({ 0, 255, 255 });
    hsv_truth.push_back({ h_max * 120 / 360, 255, 255 });
    hsv_truth.push_back({ h_max * 240 / 360, 255, 255 });
    hsv_truth.push_back({ h_max * 60 / 360, 255, 255 });
    hsv_truth.push_back({ h_max * 180 / 360, 255, 255 });
    hsv_truth.push_back({ h_max * 300 / 360, 255, 255 });
    hsv_truth.push_back({ 0, 0, 128 });
    hsv_truth.push_back({ h_max * 60 / 360, 255, 128 });
    hsv_truth.push_back({ 0, 255, 128 });
    hsv_truth.push_back({ h_max * 120 / 360, 255, 128 });
    hsv_truth.push_back({ h_max * 180 / 360, 255, 128 });
    hsv_truth.push_back({ h_max * 240 / 360, 255, 128 });
    hsv_truth.push_back({ h_max * 300 / 360, 255, 128 });

    size_t size = rgb_truth.size();

    std::vector<unsigned char> rgb_truth_continuous;
    for (size_t i = 0; i < size; ++i) {
      for (size_t j = 0; j < rgb_truth[i].size(); ++j) {
        rgb_truth_continuous.push_back(rgb_truth[i][j]);
      }
    }
    std::vector<unsigned char> rgba_truth_continuous;
    for (size_t i = 0; i < size; ++i) {
      for (size_t j = 0; j < rgb_truth[i].size(); ++j) {
        rgba_truth_continuous.push_back(rgb_truth[i][j]);
      }
      rgba_truth_continuous.push_back(vpRGBa::alpha_default);
    }
    SECTION("RGB -> HSV (unsigned char) -> RGB")
    {
      std::vector<unsigned char> hue(size);
      std::vector<unsigned char> saturation(size);
      std::vector<unsigned char> value(size);
      std::cout << "Test rgb -> hsv (unsigned char) conversion with h full scale: " << (h_full ? "yes" : "no") << std::endl;
      vpImageConvert::RGBToHSV(reinterpret_cast<unsigned char *>(&rgb_truth_continuous.front()),
                                reinterpret_cast<unsigned char *>(&hue.front()),
                                reinterpret_cast<unsigned char *>(&saturation.front()),
                                reinterpret_cast<unsigned char *>(&value.front()), size, h_full);
      CHECK(test_hsv(hue, saturation, value, rgb_truth, hsv_truth, 3, size, 1.));

      std::cout << "Test hsv (unsigned char) -> rgb conversion with h full scale: " << (h_full ? "yes" : "no") << std::endl;
      std::vector< unsigned char> rgb_continuous(rgb_truth_continuous.size() * 3);
      vpImageConvert::HSVToRGB(&hue.front(), &saturation.front(), &value.front(), &rgb_continuous.front(), size, h_full);
      CHECK(test_rgb(rgb_continuous, rgb_truth, hsv_truth, 3, size, 5.));
    }
    SECTION("RGBa -> HSV (unsigned char) -> RGBa")
    {
      std::vector<unsigned char> hue(size);
      std::vector<unsigned char> saturation(size);
      std::vector<unsigned char> value(size);
      std::cout << "Test rgba -> hsv (unsigned char) conversion with h full scale: " << (h_full ? "yes" : "no") << std::endl;
      vpImageConvert::RGBaToHSV(reinterpret_cast<unsigned char *>(&rgba_truth_continuous.front()),
                                reinterpret_cast<unsigned char *>(&hue.front()),
                                reinterpret_cast<unsigned char *>(&saturation.front()),
                                reinterpret_cast<unsigned char *>(&value.front()), size, h_full);
      CHECK(test_hsv(hue, saturation, value, rgb_truth, hsv_truth, 4, size, 1.));

      std::cout << "Test hsv (unsigned char) -> rgba conversion with h full scale: " << (h_full ? "yes" : "no") << std::endl;
      std::vector< unsigned char> rgba_continuous(rgb_truth_continuous.size() * 4);
      vpImageConvert::HSVToRGBa(&hue.front(), &saturation.front(), &value.front(), &rgba_continuous.front(), size, h_full);
      CHECK(test_rgb(rgba_continuous, rgb_truth, hsv_truth, 4, size, 5.));
    }
    if (h_full) {
      SECTION("RGB -> HSV (double) -> RGB")
      {
        std::vector<double> hue(size);
        std::vector<double> saturation(size);
        std::vector<double> value(size);
        std::cout << "Test rgb -> hsv (double) conversion" << std::endl;
        vpImageConvert::RGBToHSV(reinterpret_cast<unsigned char *>(&rgb_truth_continuous.front()),
                                  reinterpret_cast<double *>(&hue.front()),
                                  reinterpret_cast<double *>(&saturation.front()),
                                  reinterpret_cast<double *>(&value.front()), size);
        CHECK(test_hsv(hue, saturation, value, rgb_truth, hsv_truth, 3, size, 255.));

        std::cout << "Test hsv (double) -> rgb conversion" << std::endl;
        std::vector< unsigned char> rgb_continuous(rgb_truth_continuous.size());
        vpImageConvert::HSVToRGB(&hue.front(), &saturation.front(), &value.front(), &rgb_continuous.front(), size);
        CHECK(test_rgb(rgb_continuous, rgb_truth, hsv_truth, 3, size));
      }
    }

    if (h_full) {
      SECTION("RGBa -> HSV (double) -> RGBa")
      {
        std::vector<double> hue(size);
        std::vector<double> saturation(size);
        std::vector<double> value(size);
        std::cout << "Test rgba -> hsv (double) conversion" << std::endl;
        vpImageConvert::RGBaToHSV(reinterpret_cast<unsigned char *>(&rgba_truth_continuous.front()),
                                  reinterpret_cast<double *>(&hue.front()),
                                  reinterpret_cast<double *>(&saturation.front()),
                                  reinterpret_cast<double *>(&value.front()), size);
        CHECK(test_hsv(hue, saturation, value, rgb_truth, hsv_truth, 4, size, 255.));

        std::cout << "Test hsv (double) -> rgba conversion" << std::endl;
        std::vector< unsigned char> rgba_continuous(rgb_truth_continuous.size()*4);
        vpImageConvert::HSVToRGBa(&hue.front(), &saturation.front(), &value.front(), &rgba_continuous.front(), size);
        CHECK(test_rgb(rgba_continuous, rgb_truth, hsv_truth, 4, size));
      }
    }
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
  std::cout << (numFailed ? "Test failed" : "Test succeed") << std::endl;
  return numFailed;
}
#else
int main() { return EXIT_SUCCESS; }
#endif
