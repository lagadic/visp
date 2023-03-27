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
#include "common.hpp"
#include <catch.hpp>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>

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
    std::cout << "RGBa to Gray conversion, mean error: " << error << std::endl;
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
    std::cout << "RGBa to Gray conversion, mean error: " << error << std::endl;
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
    std::cout << "RGBa to Gray conversion, mean error: " << error << std::endl;
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
    std::cout << "RGBa to Gray conversion, mean error: " << error << std::endl;
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
    std::cout << "RGBa to Gray conversion, mean error: " << error << std::endl;
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
    std::cout << "RGBa to Gray conversion, mean error: " << error << std::endl;
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
    std::cout << "RGB to Gray conversion 1, mean error: " << error << std::endl;

    vpImage<unsigned char> gray2(h, w);
    vpImageConvert::RGBToGrey(rgb.data(), gray2.bitmap, w * h);
    CHECK(common_tools::almostEqual(gray_ref, gray2, maxMeanPixelError, error));
    std::cout << "RGB to Gray conversion 2, mean error: " << error << std::endl;
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
    std::cout << "RGB to Gray conversion 1, mean error: " << error << std::endl;

    vpImage<unsigned char> gray2(h, w);
    vpImageConvert::RGBToGrey(rgb.data(), gray2.bitmap, w * h);
    CHECK(common_tools::almostEqual(gray_ref, gray2, maxMeanPixelError, error));
    std::cout << "RGB to Gray conversion 2, mean error: " << error << std::endl;
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
    std::cout << "RGB to Gray conversion 1, mean error: " << error << std::endl;
    vpImage<unsigned char> gray2(h, w);
    vpImageConvert::RGBToGrey(rgb.data(), gray2.bitmap, w * h);
    CHECK(common_tools::almostEqual(gray_ref, gray2, maxMeanPixelError, error));
    std::cout << "RGB to Gray conversion 2, mean error: " << error << std::endl;
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
    std::cout << "RGB to Gray conversion 1, mean error: " << error << std::endl;

    vpImage<unsigned char> gray2(h, w);
    vpImageConvert::RGBToGrey(rgb.data(), gray2.bitmap, w * h);
    CHECK(common_tools::almostEqual(gray_ref, gray2, maxMeanPixelError, error));
    std::cout << "RGB to Gray conversion 2, mean error: " << error << std::endl;
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
    std::cout << "RGB to Gray conversion 1, mean error: " << error << std::endl;

    vpImage<unsigned char> gray2(h, w);
    vpImageConvert::RGBToGrey(rgb.data(), gray2.bitmap, w * h);
    CHECK(common_tools::almostEqual(gray_ref, gray2, maxMeanPixelError, error));
    std::cout << "RGB to Gray conversion 2, mean error: " << error << std::endl;
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
    std::cout << "RGB to Gray conversion 1, mean error: " << error << std::endl;

    vpImage<unsigned char> gray2(h, w);
    vpImageConvert::RGBToGrey(rgb.data(), gray2.bitmap, w * h);
    CHECK(common_tools::almostEqual(gray_ref, gray2, maxMeanPixelError, error));
    std::cout << "RGB to Gray conversion 2, mean error: " << error << std::endl;
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
    std::cout << "BGR to Gray conversion, mean error: " << error << std::endl;
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
    std::cout << "BGR to Gray conversion, mean error: " << error << std::endl;
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
    std::cout << "BGR to Gray conversion, mean error: " << error << std::endl;
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
    std::cout << "BGR to Gray conversion, mean error: " << error << std::endl;
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
    std::cout << "BGR to Gray conversion, mean error: " << error << std::endl;
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
    std::cout << "BGR to Gray conversion, mean error: " << error << std::endl;
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
    std::cout << "BGRa to Gray conversion, mean error: " << error << std::endl;
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
    std::cout << "BGRa to Gray conversion, mean error: " << error << std::endl;
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
    std::cout << "BGRa to Gray conversion, mean error: " << error << std::endl;
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
    std::cout << "BGRa to Gray conversion, mean error: " << error << std::endl;
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
    std::cout << "BGRa to Gray conversion, mean error: " << error << std::endl;
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
    std::cout << "BGRa to Gray conversion, mean error: " << error << std::endl;
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
    std::cout << "BGRa to RGBa conversion, mean error: " << error << std::endl;
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
    std::cout << "BGRa to RGBa conversion, mean error: " << error << std::endl;
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
    std::cout << "BGRa to RGBa conversion, mean error: " << error << std::endl;
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
    std::cout << "BGRa to RGBa conversion, mean error: " << error << std::endl;
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
    std::cout << "BGRa to RGBa conversion, mean error: " << error << std::endl;
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
    std::cout << "BGRa to RGBa conversion, mean error: " << error << std::endl;
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

#if VISP_HAVE_OPENCV_VERSION >= 0x020100
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

void col2im(const std::vector<uint8_t> &buffer, vpImage<uint8_t> &I_Bayer_8U)
{
  for (unsigned int i = 0; i < I_Bayer_8U.getHeight(); i++) {
    for (unsigned int j = 0; j < I_Bayer_8U.getWidth(); j++) {
      I_Bayer_8U[i][j] = buffer[j * I_Bayer_8U.getHeight() + i];
    }
  }
}

void col2im(const std::vector<uint16_t> &buffer, vpImage<uint16_t> &I_Bayer_16U)
{
  for (unsigned int i = 0; i < I_Bayer_16U.getHeight(); i++) {
    for (unsigned int j = 0; j < I_Bayer_16U.getWidth(); j++) {
      I_Bayer_16U[i][j] = buffer[j * I_Bayer_16U.getHeight() + i];
    }
  }
}

void convertTo(const vpImage<uint16_t> &I_RGBA_16U, vpImage<vpRGBa> &I_RGBA_8U, int divisor = 1 << (12 - 8))
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

double computePSNR(const vpImage<vpRGBa> &I_RGBA_8U, const vpImage<vpRGBa> &I_RGBA_8U_ref)
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

void readBinaryFile(const std::string &filename, std::vector<uint16_t> &buffer)
{
  std::FILE *f = std::fopen(filename.c_str(), "rb");
  size_t sread = std::fread(&buffer[0], sizeof buffer[0], buffer.size(), f);
  REQUIRE(sread == buffer.size());

#ifdef VISP_BIG_ENDIAN
  std::vector<uint16_t> tmp = buffer;
  for (size_t i = 0; i < tmp.size(); i++) {
    buffer[i] = vpEndian::swap16bits(tmp[i]);
  }
#endif
  std::fclose(f);
}

#if (VISP_HAVE_DATASET_VERSION >= 0x040500)
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
      readBinaryFile(filename, buffer);

      col2im(buffer, I_Bayer_16U);

      SECTION("Bilinear")
      {
        vpImageConvert::demosaicBGGRToRGBaBilinear(I_Bayer_16U.bitmap, I_RGBA_16U.bitmap, I_Bayer_16U.getWidth(),
                                                   I_Bayer_16U.getHeight());

        convertTo(I_RGBA_16U, I_RGBA_8U);
        double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
        std::cout << "16-bit - BGGR - Bilinear - PSNR: " << PSNR << std::endl;
        CHECK(PSNR >= min_PSNR_bilinear);
      }

      SECTION("Malvar")
      {
        vpImageConvert::demosaicBGGRToRGBaMalvar(I_Bayer_16U.bitmap, I_RGBA_16U.bitmap, I_Bayer_16U.getWidth(),
                                                 I_Bayer_16U.getHeight());

        convertTo(I_RGBA_16U, I_RGBA_8U);
        double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
        std::cout << "16-bit - BGGR - Malvar - PSNR: " << PSNR << std::endl;
        CHECK(PSNR >= min_PSNR_Malvar);
      }
    }

    SECTION("GBRG")
    {
      const std::string filename =
          vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Bayer/Klimt_Bayer_560x558_GBRG_12bits.raw");
      readBinaryFile(filename, buffer);

      col2im(buffer, I_Bayer_16U);

      SECTION("Bilinear")
      {
        vpImageConvert::demosaicGBRGToRGBaBilinear(I_Bayer_16U.bitmap, I_RGBA_16U.bitmap, I_Bayer_16U.getWidth(),
                                                   I_Bayer_16U.getHeight());

        convertTo(I_RGBA_16U, I_RGBA_8U);
        double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
        std::cout << "16-bit - GBRG - Bilinear - PSNR: " << PSNR << std::endl;
        CHECK(PSNR >= min_PSNR_bilinear);
      }

      SECTION("Malvar")
      {
        vpImageConvert::demosaicGBRGToRGBaMalvar(I_Bayer_16U.bitmap, I_RGBA_16U.bitmap, I_Bayer_16U.getWidth(),
                                                 I_Bayer_16U.getHeight());

        convertTo(I_RGBA_16U, I_RGBA_8U);
        double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
        std::cout << "16-bit - GBRG - Malvar - PSNR: " << PSNR << std::endl;
        CHECK(PSNR >= min_PSNR_Malvar);
      }
    }

    SECTION("GRBG")
    {
      const std::string filename =
          vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Bayer/Klimt_Bayer_560x558_GRBG_12bits.raw");
      readBinaryFile(filename, buffer);

      col2im(buffer, I_Bayer_16U);

      SECTION("Bilinear")
      {
        vpImageConvert::demosaicGRBGToRGBaBilinear(I_Bayer_16U.bitmap, I_RGBA_16U.bitmap, I_Bayer_16U.getWidth(),
                                                   I_Bayer_16U.getHeight());

        convertTo(I_RGBA_16U, I_RGBA_8U);
        double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
        std::cout << "16-bit - GRBG - Bilinear - PSNR: " << PSNR << std::endl;
        CHECK(PSNR >= min_PSNR_bilinear);
      }

      SECTION("Malvar")
      {
        vpImageConvert::demosaicGRBGToRGBaMalvar(I_Bayer_16U.bitmap, I_RGBA_16U.bitmap, I_Bayer_16U.getWidth(),
                                                 I_Bayer_16U.getHeight());

        convertTo(I_RGBA_16U, I_RGBA_8U);
        double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
        std::cout << "16-bit - GRBG - Malvar - PSNR: " << PSNR << std::endl;
        CHECK(PSNR >= min_PSNR_Malvar);
      }
    }

    SECTION("RGGB")
    {
      const std::string filename =
          vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Bayer/Klimt_Bayer_560x558_RGGB_12bits.raw");
      readBinaryFile(filename, buffer);

      col2im(buffer, I_Bayer_16U);

      SECTION("Bilinear")
      {
        vpImageConvert::demosaicRGGBToRGBaBilinear(I_Bayer_16U.bitmap, I_RGBA_16U.bitmap, I_Bayer_16U.getWidth(),
                                                   I_Bayer_16U.getHeight());

        convertTo(I_RGBA_16U, I_RGBA_8U);
        double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
        std::cout << "16-bit - RGGB - Bilinear - PSNR: " << PSNR << std::endl;
        CHECK(PSNR >= min_PSNR_bilinear);
      }

      SECTION("Malvar")
      {
        vpImageConvert::demosaicRGGBToRGBaMalvar(I_Bayer_16U.bitmap, I_RGBA_16U.bitmap, I_Bayer_16U.getWidth(),
                                                 I_Bayer_16U.getHeight());

        convertTo(I_RGBA_16U, I_RGBA_8U);
        double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
        std::cout << "16-bit - RGGB - Malvar - PSNR: " << PSNR << std::endl;
        CHECK(PSNR >= min_PSNR_Malvar);
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

      std::FILE *f = std::fopen(filename.c_str(), "rb");
      size_t sread = std::fread(&buffer[0], sizeof buffer[0], buffer.size(), f);
      REQUIRE(sread == buffer.size());

      col2im(buffer, I_Bayer_8U);

      SECTION("Bilinear")
      {
        vpImageConvert::demosaicBGGRToRGBaBilinear(I_Bayer_8U.bitmap, reinterpret_cast<uint8_t *>(I_RGBA_8U.bitmap),
                                                   I_Bayer_8U.getWidth(), I_Bayer_8U.getHeight());

        double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
        std::cout << "8-bit - BGGR - Bilinear - PSNR: " << PSNR << std::endl;
        CHECK(PSNR >= min_PSNR_bilinear);
      }

      SECTION("Malvar")
      {
        vpImageConvert::demosaicBGGRToRGBaMalvar(I_Bayer_8U.bitmap, reinterpret_cast<uint8_t *>(I_RGBA_8U.bitmap),
                                                 I_Bayer_8U.getWidth(), I_Bayer_8U.getHeight());

        double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
        std::cout << "8-bit - BGGR - Malvar - PSNR: " << PSNR << std::endl;
        CHECK(PSNR >= min_PSNR_Malvar);
      }
      std::fclose(f);
    }

    SECTION("GBRG")
    {
      const std::string filename =
          vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Bayer/Klimt_Bayer_560x558_GBRG_08bits.raw");

      std::FILE *f = std::fopen(filename.c_str(), "rb");
      size_t sread = std::fread(&buffer[0], sizeof buffer[0], buffer.size(), f);
      REQUIRE(sread == buffer.size());

      col2im(buffer, I_Bayer_8U);

      SECTION("Bilinear")
      {
        vpImageConvert::demosaicGBRGToRGBaBilinear(I_Bayer_8U.bitmap, reinterpret_cast<uint8_t *>(I_RGBA_8U.bitmap),
                                                   I_Bayer_8U.getWidth(), I_Bayer_8U.getHeight());

        double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
        std::cout << "8-bit - GBRG - Bilinear - PSNR: " << PSNR << std::endl;
        CHECK(PSNR >= min_PSNR_bilinear);
      }

      SECTION("Malvar")
      {
        vpImageConvert::demosaicGBRGToRGBaMalvar(I_Bayer_8U.bitmap, reinterpret_cast<uint8_t *>(I_RGBA_8U.bitmap),
                                                 I_Bayer_8U.getWidth(), I_Bayer_8U.getHeight());

        double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
        std::cout << "8-bit - GBRG - Malvar - PSNR: " << PSNR << std::endl;
        CHECK(PSNR >= min_PSNR_Malvar);
      }
      std::fclose(f);
    }

    SECTION("GRBG")
    {
      const std::string filename =
          vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Bayer/Klimt_Bayer_560x558_GRBG_08bits.raw");

      std::FILE *f = std::fopen(filename.c_str(), "rb");
      size_t sread = std::fread(&buffer[0], sizeof buffer[0], buffer.size(), f);
      REQUIRE(sread == buffer.size());

      col2im(buffer, I_Bayer_8U);

      SECTION("Bilinear")
      {
        vpImageConvert::demosaicGRBGToRGBaBilinear(I_Bayer_8U.bitmap, reinterpret_cast<uint8_t *>(I_RGBA_8U.bitmap),
                                                   I_Bayer_8U.getWidth(), I_Bayer_8U.getHeight());

        double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
        std::cout << "8-bit - GRBG - Bilinear - PSNR: " << PSNR << std::endl;
        CHECK(PSNR >= min_PSNR_bilinear);
      }

      SECTION("Malvar")
      {
        vpImageConvert::demosaicGRBGToRGBaMalvar(I_Bayer_8U.bitmap, reinterpret_cast<uint8_t *>(I_RGBA_8U.bitmap),
                                                 I_Bayer_8U.getWidth(), I_Bayer_8U.getHeight());

        double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
        std::cout << "8-bit - GRBG - Malvar - PSNR: " << PSNR << std::endl;
        CHECK(PSNR >= min_PSNR_Malvar);
      }
      std::fclose(f);
    }

    SECTION("RGGB")
    {
      const std::string filename =
          vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Bayer/Klimt_Bayer_560x558_RGGB_08bits.raw");

      std::FILE *f = std::fopen(filename.c_str(), "rb");
      size_t sread = std::fread(&buffer[0], sizeof buffer[0], buffer.size(), f);
      REQUIRE(sread == buffer.size());

      col2im(buffer, I_Bayer_8U);

      SECTION("Bilinear")
      {
        vpImageConvert::demosaicRGGBToRGBaBilinear(I_Bayer_8U.bitmap, reinterpret_cast<uint8_t *>(I_RGBA_8U.bitmap),
                                                   I_Bayer_8U.getWidth(), I_Bayer_8U.getHeight());

        double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
        std::cout << "8-bit - RGGB - Bilinear - PSNR: " << PSNR << std::endl;
        CHECK(PSNR >= min_PSNR_bilinear);
      }

      SECTION("Malvar")
      {
        vpImageConvert::demosaicRGGBToRGBaMalvar(I_Bayer_8U.bitmap, reinterpret_cast<uint8_t *>(I_RGBA_8U.bitmap),
                                                 I_Bayer_8U.getWidth(), I_Bayer_8U.getHeight());

        double PSNR = computePSNR(I_RGBA_8U, I_RGBA_8U_ref);
        std::cout << "8-bit - RGGB - Malvar - PSNR: " << PSNR << std::endl;
        CHECK(PSNR >= min_PSNR_Malvar);
      }
      std::fclose(f);
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
int main() { return EXIT_SUCCESS; }
#endif
