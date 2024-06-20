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
 * Benchmark Gaussian filter.
 */

/*!
  \example perfGaussianFilter.cpp
 */

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_SIMDLIB) && defined(VISP_HAVE_CATCH2)
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include <visp3/core/vpGaussianFilter.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS) && defined(HAVE_OPENCV_IMGPROC)
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif
static const std::string ipath = vpIoTools::getViSPImagesDataPath();
static std::string imagePath = vpIoTools::createFilePath(ipath, "faces/1280px-Solvay_conference_1927.png");

TEST_CASE("vpGaussianFilter", "[benchmark]")
{
  SECTION("unsigned char")
  {
    vpImage<unsigned char> I, I_blur;
    vpImageIo::read(I, imagePath);

    const float sigma = 5.0f;
    vpGaussianFilter gaussianFilter(I.getWidth(), I.getHeight(), sigma);
    BENCHMARK("Benchmark vpGaussianFilter uchar")
    {
      gaussianFilter.apply(I, I_blur);
      return I_blur;
    };
  }

  SECTION("vpRGBa")
  {
    vpImage<vpRGBa> I, I_blur;
    vpImageIo::read(I, imagePath);

    const float sigma = 5.0f;
    vpGaussianFilter gaussianFilter(I.getWidth(), I.getHeight(), sigma);
    BENCHMARK("Benchmark vpGaussianFilter vpRGBa")
    {
      gaussianFilter.apply(I, I_blur);
      return I_blur;
    };
  }

  SECTION("vpRGBa + deinterleave")
  {
    vpImage<vpRGBa> I, I_blur;
    vpImageIo::read(I, imagePath);

    const float sigma = 5.0f;
    const bool deinterleave = true;
    vpGaussianFilter gaussianFilter(I.getWidth(), I.getHeight(), sigma, deinterleave);
    BENCHMARK("Benchmark vpGaussianFilter vpRGBa")
    {
      gaussianFilter.apply(I, I_blur);
      return I_blur;
    };
  }
}

TEST_CASE("vpImageFilter::gaussianBlur", "[benchmark]")
{
  SECTION("unsigned char")
  {
    vpImage<unsigned char> I;
    vpImageIo::read(I, imagePath);

    vpImage<double> I_blur;
    const unsigned int kernelSize = 7;
    const double sigma = 5.0;
    BENCHMARK("Benchmark vpImageFilter::gaussianBlur uchar")
    {
      vpImageFilter::gaussianBlur(I, I_blur, kernelSize, sigma);
      return I_blur;
    };
  }

  SECTION("vpRGBa")
  {
    vpImage<vpRGBa> I, I_blur;
    vpImageIo::read(I, imagePath);

    const unsigned int kernelSize = 7;
    const double sigma = 5.0;
    BENCHMARK("Benchmark vpImageFilter::gaussianBlur vpRGBa")
    {
      vpImageFilter::gaussianBlur(I, I_blur, kernelSize, sigma);
      return I_blur;
    };
  }
}

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS) && defined(HAVE_OPENCV_IMGPROC)

TEST_CASE("Gaussian filter (OpenCV)", "[benchmark]")
{
  SECTION("unsigned char")
  {
    cv::Mat img, img_blur;
    img = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);

    const double sigma = 5.0;
    BENCHMARK("Benchmark Gaussian filter uchar (OpenCV)")
    {
      cv::GaussianBlur(img, img_blur, cv::Size(), sigma);
      return img_blur;
    };
  }

  SECTION("BGR")
  {
    cv::Mat img, img_blur;
    img = cv::imread(imagePath, cv::IMREAD_COLOR);

    const double sigma = 5.0;
    BENCHMARK("Benchmark Gaussian filter BGR (OpenCV)")
    {
      cv::GaussianBlur(img, img_blur, cv::Size(), sigma);
      return img_blur;
    };
  }
}
#endif

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance

  bool runBenchmark = false;
  // Build a new parser on top of Catch's
  using namespace Catch::clara;
  auto cli = session.cli()         // Get Catch's composite command line parser
    | Opt(runBenchmark)   // bind variable to a new option, with a hint string
    ["--benchmark"] // the option names it will respond to
    ("run benchmark?")    // description string for the help output
    ;

// Now pass the new composite back to Catch so it uses that
  session.cli(cli);

  // Let Catch (using Clara) parse the command line
  session.applyCommandLine(argc, argv);

  if (runBenchmark) {
    int numFailed = session.run();

    // numFailed is clamped to 255 as some unices only use the lower 8 bits.
    // This clamping has already been applied, so just return it here
    // You can also do any post run clean-up here
    return numFailed;
  }

  return EXIT_SUCCESS;
}
#else
#include <iostream>

int main() { return EXIT_SUCCESS; }
#endif
