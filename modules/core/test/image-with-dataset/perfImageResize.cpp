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
 * Benchmark image resize.
 */

/*!
  \example perfImageResize.cpp
 */

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2) && defined(VISP_HAVE_THREADS)
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include "common.hpp"
#include <thread>
#include <visp3/core/vpImageTools.h>
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
static std::string imagePathColor = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");
static std::string imagePathGray = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");
static unsigned int g_resize_width = 293;
static unsigned int g_resize_height = 137;

TEST_CASE("Nearest Neighbor image resize (naive code)", "[benchmark]")
{
  SECTION("unsigned char")
  {
    vpImage<unsigned char> I, Iresize(g_resize_height, g_resize_width);
    vpImageIo::read(I, imagePathGray);

    BENCHMARK("Benchmark Nearest Neighbor uchar image resize (naive code)")
    {
      common_tools::resizeRef(I, Iresize, common_tools::g_nearest_neighbor);
      return Iresize;
    };
  }

  SECTION("vpRGBa")
  {
    vpImage<vpRGBa> I, Iresize(g_resize_height, g_resize_width);
    vpImageIo::read(I, imagePathColor);

    BENCHMARK("Benchmark Nearest Neighbor RGBa image resize (naive code)")
    {
      common_tools::resizeRef(I, Iresize, common_tools::g_nearest_neighbor);
      return Iresize;
    };
  }
}

TEST_CASE("Nearest Neighbor image resize (ViSP)", "[benchmark]")
{
  SECTION("unsigned char")
  {
    vpImage<unsigned char> I, Iresize(g_resize_height, g_resize_width);
    vpImageIo::read(I, imagePathGray);

    BENCHMARK("Benchmark Nearest Neighbor uchar image resize (ViSP) (1 thread)")
    {
      vpImageTools::resize(I, Iresize, vpImageTools::INTERPOLATION_NEAREST, 1);
      return Iresize;
    };

    const unsigned int nThreads = std::thread::hardware_concurrency();
    std::stringstream buffer;
    buffer << "Benchmark Nearest Neighbor uchar image resize (ViSP) (" << nThreads << "threads)";
    BENCHMARK(buffer.str().c_str())
    {
      vpImageTools::resize(I, Iresize, vpImageTools::INTERPOLATION_NEAREST, nThreads);
      return Iresize;
    };
  }

  SECTION("vpRGBa")
  {
    vpImage<vpRGBa> I, Iresize(g_resize_height, g_resize_width);
    vpImageIo::read(I, imagePathColor);

    BENCHMARK("Benchmark Nearest Neighbor RGBa image resize (ViSP) (1 thread)")
    {
      vpImageTools::resize(I, Iresize, vpImageTools::INTERPOLATION_NEAREST, 1);
      return Iresize;
    };

    const unsigned int nThreads = std::thread::hardware_concurrency();
    std::stringstream buffer;
    buffer << "Benchmark Nearest Neighbor RGBa image resize (ViSP) (" << nThreads << " threads)";
    BENCHMARK(buffer.str().c_str())
    {
      vpImageTools::resize(I, Iresize, vpImageTools::INTERPOLATION_NEAREST, nThreads);
      return Iresize;
    };
  }
}

TEST_CASE("Bilinear image resize (naive code)", "[benchmark]")
{
  SECTION("unsigned char")
  {
    vpImage<unsigned char> I, Iresize(g_resize_height, g_resize_width);
    vpImageIo::read(I, imagePathGray);

    BENCHMARK("Benchmark Bilinear uchar image resize (naive code)")
    {
      common_tools::resizeRef(I, Iresize, common_tools::g_bilinear);
      return Iresize;
    };
  }

  SECTION("vpRGBa")
  {
    vpImage<vpRGBa> I, Iresize(g_resize_height, g_resize_width);
    vpImageIo::read(I, imagePathColor);

    BENCHMARK("Benchmark Bilinear RGBa image resize (naive code)")
    {
      common_tools::resizeRef(I, Iresize, common_tools::g_bilinear);
      return Iresize;
    };
  }
}

TEST_CASE("Bilinear image resize (ViSP)", "[benchmark]")
{
  SECTION("unsigned char")
  {
    vpImage<unsigned char> I, Iresize(g_resize_height, g_resize_width);
    vpImageIo::read(I, imagePathGray);

    BENCHMARK("Benchmark Bilinear uchar image resize (ViSP)")
    {
      vpImageTools::resize(I, Iresize, vpImageTools::INTERPOLATION_LINEAR);
      return Iresize;
    };
  }

  SECTION("vpRGBa")
  {
    vpImage<vpRGBa> I, Iresize(g_resize_height, g_resize_width);
    vpImageIo::read(I, imagePathColor);

    BENCHMARK("Benchmark Bilinear RGBa image resize (ViSP)")
    {
      vpImageTools::resize(I, Iresize, vpImageTools::INTERPOLATION_LINEAR);
      return Iresize;
    };
  }
}

TEST_CASE("Area image resize (ViSP)", "[benchmark]")
{
  SECTION("unsigned char")
  {
    vpImage<unsigned char> I, Iresize(g_resize_height, g_resize_width);
    vpImageIo::read(I, imagePathGray);

    BENCHMARK("Benchmark Area uchar image resize (ViSP)")
    {
      vpImageTools::resize(I, Iresize, vpImageTools::INTERPOLATION_AREA);
      return Iresize;
    };
  }

  SECTION("vpRGBa")
  {
    vpImage<vpRGBa> I, Iresize(g_resize_height, g_resize_width);
    vpImageIo::read(I, imagePathColor);

    BENCHMARK("Benchmark Area RGBa image resize (ViSP)")
    {
      vpImageTools::resize(I, Iresize, vpImageTools::INTERPOLATION_AREA);
      return Iresize;
    };
  }
}

TEST_CASE("Bicubic image resize (ViSP)", "[benchmark]")
{
  SECTION("unsigned char")
  {
    vpImage<unsigned char> I, Iresize(g_resize_height, g_resize_width);
    vpImageIo::read(I, imagePathGray);

    BENCHMARK("Benchmark Bicubic uchar image resize (ViSP) (1 thread)")
    {
      vpImageTools::resize(I, Iresize, vpImageTools::INTERPOLATION_CUBIC, 1);
      return Iresize;
    };

    const unsigned int nThreads = std::thread::hardware_concurrency();
    std::stringstream buffer;
    buffer << "Benchmark Bicubic uchar image resize (ViSP) (" << nThreads << " threads)";
    BENCHMARK(buffer.str().c_str())
    {
      vpImageTools::resize(I, Iresize, vpImageTools::INTERPOLATION_CUBIC, nThreads);
      return Iresize;
    };
  }

  SECTION("vpRGBa")
  {
    vpImage<vpRGBa> I, Iresize(g_resize_height, g_resize_width);
    vpImageIo::read(I, imagePathColor);

    BENCHMARK("Benchmark Bicubic RGBa image resize (ViSP) (1 thread)")
    {
      vpImageTools::resize(I, Iresize, vpImageTools::INTERPOLATION_CUBIC, 1);
      return Iresize;
    };

    const unsigned int nThreads = std::thread::hardware_concurrency();
    std::stringstream buffer;
    buffer << "Benchmark Bicubic RGBa image resize (ViSP) (" << nThreads << " threads)";
    BENCHMARK(buffer.str().c_str())
    {
      vpImageTools::resize(I, Iresize, vpImageTools::INTERPOLATION_CUBIC, nThreads);
      return Iresize;
    };
  }
}

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS) && defined(HAVE_OPENCV_IMGPROC)
TEST_CASE("Nearest Neighbor image resize (OpenCV)", "[benchmark]")
{
  SECTION("unsigned char")
  {
    cv::Mat img, img_resize;
    img = cv::imread(imagePathGray, cv::IMREAD_GRAYSCALE);

    BENCHMARK("Benchmark Nearest Neighbor uchar image resize (OpenCV)")
    {
      cv::resize(img, img_resize, cv::Size(g_resize_width, g_resize_height), 0, 0, cv::INTER_NEAREST);
      return img_resize;
    };
  }

  SECTION("BGR")
  {
    cv::Mat img, img_resize;
    img = cv::imread(imagePathColor, cv::IMREAD_COLOR);

    BENCHMARK("Benchmark Nearest Neighbor BGR image resize (OpenCV)")
    {
      cv::resize(img, img_resize, cv::Size(g_resize_width, g_resize_height), 0, 0, cv::INTER_NEAREST);
      return img_resize;
    };
  }
}

TEST_CASE("Bilinear image resize (OpenCV)", "[benchmark]")
{
  SECTION("unsigned char")
  {
    cv::Mat img, img_resize;
    img = cv::imread(imagePathGray, cv::IMREAD_GRAYSCALE);

    BENCHMARK("Benchmark Bilinear uchar image resize (OpenCV)")
    {
      cv::resize(img, img_resize, cv::Size(g_resize_width, g_resize_height), 0, 0, cv::INTER_LINEAR);
      return img_resize;
    };
  }

  SECTION("BGR")
  {
    cv::Mat img, img_resize;
    img = cv::imread(imagePathColor, cv::IMREAD_COLOR);

    BENCHMARK("Benchmark Bilinear BGR image resize (OpenCV)")
    {
      cv::resize(img, img_resize, cv::Size(g_resize_width, g_resize_height), 0, 0, cv::INTER_LINEAR);
      return img_resize;
    };
  }
}

TEST_CASE("Area image resize (OpenCV)", "[benchmark]")
{
  SECTION("unsigned char")
  {
    cv::Mat img, img_resize;
    img = cv::imread(imagePathGray, cv::IMREAD_GRAYSCALE);

    BENCHMARK("Benchmark Area uchar image resize (OpenCV)")
    {
      cv::resize(img, img_resize, cv::Size(g_resize_width, g_resize_height), 0, 0, cv::INTER_AREA);
      return img_resize;
    };
  }

  SECTION("BGR")
  {
    cv::Mat img, img_resize;
    img = cv::imread(imagePathColor, cv::IMREAD_COLOR);

    BENCHMARK("Benchmark Area BGR image resize (OpenCV)")
    {
      cv::resize(img, img_resize, cv::Size(g_resize_width, g_resize_height), 0, 0, cv::INTER_AREA);
      return img_resize;
    };
  }
}

TEST_CASE("Bicubic image resize (OpenCV)", "[benchmark]")
{
  SECTION("unsigned char")
  {
    cv::Mat img, img_resize;
    img = cv::imread(imagePathGray, cv::IMREAD_GRAYSCALE);

    BENCHMARK("Benchmark Bicubic uchar image resize (OpenCV)")
    {
      cv::resize(img, img_resize, cv::Size(g_resize_width, g_resize_height), 0, 0, cv::INTER_CUBIC);
      return img_resize;
    };
  }

  SECTION("BGR")
  {
    cv::Mat img, img_resize;
    img = cv::imread(imagePathColor, cv::IMREAD_COLOR);

    BENCHMARK("Benchmark Bicubic BGR image resize (OpenCV)")
    {
      cv::resize(img, img_resize, cv::Size(g_resize_width, g_resize_height), 0, 0, cv::INTER_CUBIC);
      return img_resize;
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
    | Opt(imagePathColor, "imagePathColor")["--imagePathColor"]("Path to color image") |
    Opt(imagePathGray, "imagePathColor")["--imagePathGray"] |
    Opt(g_resize_width, "g_resize_width")["--width"]("Resize width") |
    Opt(g_resize_height, "g_resize_height")["--height"]("Resize height");

// Now pass the new composite back to Catch so it uses that
  session.cli(cli);

  // Let Catch (using Clara) parse the command line
  session.applyCommandLine(argc, argv);

  if (runBenchmark) {
    vpImage<vpRGBa> I_color;
    vpImageIo::read(I_color, imagePathColor);
    std::cout << "imagePathColor:\n\t" << imagePathColor << "\n\t" << I_color.getWidth() << "x" << I_color.getHeight()
      << std::endl;

    vpImage<unsigned char> I_gray;
    vpImageIo::read(I_gray, imagePathGray);
    std::cout << "imagePathGray:\n\t" << imagePathGray << "\n\t" << I_gray.getWidth() << "x" << I_gray.getHeight()
      << std::endl;
    std::cout << "Resize to: " << g_resize_width << "x" << g_resize_height << std::endl;

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
