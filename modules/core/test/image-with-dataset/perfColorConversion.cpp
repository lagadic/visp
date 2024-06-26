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
 * Benchmark color image conversion.
 */

/*!
  \example perfColorConversion.cpp
 */
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2) && defined(VISP_HAVE_THREADS)
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include "common.hpp"
#include <thread>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS) && defined(HAVE_OPENCV_IMGPROC)
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif
static std::string ipath = vpIoTools::getViSPImagesDataPath();
static std::string imagePathColor = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");
static std::string imagePathGray = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");
static int nThreads = 0;

TEST_CASE("Benchmark rgba to grayscale (naive code)", "[benchmark]")
{
  vpImage<vpRGBa> I;
  vpImageIo::read(I, imagePathColor);

  vpImage<unsigned char> I_gray(I.getHeight(), I.getWidth());

  BENCHMARK("Benchmark rgba to grayscale (naive code)")
  {
    common_tools::RGBaToGrayRef(reinterpret_cast<unsigned char *>(I.bitmap), I_gray.bitmap, I.getSize());
    return I_gray;
  };
}

TEST_CASE("Benchmark rgba to grayscale (ViSP)", "[benchmark]")
{
  vpImage<vpRGBa> I;
  vpImageIo::read(I, imagePathColor);

  vpImage<unsigned char> I_gray(I.getHeight(), I.getWidth());

  BENCHMARK("Benchmark rgba to grayscale (ViSP)")
  {
    vpImageConvert::convert(I, I_gray, nThreads);
    return I_gray;
  };
}

TEST_CASE("Benchmark grayscale to rgba (naive code)", "[benchmark]")
{
  vpImage<unsigned char> I;
  vpImageIo::read(I, imagePathGray);

  vpImage<vpRGBa> I_color(I.getHeight(), I.getWidth());

  BENCHMARK("Benchmark grayscale to rgba (naive code)")
  {
    common_tools::grayToRGBaRef(I.bitmap, reinterpret_cast<unsigned char *>(I_color.bitmap), I.getSize());
    return I_color;
  };
}

TEST_CASE("Benchmark grayscale to rgba (ViSP)", "[benchmark]")
{
  vpImage<unsigned char> I;
  vpImageIo::read(I, imagePathGray);

  vpImage<vpRGBa> I_color(I.getHeight(), I.getWidth());

  BENCHMARK("Benchmark grayscale to rgba (ViSP)")
  {
    vpImageConvert::convert(I, I_color);
    return I_color;
  };
}

TEST_CASE("Benchmark split RGBa (ViSP)", "[benchmark]")
{
  vpImage<vpRGBa> I;
  vpImageIo::read(I, imagePathColor);

  vpImage<unsigned char> R, G, B, A;
  BENCHMARK("Benchmark split RGBa (ViSP)")
  {
    vpImageConvert::split(I, &R, &G, &B, &A);
    return R;
  };
}

TEST_CASE("Benchmark merge to RGBa (ViSP)", "[benchmark]")
{
  vpImage<vpRGBa> I;
  vpImageIo::read(I, imagePathColor);

  vpImage<unsigned char> R, G, B, A;
  vpImageConvert::split(I, &R, &G, &B, &A);

  vpImage<vpRGBa> I_merge(I.getHeight(), I.getWidth());
  BENCHMARK("Benchmark merge to RGBa (ViSP)")
  {
    vpImageConvert::merge(&R, &G, &B, &A, I_merge);
    return I_merge;
  };
}

TEST_CASE("Benchmark bgr to grayscale (naive code)", "[benchmark]")
{
  vpImage<vpRGBa> I;
  vpImageIo::read(I, imagePathColor);

  std::vector<unsigned char> bgr;
  common_tools::RGBaToBGR(I, bgr);

  vpImage<unsigned char> I_gray(I.getHeight(), I.getWidth());

  BENCHMARK("Benchmark bgr to grayscale (naive code)")
  {
    common_tools::BGRToGrayRef(bgr.data(), reinterpret_cast<unsigned char *>(I_gray.bitmap), I_gray.getWidth(),
                               I_gray.getHeight(), false);
    return I_gray;
  };
}

TEST_CASE("Benchmark bgr to grayscale (ViSP)", "[benchmark]")
{
  vpImage<vpRGBa> I;
  vpImageIo::read(I, imagePathColor);

  std::vector<unsigned char> bgr;
  common_tools::RGBaToBGR(I, bgr);

  vpImage<unsigned char> I_gray(I.getHeight(), I.getWidth());

  BENCHMARK("Benchmark bgr to grayscale (ViSP)")
  {
    vpImageConvert::BGRToGrey(bgr.data(), I_gray.bitmap, I.getWidth(), I.getHeight(), false, nThreads);
    return I_gray;
  };

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC)

  SECTION("OpenCV Mat type")
  {
    cv::Mat img;
    vpImageConvert::convert(I, img);

    BENCHMARK("Benchmark bgr to grayscale (ViSP + OpenCV Mat type)")
    {
      vpImageConvert::convert(img, I_gray, false, nThreads);
      return I_gray;
    };
  }
#endif
}

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS) && defined(HAVE_OPENCV_IMGPROC)
TEST_CASE("Benchmark bgr to grayscale (OpenCV)", "[benchmark]")
{
  cv::Mat img = cv::imread(imagePathColor);
  cv::Mat img_gray(img.size(), CV_8UC1);

  BENCHMARK("Benchmark bgr to grayscale (OpenCV)")
  {
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    return img_gray;
  };
}
#endif

TEST_CASE("Benchmark bgr to rgba (naive code)", "[benchmark]")
{
  vpImage<vpRGBa> I;
  vpImageIo::read(I, imagePathColor);

  std::vector<unsigned char> bgr;
  common_tools::RGBaToBGR(I, bgr);

  vpImage<vpRGBa> I_bench(I.getHeight(), I.getWidth());
  BENCHMARK("Benchmark bgr to rgba (naive code)")
  {
    common_tools::BGRToRGBaRef(bgr.data(), reinterpret_cast<unsigned char *>(I_bench.bitmap), I.getWidth(),
                               I.getHeight(), false);
    return I_bench;
  };
}

TEST_CASE("Benchmark bgr to rgba (ViSP)", "[benchmark]")
{
  vpImage<vpRGBa> I;
  vpImageIo::read(I, imagePathColor);

  std::vector<unsigned char> bgr;
  common_tools::RGBaToBGR(I, bgr);

  SECTION("Check BGR to RGBa conversion")
  {
    vpImage<vpRGBa> ref(I.getHeight(), I.getWidth());
    common_tools::BGRToRGBaRef(bgr.data(), reinterpret_cast<unsigned char *>(ref.bitmap), I.getWidth(), I.getHeight(),
                               false);
    vpImage<vpRGBa> rgba(I.getHeight(), I.getWidth());
    vpImageConvert::BGRToRGBa(bgr.data(), reinterpret_cast<unsigned char *>(rgba.bitmap), I.getWidth(), I.getHeight(),
                              false);

    CHECK((rgba == ref));
  }

  vpImage<vpRGBa> I_rgba(I.getHeight(), I.getWidth());
  BENCHMARK("Benchmark bgr to rgba (ViSP)")
  {
    vpImageConvert::BGRToRGBa(bgr.data(), reinterpret_cast<unsigned char *>(I_rgba.bitmap), I.getWidth(), I.getHeight(),
                              false);
    return I_rgba;
  };

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC)
  SECTION("OpenCV Mat type")
  {
    cv::Mat img;
    vpImageConvert::convert(I, img);

    BENCHMARK("Benchmark bgr to rgba (ViSP + OpenCV Mat type)")
    {
      vpImageConvert::convert(img, I_rgba);
      return I_rgba;
    };
  }
#endif
}

TEST_CASE("Benchmark bgra to rgba (naive code)", "[benchmark]")
{
  vpImage<vpRGBa> I;
  vpImageIo::read(I, imagePathColor);

  std::vector<unsigned char> bgra;
  common_tools::RGBaToBGRa(I, bgra);

  vpImage<vpRGBa> I_bench(I.getHeight(), I.getWidth());
  BENCHMARK("Benchmark bgra to rgba (naive code)")
  {
    common_tools::BGRaToRGBaRef(bgra.data(), reinterpret_cast<unsigned char *>(I_bench.bitmap), I.getWidth(),
                                I.getHeight(), false);
    return I_bench;
  };
}

TEST_CASE("Benchmark bgra to rgba (ViSP)", "[benchmark]")
{
  vpImage<vpRGBa> I;
  vpImageIo::read(I, imagePathColor);

  std::vector<unsigned char> bgra;
  common_tools::RGBaToBGRa(I, bgra);

  SECTION("Check BGRa to RGBa conversion")
  {
    vpImage<vpRGBa> ref(I.getHeight(), I.getWidth());
    common_tools::BGRaToRGBaRef(bgra.data(), reinterpret_cast<unsigned char *>(ref.bitmap), I.getWidth(), I.getHeight(),
                                false);
    vpImage<vpRGBa> rgba(I.getHeight(), I.getWidth());
    vpImageConvert::BGRaToRGBa(bgra.data(), reinterpret_cast<unsigned char *>(rgba.bitmap), I.getWidth(), I.getHeight(),
                               false);

    CHECK((rgba == ref));
  }
  vpImage<vpRGBa> I_rgba(I.getHeight(), I.getWidth());
  BENCHMARK("Benchmark bgra to rgba (ViSP)")
  {
    vpImageConvert::BGRaToRGBa(bgra.data(), reinterpret_cast<unsigned char *>(I_rgba.bitmap), I.getWidth(),
                               I.getHeight(), false);
    return I_rgba;
  };
}

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
    Opt(imagePathGray, "imagePathColor")["--imagePathGray"]("Path to gray image") |
    Opt(nThreads, "nThreads")["--nThreads"]("Number of threads");

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
    std::cout << "nThreads: " << nThreads << " / available threads: " << std::thread::hardware_concurrency()
      << std::endl;

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
