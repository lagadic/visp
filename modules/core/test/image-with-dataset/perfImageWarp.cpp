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
 * Benchmark image warping.
 */

/*!
  \example perfImageWarp.cpp
 */

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_CATCH2
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif
namespace
{
static std::string ipath = vpIoTools::getViSPImagesDataPath();
}

TEST_CASE("Benchmark affine warp on grayscale image", "[benchmark]")
{
  std::string imgPath = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");
  REQUIRE(vpIoTools::checkFilename(imgPath));

  vpImage<unsigned char> I;
  vpImageIo::read(I, imgPath);
  REQUIRE(I.getSize() > 0);
  vpImage<unsigned char> I_affine(I.getHeight(), I.getWidth());

  vpMatrix M(2, 3);
  M.eye();

  const double theta = vpMath::rad(45);
  M[0][0] = cos(theta);
  M[0][1] = -sin(theta);
  M[0][2] = I.getWidth() / 2;
  M[1][0] = sin(theta);
  M[1][1] = cos(theta);
  M[1][2] = I.getHeight() / 2;

  BENCHMARK("Benchmark affine warp (ref code) (NN)")
  {
    vpImageTools::warpImage(I, M, I_affine, vpImageTools::INTERPOLATION_NEAREST, false);
    return I_affine;
  };

  BENCHMARK("Benchmark affine warp (fixed-point) (NN)")
  {
    vpImageTools::warpImage(I, M, I_affine, vpImageTools::INTERPOLATION_NEAREST);
    return I_affine;
  };

  BENCHMARK("Benchmark affine warp (ref code) (bilinear)")
  {
    vpImageTools::warpImage(I, M, I_affine, vpImageTools::INTERPOLATION_LINEAR, false);
    return I_affine;
  };

  BENCHMARK("Benchmark affine warp (fixed-point) (bilinear)")
  {
    vpImageTools::warpImage(I, M, I_affine, vpImageTools::INTERPOLATION_LINEAR);
    return I_affine;
  };

#if (VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGPROC)
  cv::Mat img, img_affine;
  vpImageConvert::convert(I, img);
  vpImageConvert::convert(I, img_affine);

  cv::Mat M_cv(2, 3, CV_64FC1);
  for (unsigned int i = 0; i < M.getRows(); i++) {
    for (unsigned int j = 0; j < M.getCols(); j++) {
      M_cv.at<double>(i, j) = M[i][j];
    }
  }

  BENCHMARK("Benchmark affine warp (OpenCV) (NN)")
  {
    cv::warpAffine(img, img_affine, M_cv, img.size(), cv::INTER_NEAREST);
    return img_affine;
  };

  BENCHMARK("Benchmark affine warp (OpenCV) (bilinear)")
  {
    cv::warpAffine(img, img_affine, M_cv, img.size(), cv::INTER_LINEAR);
    return img_affine;
  };
#endif
}

TEST_CASE("Benchmark affine warp on color image", "[benchmark]")
{
  std::string imgPath = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");
  REQUIRE(vpIoTools::checkFilename(imgPath));

  vpImage<vpRGBa> I;
  vpImageIo::read(I, imgPath);
  REQUIRE(I.getSize() > 0);
  vpImage<vpRGBa> I_affine(I.getHeight(), I.getWidth());

  vpMatrix M(2, 3);
  M.eye();

  const double theta = vpMath::rad(45);
  M[0][0] = cos(theta);
  M[0][1] = -sin(theta);
  M[0][2] = I.getWidth() / 2;
  M[1][0] = sin(theta);
  M[1][1] = cos(theta);
  M[1][2] = I.getHeight() / 2;

  BENCHMARK("Benchmark affine warp (ref code) (NN)")
  {
    vpImageTools::warpImage(I, M, I_affine, vpImageTools::INTERPOLATION_NEAREST, false);
    return I_affine;
  };

  BENCHMARK("Benchmark affine warp (fixed-point) (NN)")
  {
    vpImageTools::warpImage(I, M, I_affine, vpImageTools::INTERPOLATION_NEAREST);
    return I_affine;
  };

  BENCHMARK("Benchmark affine warp (ref code) (bilinear)")
  {
    vpImageTools::warpImage(I, M, I_affine, vpImageTools::INTERPOLATION_LINEAR, false);
    return I_affine;
  };

  BENCHMARK("Benchmark affine warp (fixed-point) (bilinear)")
  {
    vpImageTools::warpImage(I, M, I_affine, vpImageTools::INTERPOLATION_LINEAR);
    return I_affine;
  };

#if (VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGPROC)
  cv::Mat img, img_affine;
  vpImageConvert::convert(I, img);
  vpImageConvert::convert(I, img_affine);

  cv::Mat M_cv(2, 3, CV_64FC1);
  for (unsigned int i = 0; i < M.getRows(); i++) {
    for (unsigned int j = 0; j < M.getCols(); j++) {
      M_cv.at<double>(i, j) = M[i][j];
    }
  }

  BENCHMARK("Benchmark affine warp (OpenCV) (NN)")
  {
    cv::warpAffine(img, img_affine, M_cv, img.size(), cv::INTER_NEAREST);
    return img_affine;
  };

  BENCHMARK("Benchmark affine warp (OpenCV) (bilinear)")
  {
    cv::warpAffine(img, img_affine, M_cv, img.size(), cv::INTER_LINEAR);
    return img_affine;
  };
#endif
}

TEST_CASE("Benchmark perspective warp on grayscale image", "[benchmark]")
{
  std::string imgPath = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");
  REQUIRE(vpIoTools::checkFilename(imgPath));

  vpImage<unsigned char> I;
  vpImageIo::read(I, imgPath);
  REQUIRE(I.getSize() > 0);
  vpImage<unsigned char> I_perspective(I.getHeight(), I.getWidth());

  vpMatrix M(3, 3);
  M.eye();

  const double theta = vpMath::rad(45);
  M[0][0] = cos(theta);
  M[0][1] = -sin(theta);
  M[0][2] = I.getWidth() / 2;
  M[1][0] = sin(theta);
  M[1][1] = cos(theta);
  M[1][2] = I.getHeight() / 2;

  BENCHMARK("Benchmark perspective warp (ref code) (NN)")
  {
    vpImageTools::warpImage(I, M, I_perspective, vpImageTools::INTERPOLATION_NEAREST, false);
    return I_perspective;
  };

  BENCHMARK("Benchmark perspective warp (fixed-point) (NN)")
  {
    vpImageTools::warpImage(I, M, I_perspective, vpImageTools::INTERPOLATION_NEAREST);
    return I_perspective;
  };

  BENCHMARK("Benchmark perspective warp (ref code) (bilinear)")
  {
    vpImageTools::warpImage(I, M, I_perspective, vpImageTools::INTERPOLATION_LINEAR, false);
    return I_perspective;
  };

  BENCHMARK("Benchmark perspective warp (fixed-point) (bilinear)")
  {
    vpImageTools::warpImage(I, M, I_perspective, vpImageTools::INTERPOLATION_LINEAR);
    return I_perspective;
  };

#if (VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGPROC)
  cv::Mat img, img_perspective;
  vpImageConvert::convert(I, img);
  vpImageConvert::convert(I, img_perspective);

  cv::Mat M_cv(3, 3, CV_64FC1);
  for (unsigned int i = 0; i < M.getRows(); i++) {
    for (unsigned int j = 0; j < M.getCols(); j++) {
      M_cv.at<double>(i, j) = M[i][j];
    }
  }

  BENCHMARK("Benchmark perspective warp (OpenCV) (NN)")
  {
    cv::warpPerspective(img, img_perspective, M_cv, img.size(), cv::INTER_NEAREST);
    return img_perspective;
  };

  BENCHMARK("Benchmark perspective warp (OpenCV) (bilinear)")
  {
    cv::warpPerspective(img, img_perspective, M_cv, img.size(), cv::INTER_LINEAR);
    return img_perspective;
  };
#endif
}

TEST_CASE("Benchmark perspective warp on color image", "[benchmark]")
{
  std::string imgPath = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");
  REQUIRE(vpIoTools::checkFilename(imgPath));

  vpImage<vpRGBa> I;
  vpImageIo::read(I, imgPath);
  REQUIRE(I.getSize() > 0);
  vpImage<vpRGBa> I_perspective(I.getHeight(), I.getWidth());

  vpMatrix M(3, 3);
  M.eye();

  const double theta = vpMath::rad(45);
  M[0][0] = cos(theta);
  M[0][1] = -sin(theta);
  M[0][2] = I.getWidth() / 2;
  M[1][0] = sin(theta);
  M[1][1] = cos(theta);
  M[1][2] = I.getHeight() / 2;

  BENCHMARK("Benchmark perspective warp (ref code) (NN)")
  {
    vpImageTools::warpImage(I, M, I_perspective, vpImageTools::INTERPOLATION_NEAREST, false);
    return I_perspective;
  };

  BENCHMARK("Benchmark perspective warp (fixed-point) (NN)")
  {
    vpImageTools::warpImage(I, M, I_perspective, vpImageTools::INTERPOLATION_NEAREST);
    return I_perspective;
  };

  BENCHMARK("Benchmark perspective warp (ref code) (bilinear)")
  {
    vpImageTools::warpImage(I, M, I_perspective, vpImageTools::INTERPOLATION_LINEAR, false);
    return I_perspective;
  };

  BENCHMARK("Benchmark perspective warp (fixed-point) (bilinear)")
  {
    vpImageTools::warpImage(I, M, I_perspective, vpImageTools::INTERPOLATION_LINEAR);
    return I_perspective;
  };

#if (VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGPROC)
  cv::Mat img, img_perspective;
  vpImageConvert::convert(I, img);
  vpImageConvert::convert(I, img_perspective);

  cv::Mat M_cv(3, 3, CV_64FC1);
  for (unsigned int i = 0; i < M.getRows(); i++) {
    for (unsigned int j = 0; j < M.getCols(); j++) {
      M_cv.at<double>(i, j) = M[i][j];
    }
  }

  BENCHMARK("Benchmark perspective warp (OpenCV) (NN)")
  {
    cv::warpPerspective(img, img_perspective, M_cv, img.size(), cv::INTER_NEAREST);
    return img_perspective;
  };

  BENCHMARK("Benchmark perspective warp (OpenCV) (bilinear)")
  {
    cv::warpPerspective(img, img_perspective, M_cv, img.size(), cv::INTER_LINEAR);
    return img_perspective;
  };
#endif
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
    ("run benchmark?");   // description string for the help output

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
