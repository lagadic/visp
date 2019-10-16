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
 * Benchmark rgba to grayscale image conversion.
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_CATCH2
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>

namespace {
static std::string ipath = vpIoTools::getViSPImagesDataPath();

void computeRegularRGBaToGrayscale(const unsigned char * rgba, unsigned char *grey, unsigned int size)
{
  const unsigned char *pt_input = rgba;
  const unsigned char *pt_end = rgba + size * 4;
  unsigned char *pt_output = grey;

  while (pt_input != pt_end) {
    *pt_output = (unsigned char)(0.2126 * (*pt_input) + 0.7152 * (*(pt_input + 1)) + 0.0722 * (*(pt_input + 2)));
    pt_input += 4;
    pt_output++;
  }
}

#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
void computeRegularBGRToGrayscale(unsigned char *bgr, unsigned char *grey, unsigned int width,
                                  unsigned int height, bool flip=false)
{
  // if we have to flip the image, we start from the end last scanline so the
  // step is negative
  int lineStep = (flip) ? -(int)(width * 3) : (int)(width * 3);

  // starting source address = last line if we need to flip the image
  unsigned char *src = (flip) ? bgr + (width * height * 3) + lineStep : bgr;

  unsigned int j = 0;
  unsigned int i = 0;

  for (i = 0; i < height; i++) {
    unsigned char *line = src;
    for (j = 0; j < width; j++) {
      *grey++ = (unsigned char)(0.2126 * *(line + 2) + 0.7152 * *(line + 1) + 0.0722 * *(line + 0));
      line += 3;
    }

    // go to the next line
    src += lineStep;
  }
}
#endif
}

TEST_CASE("Benchmark rgba to grayscale (naive code)", "[benchmark]") {
  std::string imagePath = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");
  vpImage<vpRGBa> I;
  vpImageIo::read(I, imagePath);

  vpImage<unsigned char> I_gray(I.getHeight(), I.getWidth());

  BENCHMARK("Benchmark rgba to grayscale Klimt (naive code)") {
    computeRegularRGBaToGrayscale(reinterpret_cast<unsigned char *>(I.bitmap),
                                  I_gray.bitmap, I.getSize());
    return I_gray;
  };
}

TEST_CASE("Benchmark rgba to grayscale (ViSP)", "[benchmark]") {
  std::string imagePath = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");
  vpImage<vpRGBa> I;
  vpImageIo::read(I, imagePath);

  vpImage<unsigned char> I_gray(I.getHeight(), I.getWidth());

  BENCHMARK("Benchmark rgba to grayscale Klimt (ViSP)") {
    vpImageConvert::convert(I, I_gray);
    return I_gray;
  };
}

#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
TEST_CASE("Benchmark bgr to grayscale (naive code)", "[benchmark]") {
  std::string imagePath = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");
  cv::Mat img = cv::imread(imagePath);

  vpImage<unsigned char> I_gray(img.rows, img.cols);

  BENCHMARK("Benchmark bgr to grayscale (naive code)") {
    computeRegularBGRToGrayscale(reinterpret_cast<unsigned char*>(img.ptr<uchar>()),
                                 reinterpret_cast<unsigned char *>(I_gray.bitmap),
                                 I_gray.getWidth(), I_gray.getHeight());
    return I_gray;
  };
}

TEST_CASE("Benchmark bgr to grayscale (ViSP)", "[benchmark]") {
  std::string imagePath = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");
  cv::Mat img = cv::imread(imagePath);

  std::vector<unsigned char> grayscale(img.rows*img.cols);

  BENCHMARK("Benchmark bgr to grayscale (ViSP)") {
    vpImageConvert::BGRToGrey(reinterpret_cast<unsigned char *>(img.ptr<uchar>()),
                              grayscale.data(), img.cols, img.rows);
    return grayscale;
  };
}

TEST_CASE("Benchmark bgr to grayscale (OpenCV)", "[benchmark]") {
  std::string imagePath = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");
  cv::Mat img = cv::imread(imagePath);
  cv::Mat img_gray(img.size(), CV_8UC1);

  BENCHMARK("Benchmark bgr to grayscale (OpenCV)") {
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    return img_gray;
  };
}
#endif

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance

  bool runBenchmark = false;
  // Build a new parser on top of Catch's
  using namespace Catch::clara;
  auto cli = session.cli() // Get Catch's composite command line parser
    | Opt(runBenchmark)    // bind variable to a new option, with a hint string
    ["--benchmark"]        // the option names it will respond to
    ("run benchmark?");    // description string for the help output

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

int main()
{
  return 0;
}
#endif
