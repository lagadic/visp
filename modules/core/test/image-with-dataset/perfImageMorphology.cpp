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
 * Benchmark image morphology.
 */

/*!
  \example perfImageMorphology.cpp
 */

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_CATCH2
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include "common.hpp"
#include <visp3/core/vpImageMorphology.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif
static std::string ipath = vpIoTools::getViSPImagesDataPath();

TEST_CASE("Benchmark binary image morphology", "[benchmark]")
{
  std::string imagePath = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");
  vpImage<unsigned char> I;
  vpImageIo::read(I, imagePath);

  vpImage<unsigned char> I_Klimt_binarized = I;
  vpImageTools::binarise(I_Klimt_binarized, (unsigned char)127, (unsigned char)127, (unsigned char)0, (unsigned char)1,
    (unsigned char)1, true);

  SECTION("Dilatation")
  {
    SECTION("4-connexity")
    {
      const vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_4;
      BENCHMARK("Benchmark dilatation (naive code)")
      {
        common_tools::imageDilatationRef(I_Klimt_binarized, connexity);
        return I_Klimt_binarized;
      };

      BENCHMARK("Benchmark dilatation (ViSP)")
      {
        vpImageMorphology::dilatation(I_Klimt_binarized, (unsigned char)1, (unsigned char)0, connexity);
        return I_Klimt_binarized;
      };
    }

    SECTION("8-connexity")
    {
      const vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_8;
      BENCHMARK("Benchmark dilatation (naive code)")
      {
        common_tools::imageDilatationRef(I_Klimt_binarized, connexity);
        return I_Klimt_binarized;
      };

      BENCHMARK("Benchmark dilatation (ViSP)")
      {
        vpImageMorphology::dilatation(I_Klimt_binarized, (unsigned char)1, (unsigned char)0, connexity);
        return I_Klimt_binarized;
      };
    }
  }

  SECTION("Erosion")
  {
    SECTION("4-connexity")
    {
      const vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_4;
      BENCHMARK("Benchmark erosion (naive code)")
      {
        common_tools::imageErosionRef(I_Klimt_binarized, connexity);
        return I_Klimt_binarized;
      };

      BENCHMARK("Benchmark erosion (ViSP)")
      {
        vpImageMorphology::erosion(I_Klimt_binarized, (unsigned char)1, (unsigned char)0, connexity);
        return I_Klimt_binarized;
      };
    }

    SECTION("8-connexity")
    {
      const vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_8;
      BENCHMARK("Benchmark erosion (naive code)")
      {
        common_tools::imageErosionRef(I_Klimt_binarized, connexity);
        return I_Klimt_binarized;
      };

      BENCHMARK("Benchmark erosion (ViSP)")
      {
        vpImageMorphology::erosion(I_Klimt_binarized, (unsigned char)1, (unsigned char)0, connexity);
        return I_Klimt_binarized;
      };
    }
  }
}

#if (VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGPROC)
TEST_CASE("Benchmark gray image morphology", "[benchmark]")
{
  std::string imagePath = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");
  vpImage<unsigned char> I;
  vpImageIo::read(I, imagePath);

  cv::Mat img, imgMorph;
  vpImageConvert::convert(I, img);
  vpImageConvert::convert(I, imgMorph);
  cv::Mat cross_SE = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
  cv::Mat rect_SE = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

  SECTION("Dilatation")
  {
    SECTION("4-connexity")
    {
      const vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_4;
      BENCHMARK("Benchmark dilatation (naive code)")
      {
        common_tools::imageDilatationRef(I, connexity);
        return I;
      };

      BENCHMARK("Benchmark dilatation (ViSP)")
      {
        vpImageMorphology::dilatation<unsigned char>(I, connexity);
        return I;
      };

      BENCHMARK("Benchmark dilatation (OpenCV)")
      {
        cv::morphologyEx(imgMorph, imgMorph, cv::MORPH_DILATE, cross_SE);
        return I;
      };
    }

    SECTION("8-connexity")
    {
      const vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_8;
      BENCHMARK("Benchmark dilatation (naive code)")
      {
        common_tools::imageDilatationRef(I, connexity);
        return I;
      };

      BENCHMARK("Benchmark dilatation (ViSP)")
      {
        vpImageMorphology::dilatation<unsigned char>(I, connexity);
        return I;
      };

      BENCHMARK("Benchmark dilatation (OpenCV)")
      {
        cv::morphologyEx(imgMorph, imgMorph, cv::MORPH_DILATE, rect_SE);
        return I;
      };
    }
  }

  SECTION("Erosion")
  {
    SECTION("4-connexity")
    {
      const vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_4;
      BENCHMARK("Benchmark erosion (naive code)")
      {
        common_tools::imageErosionRef(I, connexity);
        return I;
      };

      BENCHMARK("Benchmark erosion (ViSP)")
      {
        vpImageMorphology::erosion<unsigned char>(I, connexity);
        return I;
      };

      BENCHMARK("Benchmark dilatation (OpenCV)")
      {
        cv::morphologyEx(imgMorph, imgMorph, cv::MORPH_ERODE, cross_SE);
        return I;
      };
    }

    SECTION("8-connexity")
    {
      const vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_8;
      BENCHMARK("Benchmark erosion (naive code)")
      {
        common_tools::imageErosionRef(I, connexity);
        return I;
      };

      BENCHMARK("Benchmark erosion (ViSP)")
      {
        vpImageMorphology::erosion<unsigned char>(I, connexity);
        return I;
      };

      BENCHMARK("Benchmark dilatation (OpenCV)")
      {
        cv::morphologyEx(imgMorph, imgMorph, cv::MORPH_ERODE, rect_SE);
        return I;
      };
    }
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
