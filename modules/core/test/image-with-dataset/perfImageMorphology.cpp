/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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

#if defined(VISP_HAVE_CATCH2)

#include <catch_amalgamated.hpp>

#include "common.hpp"
#include <visp3/core/vpImageMorphology.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif
VP_ATTRIBUTE_NO_DESTROY static std::string ipath = vpIoTools::getViSPImagesDataPath();

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

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGPROC)
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
  Catch::Session session;

  bool runBenchmark = false;
  auto cli = session.cli()
    | Catch::Clara::Opt(runBenchmark)["--benchmark"]("run benchmark?");

  session.cli(cli);
  session.applyCommandLine(argc, argv);

  if (runBenchmark) {
    int numFailed = session.run();

    return numFailed;
  }

  return EXIT_SUCCESS;
}
#else
#include <iostream>

int main() { return EXIT_SUCCESS; }
#endif
