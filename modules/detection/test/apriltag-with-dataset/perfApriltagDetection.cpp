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
 * Apriltag detection performance test.
 */

/*!
  \example perfApriltagDetection.cpp
 */
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2)
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include <visp3/core/vpIoTools.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/io/vpImageIo.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

TEST_CASE("Benchmark Apriltag detection 1920x1080", "[benchmark]")
{
  const double tagSize = 0.25;
  const vpCameraParameters cam(2100, 2100, 960, 540);
  const size_t nbTags = 5;

  SECTION("tag16_05")
  {
    std::string filename = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                                     "AprilTag/benchmark/1920x1080/tag16_05_1920x1080.png");
    REQUIRE(vpIoTools::checkFilename(filename));
    vpImage<unsigned char> I;
    vpImageIo::read(I, filename);

    vpDetectorAprilTag apriltag_detector(vpDetectorAprilTag::TAG_16h5);
    BENCHMARK("Benchmark Apriltag detection: tag16_05 1920x1080")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };

    apriltag_detector.setAprilTagQuadDecimate(2);
    BENCHMARK("Benchmark Apriltag detection: tag16_05 1920x1080 decimate=2")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };

    apriltag_detector.setAprilTagQuadDecimate(3);
    BENCHMARK("Benchmark Apriltag detection: tag16_05 1920x1080 decimate=3")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };
  }

  SECTION("tag25_09")
  {
    std::string filename = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                                     "AprilTag/benchmark/1920x1080/tag25_09_1920x1080.png");
    REQUIRE(vpIoTools::checkFilename(filename));
    vpImage<unsigned char> I;
    vpImageIo::read(I, filename);

    vpDetectorAprilTag apriltag_detector(vpDetectorAprilTag::TAG_25h9);
    BENCHMARK("Benchmark Apriltag detection: tag25_09 1920x1080")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };

    apriltag_detector.setAprilTagQuadDecimate(2);
    BENCHMARK("Benchmark Apriltag detection: tag25_09 1920x1080 decimate=2")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };

    apriltag_detector.setAprilTagQuadDecimate(3);
    BENCHMARK("Benchmark Apriltag detection: tag25_09 1920x1080 decimate=3")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };
  }

  SECTION("tag36_11")
  {
    std::string filename = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                                     "AprilTag/benchmark/1920x1080/tag36_11_1920x1080.png");
    REQUIRE(vpIoTools::checkFilename(filename));
    vpImage<unsigned char> I;
    vpImageIo::read(I, filename);

    vpDetectorAprilTag apriltag_detector(vpDetectorAprilTag::TAG_36h11);
    BENCHMARK("Benchmark Apriltag detection: tag36_11 1920x1080")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };

    apriltag_detector.setAprilTagQuadDecimate(2);
    BENCHMARK("Benchmark Apriltag detection: tag36_11 1920x1080 decimate=2")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };

    apriltag_detector.setAprilTagQuadDecimate(3);
    BENCHMARK("Benchmark Apriltag detection: tag36_11 1920x1080 decimate=3")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };
  }

  SECTION("tag21_07")
  {
    std::string filename = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                                     "AprilTag/benchmark/1920x1080/tag21_07_1920x1080.png");
    REQUIRE(vpIoTools::checkFilename(filename));
    vpImage<unsigned char> I;
    vpImageIo::read(I, filename);

    vpDetectorAprilTag apriltag_detector(vpDetectorAprilTag::TAG_CIRCLE21h7);
    BENCHMARK("Benchmark Apriltag detection: tag21_07 1920x1080")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };

    apriltag_detector.setAprilTagQuadDecimate(2);
    BENCHMARK("Benchmark Apriltag detection: tag21_07 1920x1080 decimate=2")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };

    apriltag_detector.setAprilTagQuadDecimate(3);
    BENCHMARK("Benchmark Apriltag detection: tag21_07 1920x1080 decimate=3")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };
  }

#if defined(VISP_HAVE_APRILTAG_BIG_FAMILY)
  SECTION("tag49_12")
  {
    std::string filename = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                                     "AprilTag/benchmark/1920x1080/tag49_12_1920x1080.png");
    REQUIRE(vpIoTools::checkFilename(filename));
    vpImage<unsigned char> I;
    vpImageIo::read(I, filename);

    vpDetectorAprilTag apriltag_detector(vpDetectorAprilTag::TAG_CIRCLE49h12);
    BENCHMARK("Benchmark Apriltag detection: tag49_12 1920x1080")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };

    apriltag_detector.setAprilTagQuadDecimate(2);
    BENCHMARK("Benchmark Apriltag detection: tag49_12 1920x1080 decimate=2")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };

    apriltag_detector.setAprilTagQuadDecimate(3);
    BENCHMARK("Benchmark Apriltag detection: tag49_12 1920x1080 decimate=3")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };
  }

  SECTION("tag48_12")
  {
    std::string filename = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                                     "AprilTag/benchmark/1920x1080/tag48_12_1920x1080.png");
    REQUIRE(vpIoTools::checkFilename(filename));
    vpImage<unsigned char> I;
    vpImageIo::read(I, filename);

    vpDetectorAprilTag apriltag_detector(vpDetectorAprilTag::TAG_CUSTOM48h12);
    BENCHMARK("Benchmark Apriltag detection: tag48_12 1920x1080")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };

    apriltag_detector.setAprilTagQuadDecimate(2);
    BENCHMARK("Benchmark Apriltag detection: tag48_12 1920x1080 decimate=2")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };

    apriltag_detector.setAprilTagQuadDecimate(3);
    BENCHMARK("Benchmark Apriltag detection: tag48_12 1920x1080 decimate=3")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };
  }

  SECTION("tag41_12")
  {
    std::string filename = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                                     "AprilTag/benchmark/1920x1080/tag41_12_1920x1080.png");
    REQUIRE(vpIoTools::checkFilename(filename));
    vpImage<unsigned char> I;
    vpImageIo::read(I, filename);

    vpDetectorAprilTag apriltag_detector(vpDetectorAprilTag::TAG_STANDARD41h12);
    BENCHMARK("Benchmark Apriltag detection: tag41_12 1920x1080")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };

    apriltag_detector.setAprilTagQuadDecimate(2);
    BENCHMARK("Benchmark Apriltag detection: tag41_12 1920x1080 decimate=2")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };

    apriltag_detector.setAprilTagQuadDecimate(3);
    BENCHMARK("Benchmark Apriltag detection: tag41_12 1920x1080 decimate=3")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };
  }

  SECTION("tag52_13")
  {
    std::string filename = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                                     "AprilTag/benchmark/1920x1080/tag52_13_1920x1080.png");
    REQUIRE(vpIoTools::checkFilename(filename));
    vpImage<unsigned char> I;
    vpImageIo::read(I, filename);

    vpDetectorAprilTag apriltag_detector(vpDetectorAprilTag::TAG_STANDARD52h13);
    BENCHMARK("Benchmark Apriltag detection: tag52_13 1920x1080")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };

    apriltag_detector.setAprilTagQuadDecimate(2);
    BENCHMARK("Benchmark Apriltag detection: tag52_13 1920x1080 decimate=2")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };

    apriltag_detector.setAprilTagQuadDecimate(3);
    BENCHMARK("Benchmark Apriltag detection: tag52_13 1920x1080 decimate=3")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };
  }
#endif
}

TEST_CASE("Benchmark Apriltag detection 640x480", "[benchmark]")
{
  const double tagSize = 0.25;
  const vpCameraParameters cam(700, 700, 320, 240);
  const size_t nbTags = 5;

  SECTION("tag16_05")
  {
    std::string filename = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                                     "AprilTag/benchmark/640x480/tag16_05_640x480.png");
    REQUIRE(vpIoTools::checkFilename(filename));
    vpImage<unsigned char> I;
    vpImageIo::read(I, filename);

    vpDetectorAprilTag apriltag_detector(vpDetectorAprilTag::TAG_16h5);
    BENCHMARK("Benchmark Apriltag detection: tag16_05 640x480")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };
  }

  SECTION("tag25_09")
  {
    std::string filename = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                                     "AprilTag/benchmark/640x480/tag25_09_640x480.png");
    REQUIRE(vpIoTools::checkFilename(filename));
    vpImage<unsigned char> I;
    vpImageIo::read(I, filename);

    vpDetectorAprilTag apriltag_detector(vpDetectorAprilTag::TAG_25h9);
    BENCHMARK("Benchmark Apriltag detection: tag25_09 640x480")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };
  }

  SECTION("tag36_11")
  {
    std::string filename = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                                     "AprilTag/benchmark/640x480/tag36_11_640x480.png");
    REQUIRE(vpIoTools::checkFilename(filename));
    vpImage<unsigned char> I;
    vpImageIo::read(I, filename);

    vpDetectorAprilTag apriltag_detector(vpDetectorAprilTag::TAG_36h11);
    BENCHMARK("Benchmark Apriltag detection: tag36_11 640x480")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };
  }

  SECTION("tag21_07")
  {
    std::string filename = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                                     "AprilTag/benchmark/640x480/tag21_07_640x480.png");
    REQUIRE(vpIoTools::checkFilename(filename));
    vpImage<unsigned char> I;
    vpImageIo::read(I, filename);

    vpDetectorAprilTag apriltag_detector(vpDetectorAprilTag::TAG_CIRCLE21h7);
    BENCHMARK("Benchmark Apriltag detection: tag21_07 640x480")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };
  }

#if defined(VISP_HAVE_APRILTAG_BIG_FAMILY)
  SECTION("tag49_12")
  {
    std::string filename = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                                     "AprilTag/benchmark/640x480/tag49_12_640x480.png");
    REQUIRE(vpIoTools::checkFilename(filename));
    vpImage<unsigned char> I;
    vpImageIo::read(I, filename);

    vpDetectorAprilTag apriltag_detector(vpDetectorAprilTag::TAG_CIRCLE49h12);
    BENCHMARK("Benchmark Apriltag detection: tag49_12 640x480")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };
  }

  SECTION("tag48_12")
  {
    std::string filename = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                                     "AprilTag/benchmark/640x480/tag48_12_640x480.png");
    REQUIRE(vpIoTools::checkFilename(filename));
    vpImage<unsigned char> I;
    vpImageIo::read(I, filename);

    vpDetectorAprilTag apriltag_detector(vpDetectorAprilTag::TAG_CUSTOM48h12);
    BENCHMARK("Benchmark Apriltag detection: tag48_12 640x480")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };
  }

  SECTION("tag41_12")
  {
    std::string filename = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                                     "AprilTag/benchmark/640x480/tag41_12_640x480.png");
    REQUIRE(vpIoTools::checkFilename(filename));
    vpImage<unsigned char> I;
    vpImageIo::read(I, filename);

    vpDetectorAprilTag apriltag_detector(vpDetectorAprilTag::TAG_STANDARD41h12);
    BENCHMARK("Benchmark Apriltag detection: tag41_12 640x480")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };
  }

  SECTION("tag52_13")
  {
    std::string filename = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                                     "AprilTag/benchmark/640x480/tag52_13_640x480.png");
    REQUIRE(vpIoTools::checkFilename(filename));
    vpImage<unsigned char> I;
    vpImageIo::read(I, filename);

    vpDetectorAprilTag apriltag_detector(vpDetectorAprilTag::TAG_STANDARD52h13);
    BENCHMARK("Benchmark Apriltag detection: tag52_13 640x480")
    {
      std::vector<vpHomogeneousMatrix> cMo_vec;
      apriltag_detector.detect(I, tagSize, cam, cMo_vec);
      CHECK(cMo_vec.size() == nbTags);
      return cMo_vec;
    };
  }
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
int main() { return EXIT_SUCCESS; }
#endif
