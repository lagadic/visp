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
  \example perfImageLoadSave.cpp
 */

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2) && defined(VISP_HAVE_THREADS)
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include <thread>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

static std::string ipath = vpIoTools::getViSPImagesDataPath();
static std::vector<std::string> paths {
    ipath + "/Solvay/Solvay_conference_1927_Version2_640x440",
    ipath + "/Solvay/Solvay_conference_1927_Version2_1024x705",
    ipath + "/Solvay/Solvay_conference_1927_Version2_1280x881",
    ipath + "/Solvay/Solvay_conference_1927_Version2_2126x1463",
};
static std::vector<std::string> names { "Solvay (640x440)", "Solvay (1024x705)", "Solvay (1280x881)",
                                      "Solvay (2126x1463)" };
static std::vector<vpImageIo::vpImageIoBackendType> backends
{
#if defined(VISP_HAVE_JPEG) && defined(VISP_HAVE_PNG)
  vpImageIo::IO_SYSTEM_LIB_BACKEND,
#endif
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS)
  vpImageIo::IO_OPENCV_BACKEND,
#endif
#if defined(VISP_HAVE_SIMDLIB)
  vpImageIo::IO_SIMDLIB_BACKEND,
#endif
  vpImageIo::IO_STB_IMAGE_BACKEND
};
static std::vector<std::string> backendNamesJpeg
{
#if defined(VISP_HAVE_JPEG)
  "libjpeg",
#endif
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS)
      "OpenCV",
#endif
      "simd", "stb"
};
static std::vector<std::string> backendNamesPng
{
#if defined(VISP_HAVE_PNG)
  "libpng",
#endif
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS)
      "OpenCV",
#endif
      "simd", "stb"
};
static int nThreads = 0;

TEST_CASE("Benchmark grayscale JPEG image loading", "[benchmark]")
{
  for (size_t i = 0; i < paths.size(); i++) {
    SECTION(names[i])
    {
      for (size_t j = 0; j < backends.size(); j++) {
        vpImage<vpRGBa> I;

        BENCHMARK(backendNamesJpeg[j] + " backend")
        {
          vpImageIo::read(I, paths[i] + ".jpg", backends[j]);
          return I;
        };
      }
    }
  }
}

TEST_CASE("Benchmark RGBA JPEG image loading", "[benchmark]")
{
  for (size_t i = 0; i < paths.size(); i++) {
    SECTION(names[i])
    {
      for (size_t j = 0; j < backends.size(); j++) {
        vpImage<unsigned char> I;

        BENCHMARK(backendNamesJpeg[j] + " backend")
        {
          vpImageIo::read(I, paths[i] + ".jpg", backends[j]);
          return I;
        };
      }
    }
  }
}

TEST_CASE("Benchmark grayscale PNG image loading", "[benchmark]")
{
  for (size_t i = 0; i < paths.size(); i++) {
    SECTION(names[i])
    {
      for (size_t j = 0; j < backends.size(); j++) {
        vpImage<vpRGBa> I;

        BENCHMARK(backendNamesPng[j] + " backend")
        {
          vpImageIo::read(I, paths[i] + ".png", backends[j]);
          return I;
        };
      }
    }
  }
}

TEST_CASE("Benchmark RGBA PNG image loading", "[benchmark]")
{
  for (size_t i = 0; i < paths.size(); i++) {
    SECTION(names[i])
    {
      for (size_t j = 0; j < backends.size(); j++) {
        vpImage<unsigned char> I;

        BENCHMARK(backendNamesPng[j] + " backend")
        {
          vpImageIo::read(I, paths[i] + ".png", backends[j]);
          return I;
        };
      }
    }
  }
}

TEST_CASE("Benchmark grayscale JPEG image saving", "[benchmark]")
{
  std::string tmp_dir = vpIoTools::makeTempDirectory(vpIoTools::getTempPath());
  std::string directory_filename_tmp =
    tmp_dir + "/vpIoTools_perfImageLoadSave_" + vpTime::getDateTime("%Y-%m-%d_%H.%M.%S");
  vpIoTools::makeDirectory(directory_filename_tmp);
  REQUIRE(vpIoTools::checkDirectory(directory_filename_tmp));

  for (size_t i = 0; i < paths.size(); i++) {
    vpImage<unsigned char> I;
    vpImageIo::read(I, paths[i] + ".png");

    SECTION(names[i])
    {
      for (size_t j = 0; j < backends.size(); j++) {
        BENCHMARK(backendNamesJpeg[j] + " backend")
        {
          vpImageIo::write(I, directory_filename_tmp + "/ViSP_tmp_perf_write.jpg", backends[j]);
          return I;
        };
      }
    }
  }

  REQUIRE(vpIoTools::remove(directory_filename_tmp));
}

TEST_CASE("Benchmark RGBA JPEG image saving", "[benchmark]")
{
  std::string tmp_dir = vpIoTools::makeTempDirectory(vpIoTools::getTempPath());
  std::string directory_filename_tmp =
    tmp_dir + "/vpIoTools_perfImageLoadSave_" + vpTime::getDateTime("%Y-%m-%d_%H.%M.%S");
  vpIoTools::makeDirectory(directory_filename_tmp);
  REQUIRE(vpIoTools::checkDirectory(directory_filename_tmp));

  for (size_t i = 0; i < paths.size(); i++) {
    vpImage<vpRGBa> I;
    vpImageIo::read(I, paths[i] + ".png");

    SECTION(names[i])
    {
      for (size_t j = 0; j < backends.size(); j++) {
        BENCHMARK(backendNamesJpeg[j] + " backend")
        {
          vpImageIo::write(I, directory_filename_tmp + "/ViSP_tmp_perf_write.jpg", backends[j]);
          return I;
        };
      }
    }
  }

  REQUIRE(vpIoTools::remove(directory_filename_tmp));
}

TEST_CASE("Benchmark grayscale PNG image saving", "[benchmark]")
{
  std::string tmp_dir = vpIoTools::makeTempDirectory(vpIoTools::getTempPath());
  std::string directory_filename_tmp =
    tmp_dir + "/vpIoTools_perfImageLoadSave_" + vpTime::getDateTime("%Y-%m-%d_%H.%M.%S");
  vpIoTools::makeDirectory(directory_filename_tmp);
  REQUIRE(vpIoTools::checkDirectory(directory_filename_tmp));

  for (size_t i = 0; i < paths.size(); i++) {
    vpImage<unsigned char> I;
    vpImageIo::read(I, paths[i] + ".png");

    SECTION(names[i])
    {
      for (size_t j = 0; j < backends.size(); j++) {
        BENCHMARK(backendNamesPng[j] + " backend")
        {
          vpImageIo::write(I, directory_filename_tmp + "/ViSP_tmp_perf_write.png", backends[j]);
          return I;
        };
      }
    }
  }

  REQUIRE(vpIoTools::remove(directory_filename_tmp));
}

TEST_CASE("Benchmark RGBA PNG image saving", "[benchmark]")
{
  std::string tmp_dir = vpIoTools::makeTempDirectory(vpIoTools::getTempPath());
  std::string directory_filename_tmp =
    tmp_dir + "/vpIoTools_perfImageLoadSave_" + vpTime::getDateTime("%Y-%m-%d_%H.%M.%S");
  vpIoTools::makeDirectory(directory_filename_tmp);
  REQUIRE(vpIoTools::checkDirectory(directory_filename_tmp));

  for (size_t i = 0; i < paths.size(); i++) {
    vpImage<vpRGBa> I;
    vpImageIo::read(I, paths[i] + ".png");

    SECTION(names[i])
    {
      for (size_t j = 0; j < backends.size(); j++) {
        BENCHMARK(backendNamesPng[j] + " backend")
        {
          vpImageIo::write(I, directory_filename_tmp + "/ViSP_tmp_perf_write.png", backends[j]);
          return I;
        };
      }
    }
  }

  REQUIRE(vpIoTools::remove(directory_filename_tmp));
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
    ("Run benchmark") |
    Opt(nThreads, "nThreads")["--nThreads"]("Number of threads");

// Now pass the new composite back to Catch so it uses that
  session.cli(cli);

  // Let Catch (using Clara) parse the command line
  session.applyCommandLine(argc, argv);

  if (runBenchmark) {
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
