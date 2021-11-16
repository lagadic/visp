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
 * Benchmark color image conversion.
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_CATCH2
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include <thread>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>

static std::string ipath = vpIoTools::getViSPImagesDataPath();
static std::string imagePathJpeg = vpIoTools::createFilePath(ipath, "Klimt/Klimt.jpeg");
static std::string imagePathPng = vpIoTools::createFilePath(ipath, "Klimt/Klimt.png");
static std::string imagePathPngBig = vpIoTools::createFilePath(ipath, "Klimt/test_image_resize.png");
static int nThreads = 0;

TEST_CASE("Benchmark Jpeg image loading", "[benchmark]") {
  {
    vpImage<vpRGBa> I;

    BENCHMARK("vpImageIo::read()") {
      vpImageIo::read(I, imagePathJpeg);
      return I;
    };
  }

  {
    vpImage<vpRGBa> I;

    BENCHMARK("vpImageIo::readSimdlib()") {
      vpImageIo::readJPEG(I, imagePathJpeg, vpImageIo::IO_SIMDLIB_BACKEND);
      return I;
    };
  }

  {
    vpImage<vpRGBa> I;

    BENCHMARK("vpImageIo::readStb()") {
      vpImageIo::readJPEG(I, imagePathJpeg, vpImageIo::IO_STB_IMAGE_BACKEND);
      return I;
    };
  }
}

TEST_CASE("Benchmark Png image loading", "[benchmark]") {
  {
    vpImage<vpRGBa> I;

    BENCHMARK("vpImageIo::read()") {
      vpImageIo::read(I, imagePathPng);
      return I;
    };
  }

  {
    vpImage<vpRGBa> I;

    BENCHMARK("vpImageIo::readSimdlib()") {
      vpImageIo::readPNG(I, imagePathPng, vpImageIo::IO_SIMDLIB_BACKEND);
      return I;
    };
  }

  {
    vpImage<vpRGBa> I;

    BENCHMARK("vpImageIo::readStb()") {
      vpImageIo::readPNG(I, imagePathPng, vpImageIo::IO_STB_IMAGE_BACKEND);
      return I;
    };
  }
}

TEST_CASE("Benchmark big Png image loading", "[benchmark]") {
  {
    vpImage<vpRGBa> I;

    BENCHMARK("vpImageIo::read()") {
      vpImageIo::read(I, imagePathPngBig);
      return I;
    };
  }

  {
    vpImage<vpRGBa> I;

    BENCHMARK("vpImageIo::readSimdlib()") {
      vpImageIo::readPNG(I, imagePathPngBig, vpImageIo::IO_SIMDLIB_BACKEND);
      return I;
    };
  }

  {
    vpImage<vpRGBa> I;

    BENCHMARK("vpImageIo::readStb()") {
      vpImageIo::readPNG(I, imagePathPngBig, vpImageIo::IO_STB_IMAGE_BACKEND);
      return I;
    };
  }
}

TEST_CASE("Benchmark Jpeg image saving", "[benchmark]") {
  vpImage<vpRGBa> I;
  vpImageIo::read(I, imagePathJpeg);
  {
    const std::string filename = "/tmp/Klimt_ViSP.jpg";

    BENCHMARK("vpImageIo::write()") {
      vpImageIo::write(I, filename);
      return I;
    };
  }

  {
    const std::string filename = "/tmp/Klimt_Simd.jpg";

    BENCHMARK("vpImageIo::writeSimdlib()") {
      vpImageIo::writeJPEG(I, filename, vpImageIo::IO_SIMDLIB_BACKEND);
      return I;
    };
  }

  {
    const std::string filename = "/tmp/Klimt_stb.jpg";

    BENCHMARK("vpImageIo::writeStb()") {
      vpImageIo::writeJPEG(I, filename, vpImageIo::IO_STB_IMAGE_BACKEND);
      return I;
    };
  }
}

TEST_CASE("Benchmark big Jpeg image saving", "[benchmark]") {
  vpImage<vpRGBa> I;
  vpImageIo::read(I, imagePathPngBig);
  {
    const std::string filename = "/tmp/Big_images_ViSP.jpg";

    BENCHMARK("vpImageIo::write()") {
      vpImageIo::write(I, filename);
      return I;
    };
  }

  {
    const std::string filename = "/tmp/Big_images_Simd.jpg";

    BENCHMARK("vpImageIo::writeSimdlib()") {
      vpImageIo::writeJPEG(I, filename, vpImageIo::IO_SIMDLIB_BACKEND);
      return I;
    };
  }

  {
    const std::string filename = "/tmp/Big_images_stb.jpg";

    BENCHMARK("vpImageIo::writeStb()") {
      vpImageIo::writeJPEG(I, filename, vpImageIo::IO_STB_IMAGE_BACKEND);
      return I;
    };
  }
}

TEST_CASE("Benchmark Png image saving", "[benchmark]") {
  vpImage<vpRGBa> I;
  vpImageIo::read(I, imagePathPng);
  {
    const std::string filename = "/tmp/Klimt_ViSP.png";

    BENCHMARK("vpImageIo::write()") {
      vpImageIo::write(I, filename);
      return I;
    };
  }

  {
    const std::string filename = "/tmp/Klimt_Simd.png";

    BENCHMARK("vpImageIo::writeSimdlib()") {
      vpImageIo::writePNG(I, filename, vpImageIo::IO_SIMDLIB_BACKEND);
      return I;
    };
  }

  {
    const std::string filename = "/tmp/Klimt_stb.png";

    BENCHMARK("vpImageIo::writeStb()") {
      vpImageIo::writePNG(I, filename, vpImageIo::IO_STB_IMAGE_BACKEND);
      return I;
    };
  }
}

TEST_CASE("Benchmark big Png image saving", "[benchmark]") {
  vpImage<vpRGBa> I;
  vpImageIo::read(I, imagePathPngBig);
  {
    const std::string filename = "/tmp/Big_images_ViSP.png";

    BENCHMARK("vpImageIo::write()") {
      vpImageIo::write(I, filename);
      return I;
    };
  }

  {
    const std::string filename = "/tmp/Big_images_Simd.png";

    BENCHMARK("vpImageIo::writeSimdlib()") {
      vpImageIo::writePNG(I, filename, vpImageIo::IO_SIMDLIB_BACKEND);
      return I;
    };
  }

  {
    const std::string filename = "/tmp/Big_images_stb.png";

    BENCHMARK("vpImageIo::writeStb()") {
      vpImageIo::writePNG(I, filename, vpImageIo::IO_STB_IMAGE_BACKEND);
      return I;
    };
  }
}

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance

  bool runBenchmark = false;
  // Build a new parser on top of Catch's
  using namespace Catch::clara;
  auto cli = session.cli() // Get Catch's composite command line parser
    | Opt(runBenchmark)    // bind variable to a new option, with a hint string
    ["--benchmark"]        // the option names it will respond to
    ("run benchmark?")     // description string for the help output
    | Opt(imagePathJpeg, "imagePathColor")
    ["--imagePathColor"]
    ("Path to color image")
    | Opt(imagePathPng, "imagePathColor")
    ["--imagePathGray"]
    ("Path to gray image")
    | Opt(nThreads, "nThreads")
    ["--nThreads"]
    ("Number of threads");

  // Now pass the new composite back to Catch so it uses that
  session.cli(cli);

  // Let Catch (using Clara) parse the command line
  session.applyCommandLine(argc, argv);

  if (runBenchmark) {
//    vpImage<vpRGBa> I_color;
//    vpImageIo::read(I_color, imagePathColor);
//    std::cout << "imagePathColor:\n\t" << imagePathColor << "\n\t" << I_color.getWidth() << "x" << I_color.getHeight() << std::endl;

//    vpImage<unsigned char> I_gray;
//    vpImageIo::read(I_gray, imagePathGray);
//    std::cout << "imagePathGray:\n\t" << imagePathGray << "\n\t" << I_gray.getWidth() << "x" << I_gray.getHeight() << std::endl;
    std::cout << "nThreads: " << nThreads << " / available threads: " << std::thread::hardware_concurrency() << std::endl;

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
