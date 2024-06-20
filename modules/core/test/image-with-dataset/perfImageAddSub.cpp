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
 * Test image addition / subtraction.
 */

#include <visp3/core/vpConfig.h>

/*!
  \example perfImageAddSub.cpp

  \brief Test images addition / subtraction performances.
*/

#if defined(VISP_HAVE_CATCH2)
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#define CATCH_CONFIG_RUNNER
#include "common.hpp"
#include <catch.hpp>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif
TEST_CASE("Benchmark vpImageTools::imageAdd()", "[benchmark]")
{
  const std::string filepath = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Klimt/Klimt.pgm");
  vpImage<unsigned char> I;
  vpImageIo::read(I, filepath);

  vpImage<unsigned char> I2(I.getHeight(), I.getWidth());
  common_tools::fill(I2);

  vpImage<unsigned char> Iadd(I.getHeight(), I.getWidth());
  SECTION("Without saturation")
  {
    const bool saturation = false;

    BENCHMARK("Benchmark naive imageAdd() code without saturation")
    {
      common_tools::imageAddRef(I, I2, Iadd, saturation);
      return I;
    };

    BENCHMARK("Benchmark ViSP imageAdd() code without saturation")
    {
      vpImageTools::imageAdd(I, I2, Iadd, saturation);
      return I;
    };
  }
  SECTION("With saturation")
  {
    const bool saturation = true;

    BENCHMARK("Benchmark naive imageAdd() code with saturation")
    {
      common_tools::imageAddRef(I, I2, Iadd, saturation);
      return I;
    };

    BENCHMARK("Benchmark ViSP imageAdd() code with saturation")
    {
      vpImageTools::imageAdd(I, I2, Iadd, saturation);
      return I;
    };
  }
}

TEST_CASE("Benchmark vpImageTools::imageSubtract()", "[benchmark]")
{
  const std::string filepath = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "Klimt/Klimt.pgm");
  vpImage<unsigned char> I;
  vpImageIo::read(I, filepath);

  vpImage<unsigned char> I2(I.getHeight(), I.getWidth());
  common_tools::fill(I2);

  vpImage<unsigned char> Isub(I.getHeight(), I.getWidth());
  SECTION("Without saturation")
  {
    const bool saturation = false;

    BENCHMARK("Benchmark naive imageSub() code without saturation")
    {
      common_tools::imageSubtractRef(I, I2, Isub, saturation);
      return I;
    };

    BENCHMARK("Benchmark ViSP imageSub() code without saturation")
    {
      vpImageTools::imageSubtract(I, I2, Isub, saturation);
      return I;
    };
  }
  SECTION("With saturation")
  {
    const bool saturation = true;

    BENCHMARK("Benchmark naive imageSub() code with saturation")
    {
      common_tools::imageSubtractRef(I, I2, Isub, saturation);
      return I;
    };

    BENCHMARK("Benchmark ViSP imageSub() code with saturation")
    {
      vpImageTools::imageSubtract(I, I2, Isub, saturation);
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
