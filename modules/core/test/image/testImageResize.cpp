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
 * Test image resize.
 */

/*!
  \example testImageResize.cpp

  \brief Test image resize.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2)
#define CATCH_CONFIG_RUNNER
#include "common.hpp"
#include <catch.hpp>
#include <visp3/core/vpImageTools.h>

static unsigned int g_input_width = 7;
static unsigned int g_input_height = 5;
static unsigned int g_output_width = 4;
static unsigned int g_output_height = 3;

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif
TEST_CASE("Nearest neighbor interpolation", "[image_resize]")
{
  SECTION("unsigned char")
  {
    vpImage<unsigned char> I(g_input_height, g_input_width);
    common_tools::fill(I);
    vpImage<unsigned char> Iresize_ref(g_output_height, g_output_width);
    common_tools::resizeRef(I, Iresize_ref, common_tools::g_nearest_neighbor);

    vpImage<unsigned char> Iresize;
    vpImageTools::resize(I, Iresize, g_output_width, g_output_height, vpImageTools::INTERPOLATION_NEAREST);

    std::cout << "I:\n" << I << std::endl;
    std::cout << "Iresize_ref:\n" << Iresize_ref << std::endl;
    std::cout << "Iresize:\n" << Iresize << std::endl;

    CHECK((Iresize == Iresize_ref));
  }

  SECTION("vpRGBa")
  {
    vpImage<vpRGBa> I(g_input_height, g_input_width);
    common_tools::fill(I);
    vpImage<vpRGBa> Iresize_ref(g_output_height, g_output_width);
    common_tools::resizeRef(I, Iresize_ref, common_tools::g_nearest_neighbor);

    vpImage<vpRGBa> Iresize;
    vpImageTools::resize(I, Iresize, g_output_width, g_output_height, vpImageTools::INTERPOLATION_NEAREST);

    std::cout << "I:\n" << I << std::endl;
    std::cout << "Iresize_ref:\n" << Iresize_ref << std::endl;
    std::cout << "Iresize:\n" << Iresize << std::endl;

    CHECK((Iresize == Iresize_ref));
  }
}

TEST_CASE("Bilinear interpolation", "[image_resize]")
{
  SECTION("unsigned char")
  {
    vpImage<unsigned char> I(g_input_height, g_input_width);
    common_tools::fill(I);
    vpImage<unsigned char> Iresize_ref(g_output_height, g_output_width);
    common_tools::resizeRef(I, Iresize_ref, common_tools::g_bilinear);

    vpImage<unsigned char> Iresize;
    vpImageTools::resize(I, Iresize, g_output_width, g_output_height, vpImageTools::INTERPOLATION_LINEAR);

    std::cout << "I:\n" << I << std::endl;
    std::cout << "Iresize_ref:\n" << Iresize_ref << std::endl;
    std::cout << "Iresize:\n" << Iresize << std::endl;

    CHECK((Iresize == Iresize_ref));
  }

  SECTION("vpRGBa")
  {
    vpImage<vpRGBa> I(g_input_height, g_input_width);
    common_tools::fill(I);
    vpImage<vpRGBa> Iresize_ref(g_output_height, g_output_width);
    common_tools::resizeRef(I, Iresize_ref, common_tools::g_bilinear);

    vpImage<vpRGBa> Iresize;
    vpImageTools::resize(I, Iresize, g_output_width, g_output_height, vpImageTools::INTERPOLATION_LINEAR);

    std::cout << "I:\n" << I << std::endl;
    std::cout << "Iresize_ref:\n" << Iresize_ref << std::endl;
    std::cout << "Iresize:\n" << Iresize << std::endl;

    const double max_pixel_error = 0.5;
    double error = 0.0;
    CHECK(common_tools::almostEqual(Iresize, Iresize_ref, max_pixel_error, error));
    std::cout << "Error: " << error << std::endl;
  }
}

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance

  // Build a new parser on top of Catch's
  using namespace Catch::clara;
  auto cli = session.cli()                           // Get Catch's composite command line parser
    | Opt(g_input_width, "g_input_width")   // bind variable to a new option, with a hint string
    ["--iw"]                          // the option names it will respond to
    ("Input image width.")                  // description string for the help output
    | Opt(g_input_height, "g_input_height") // bind variable to a new option, with a hint string
    ["--ih"]                          // the option names it will respond to
    ("Input image height.") |
    Opt(g_output_width, "g_output_width") // bind variable to a new option, with a hint string
    ["--ow"]                          // the option names it will respond to
    ("Output image width.") |
    Opt(g_output_height, "g_output_height") // bind variable to a new option, with a hint string
    ["--oh"]                            // the option names it will respond to
    ("Output image height.");

// Now pass the new composite back to Catch so it uses that
  session.cli(cli);

  // Let Catch (using Clara) parse the command line
  session.applyCommandLine(argc, argv);

  std::cout << "Input image (wxh): " << g_input_width << "x" << g_input_height << std::endl;
  std::cout << "Output image (wxh): " << g_output_width << "x" << g_output_height << std::endl;

  int numFailed = session.run();

  // numFailed is clamped to 255 as some unices only use the lower 8 bits.
  // This clamping has already been applied, so just return it here
  // You can also do any post run clean-up here
  return numFailed;
}
#else
int main() { return EXIT_SUCCESS; }
#endif
