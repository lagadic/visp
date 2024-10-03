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
  \example catchImageResize.cpp

  \brief Test image resize.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2)

#include "common.hpp"
#include <catch_amalgamated.hpp>
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
  Catch::Session session;

  using namespace Catch::Clara;
  auto cli = session.cli()
    | Catch::Clara::Opt(g_input_width, "g_input_width")["--iw"]("Input image width.")
    | Catch::Clara::Opt(g_input_height, "g_input_height")["--ih"]("Input image height.")
    | Catch::Clara::Opt(g_output_width, "g_output_width")["--ow"]("Output image width.")
    | Catch::Clara::Opt(g_output_height, "g_output_height")["--oh"]("Output image height.");

  session.cli(cli);
  session.applyCommandLine(argc, argv);

  std::cout << "Input image (wxh): " << g_input_width << "x" << g_input_height << std::endl;
  std::cout << "Output image (wxh): " << g_output_width << "x" << g_output_height << std::endl;

  int numFailed = session.run();
  return numFailed;
}
#else
int main() { return EXIT_SUCCESS; }
#endif
