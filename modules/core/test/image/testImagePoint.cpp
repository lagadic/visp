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
 * Test for vpImagePoint class.
 */
/*!
  \example testImagePoint.cpp

  \brief Test vpImagePoint functionalities.

*/

#include <iostream>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImagePoint.h>

#if defined(VISP_HAVE_CATCH2)
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif
TEST_CASE("Test comparison operator", "[operator]")
{
  vpImagePoint ip1, ip2, ip3;

  ip1.set_u(-11.1);
  ip1.set_v(10);

  ip2.set_j(-11.1);
  ip2.set_i(10);

  ip3.set_j(11.10001);
  ip3.set_i(10.1);
  CHECK(ip1 == ip2);
  CHECK_FALSE(ip1 != ip2);
  CHECK_FALSE(ip1 == ip3);
  CHECK(ip1 != ip3);
}

TEST_CASE("Test pixel belongs to segment", "[operator]")
{
  vpImagePoint start_pixel(10, 10);
  SECTION("Segment horizontal right")
  {
    vpImagePoint end_pixel = start_pixel + vpImagePoint(0, 5);
    auto j = start_pixel.get_j();
    for (auto curr_pixel = start_pixel; curr_pixel.inSegment(start_pixel, end_pixel);
         curr_pixel = curr_pixel.nextInSegment(start_pixel, end_pixel), j++) {
      CHECK(curr_pixel == vpImagePoint(start_pixel.get_i(), j));
      if (curr_pixel == end_pixel)
        break;
    }
  }
  SECTION("Segment horizontal left")
  {
    vpImagePoint end_pixel = start_pixel - vpImagePoint(0, 5);
    auto j = start_pixel.get_j();
    for (auto curr_pixel = start_pixel; curr_pixel.inSegment(start_pixel, end_pixel);
         curr_pixel = curr_pixel.nextInSegment(start_pixel, end_pixel), j--) {
      CHECK(curr_pixel == vpImagePoint(start_pixel.get_i(), j));
      if (curr_pixel == end_pixel)
        break;
    }
  }
  SECTION("Segment vertical bottom")
  {
    vpImagePoint end_pixel = start_pixel + vpImagePoint(5, 0);
    auto i = start_pixel.get_i();
    for (auto curr_pixel = start_pixel; curr_pixel.inSegment(start_pixel, end_pixel);
         curr_pixel = curr_pixel.nextInSegment(start_pixel, end_pixel), i++) {
      CHECK(curr_pixel == vpImagePoint(i, start_pixel.get_j()));
      if (curr_pixel == end_pixel)
        break;
    }
  }
  SECTION("Segment vertical top")
  {
    vpImagePoint end_pixel = start_pixel - vpImagePoint(5, 0);
    auto i = start_pixel.get_i();
    for (auto curr_pixel = start_pixel; curr_pixel.inSegment(start_pixel, end_pixel);
         curr_pixel = curr_pixel.nextInSegment(start_pixel, end_pixel), i--) {
      CHECK(curr_pixel == vpImagePoint(i, start_pixel.get_j()));
      if (curr_pixel == end_pixel)
        break;
    }
  }
}

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance

  // Let Catch (using Clara) parse the command line
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();

  // numFailed is clamped to 255 as some unices only use the lower 8 bits.
  // This clamping has already been applied, so just return it here
  // You can also do any post run clean-up here
  return numFailed;
}

#else
int main()
{
  vpImagePoint ip1, ip2, ip3;

  ip1.set_u(-11.1);
  ip1.set_v(10);

  ip2.set_j(-11.1);
  ip2.set_i(10);

  ip3.set_j(11.10001);
  ip3.set_i(10.1);

  std::cout << "We define ip1 with coordinates: " << ip1 << std::endl;

  std::cout << "We define ip2 with coordinates: " << ip2 << std::endl;

  std::cout << "We define ip3 with coordinates: " << ip3 << std::endl;

  if (ip1 == ip2) {
    std::cout << "ip1 == ip2" << std::endl;
  }
  else {
    std::cout << "ip1 != ip2 (bad result)" << std::endl;
    return EXIT_FAILURE;
  }

  if (ip1 != ip2) {
    std::cout << "ip1 != ip2 (bad result)" << std::endl;
    return EXIT_FAILURE;
  }
  else {
    std::cout << "ip1 == ip2" << std::endl;
  }

  if (ip1 == ip3) {
    std::cout << "ip1 == ip3 (bad result)" << std::endl;
    return EXIT_FAILURE;
  }
  else {
    std::cout << "ip1 != ip3" << std::endl;
  }

  if (ip1 != ip3) {
    std::cout << "ip1 != ip3" << std::endl;
  }
  else {
    std::cout << "ip1 == ip3 (bad result)" << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
#endif
