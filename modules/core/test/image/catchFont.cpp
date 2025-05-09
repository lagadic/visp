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
 * Test draw text with vpFont.
 */

/*!
  \example catchFont.cpp

  \brief Test draw text with vpFont.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2)

#include "common.hpp"
#include <catch_amalgamated.hpp>
#include <visp3/core/vpFont.h>


#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif
TEST_CASE("Segmentation fault reproducer", "[draw_text_font]")
{
  std::ostringstream oss;
  oss << "Computation time: " << std::fixed << std::setprecision(2) << 10.5168 << " ms/";

  SECTION("GENERIC_MONOSPACE")
  {
    vpFont font(20, vpFont::GENERIC_MONOSPACE);

    SECTION("unsigned char")
    {
      REQUIRE_NOTHROW([&]() {
        vpImage<unsigned char> I(32, 256);
        font.drawText(I, oss.str(), vpImagePoint(10, 10), 200);
      }());
    }

    SECTION("vpRGBa")
    {
      REQUIRE_NOTHROW([&]() {
        vpImage<vpRGBa> I(32, 256);
        font.drawText(I, oss.str(), vpImagePoint(10, 10), vpColor::red);
      }());
    }
  }

  SECTION("TRUETYPE_FILE")
  {
    vpFont font(20, vpFont::TRUETYPE_FILE);

    SECTION("unsigned char")
    {
      REQUIRE_NOTHROW([&]() {
        vpImage<unsigned char> I(32, 256);
        font.drawText(I, oss.str(), vpImagePoint(10, 10), 200);
      }());
    }

    SECTION("vpRGBa")
    {
      REQUIRE_NOTHROW([&]() {
        vpImage<vpRGBa> I(32, 256);
        font.drawText(I, oss.str(), vpImagePoint(10, 10), vpColor::red);
      }());
    }
  }
}

TEST_CASE("Sanity check", "[draw_text_font]")
{
  const std::string text = "ViSP standing for Visual Servoing Platform is a modular cross platform library. "
    "0123456789 ,;:!?./§ &é'(-è_çà)=";

  SECTION("GENERIC_MONOSPACE")
  {
    vpFont font(16, vpFont::GENERIC_MONOSPACE);

    SECTION("unsigned char")
    {
      REQUIRE_NOTHROW([&]() {
        vpImage<unsigned char> I(32, 512);
        font.drawText(I, text, vpImagePoint(10, 10), 200);
      }());
    }

    SECTION("vpRGBa")
    {
      REQUIRE_NOTHROW([&]() {
        vpImage<vpRGBa> I(32, 512);
        font.drawText(I, text, vpImagePoint(10, 10), vpColor::red);
      }());
    }
  }

  SECTION("TRUETYPE_FILE")
  {
    vpFont font(16, vpFont::TRUETYPE_FILE);

    SECTION("unsigned char")
    {
      REQUIRE_NOTHROW([&]() {
        vpImage<unsigned char> I(32, 512);
        font.drawText(I, text, vpImagePoint(10, 10), 200);
      }());
    }

    SECTION("vpRGBa")
    {
      REQUIRE_NOTHROW([&]() {
        vpImage<vpRGBa> I(32, 512);
        font.drawText(I, text, vpImagePoint(10, 10), vpColor::red);
      }());
    }
  }
}

int main(int argc, char *argv[])
{
  Catch::Session session;
  session.applyCommandLine(argc, argv);
  int numFailed = session.run();
  return numFailed;
}
#else
int main() { return EXIT_SUCCESS; }
#endif
