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
 * Test vpPolygon class.
 */

/*!
  \example catchPolygon2.cpp

  \brief Test vpPolygon class.
*/
#include <visp3/core/vpPolygon.h>

// Core
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpRect.h>

#if defined(VISP_HAVE_CATCH2)

#include <catch_amalgamated.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

TEST_CASE("Check polygon construction")
{
  SECTION("From vpRect")
  {
    const vpRect rect { 0, 0, 200, 400 };
    const std::vector<vpImagePoint> rect_corners { rect.getTopLeft(), rect.getTopRight(), rect.getBottomRight(),
                                                 rect.getBottomLeft() };

    vpPolygon poly {};
    poly.build(rect_corners, true);

  // Check if std:c++14 or higher
#if ((__cplusplus >= 201402L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201402L)))
    for (const auto &poly_corner : poly.getCorners()) {
      REQUIRE(std::find(cbegin(rect_corners), cend(rect_corners), poly_corner) != cend(rect_corners));
    }
#else
    for (const auto &poly_corner : poly.getCorners()) {
      REQUIRE(std::find(begin(rect_corners), end(rect_corners), poly_corner) != end(rect_corners));
    }
#endif
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

int main()
{
  return EXIT_SUCCESS;
}
#endif
