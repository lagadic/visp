/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2022 by Inria. All rights reserved.
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
 * Test additional math functions such as lon-lat generator.
 *
 *****************************************************************************/

/*!
  \example testMathLonLat.cpp

  Test additional math functions such as lon-lat generator.
*/
#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_CATCH2

#include <visp3/core/vpMath.h>
#include <visp3/core/vpHomogeneousMatrix.h>

#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

TEST_CASE("Lon-Lat generator", "[math_lonlat]")
{
  const int lonStart = 0, lonEnd = 180, nlon = 180/10;
  const int latStart = 0, latEnd = 90, nLat = 90/10;
  std::vector<double> longitudes = vpMath::linspace(lonStart, lonEnd, nlon);
  std::vector<double> latitudes = vpMath::linspace(latStart, latEnd, nLat);
  const double radius = 2;

  SECTION("NED")
  {
    std::vector<vpHomogeneousMatrix> ecef_M_ned_vec =
      vpMath::getLocalTangentPlaneTransformations(longitudes, latitudes, radius, vpMath::ned2ecef);
    for (const auto &ecef_M_ned : ecef_M_ned_vec) {
      CHECK(ecef_M_ned.isValid());
      CHECK(ecef_M_ned.getRotationMatrix().isARotationMatrix());
      CHECK(vpMath::equal(ecef_M_ned.getTranslationVector().sumSquare(), radius*radius));
    }
  }

  SECTION("ENU")
  {
    std::vector<vpHomogeneousMatrix> ecef_M_enu_vec =
      vpMath::getLocalTangentPlaneTransformations(longitudes, latitudes, radius, vpMath::enu2ecef);
    for (const auto &ecef_M_enu : ecef_M_enu_vec) {
      CHECK(ecef_M_enu.isValid());
      CHECK(ecef_M_enu.getRotationMatrix().isARotationMatrix());
      CHECK(vpMath::equal(ecef_M_enu.getTranslationVector().sumSquare(), radius*radius));
    }
  }
}

TEST_CASE("Equidistributed sphere point", "[math_equi_sphere_pts]")
{
  const unsigned int maxPoints = 200;
  std::pair<std::vector<double>, std::vector<double> > lonlat_vec = vpMath::computeRegularPointsOnSphere(maxPoints);
  const double radius = 2;

  SECTION("NED")
  {
    std::vector<vpHomogeneousMatrix> ecef_M_ned_vec =
      vpMath::getLocalTangentPlaneTransformations(lonlat_vec.first, lonlat_vec.second, radius, vpMath::ned2ecef);
    CHECK(!ecef_M_ned_vec.empty());
    for (const auto &ecef_M_ned : ecef_M_ned_vec) {
      CHECK(ecef_M_ned.isValid());
      CHECK(ecef_M_ned.getRotationMatrix().isARotationMatrix());
      CHECK(vpMath::equal(ecef_M_ned.getTranslationVector().sumSquare(), radius*radius));
    }
  }

  SECTION("ENU")
  {
    std::vector<vpHomogeneousMatrix> ecef_M_enu_vec =
      vpMath::getLocalTangentPlaneTransformations(lonlat_vec.first, lonlat_vec.second, radius, vpMath::enu2ecef);
    CHECK(!ecef_M_enu_vec.empty());
    for (const auto &ecef_M_enu : ecef_M_enu_vec) {
      CHECK(ecef_M_enu.isValid());
      CHECK(ecef_M_enu.getRotationMatrix().isARotationMatrix());
      CHECK(vpMath::equal(ecef_M_enu.getTranslationVector().sumSquare(), radius*radius));
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
#include <iostream>

int main() { return 0; }
#endif
