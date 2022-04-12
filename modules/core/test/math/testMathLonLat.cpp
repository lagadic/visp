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

// #define VERBOSE
// #define DEBUG

#ifdef DEBUG
#include <visp3/core/vpIoTools.h>
#endif

TEST_CASE("Lon-Lat generator", "[math_lonlat]")
{
  const int lonStart = 0, lonEnd = 360, nlon = 20;
  const int latStart = 0, latEnd = 90, nLat = 10;
  std::vector<double> longitudes = vpMath::linspace(lonStart, lonEnd, nlon);
  std::vector<double> latitudes = vpMath::linspace(latStart, latEnd, nLat);
  const double radius = 5;

  std::vector<std::pair<double, double> > lonlatVec;
  lonlatVec.reserve(longitudes.size()*latitudes.size());
  for (auto lon : longitudes) {
    for (auto lat : latitudes) {
      lonlatVec.emplace_back(lon, lat);
    }
  }

  SECTION("NED")
  {
    std::vector<vpHomogeneousMatrix> ecef_M_ned_vec =
      vpMath::getLocalTangentPlaneTransformations(lonlatVec, radius, vpMath::ned2ecef);
    for (const auto &ecef_M_ned : ecef_M_ned_vec) {
#ifdef VERBOSE
      std::cout << "Lon-Lat ecef_M_ned:\n" << ecef_M_ned << std::endl;
#endif
      CHECK(ecef_M_ned.isValid());
      CHECK(ecef_M_ned.getRotationMatrix().isARotationMatrix());
      CHECK(vpMath::equal(ecef_M_ned.getTranslationVector().sumSquare(), radius*radius));
    }

#ifdef DEBUG
    vpHomogeneousMatrix ned_M_cv;
    ned_M_cv[0][0] = 0;
    ned_M_cv[0][1] = -1;
    ned_M_cv[1][0] = 1;
    ned_M_cv[1][1] = 0;
    const std::string folder = "NED/lon-lat/";
    vpIoTools::makeDirectory(folder);
    int i = 0;
    for (const auto &ecef_M_ned : ecef_M_ned_vec)
    {
      char buffer[80];
      sprintf(buffer, std::string(folder + "ecef_M_cv_%04d.txt").c_str(), i++);
      std::string filename = buffer;
      std::ofstream file(filename);
      if (file.is_open())
      {
        (ecef_M_ned * ned_M_cv).save(file);
      }
    }
#endif
  }

  SECTION("ENU")
  {
    std::vector<vpHomogeneousMatrix> ecef_M_enu_vec =
      vpMath::getLocalTangentPlaneTransformations(lonlatVec, radius, vpMath::enu2ecef);
    for (const auto &ecef_M_enu : ecef_M_enu_vec) {
#ifdef VERBOSE
      std::cout << "Lon-Lat ecef_M_enu:\n" << ecef_M_enu << std::endl;
#endif
      CHECK(ecef_M_enu.isValid());
      CHECK(ecef_M_enu.getRotationMatrix().isARotationMatrix());
      CHECK(vpMath::equal(ecef_M_enu.getTranslationVector().sumSquare(), radius*radius));

#ifdef DEBUG
    vpHomogeneousMatrix enu_M_cv;
    enu_M_cv[1][1] = -1;
    enu_M_cv[2][2] = -1;
    const std::string folder = "ENU/lon-lat/";
    vpIoTools::makeDirectory(folder);
    int i = 0;
    for (const auto &ecef_M_enu : ecef_M_enu_vec)
    {
      char buffer[80];
      sprintf(buffer, std::string(folder + "ecef_M_cv_%04d.txt").c_str(), i++);
      std::string filename = buffer;
      std::ofstream file(filename);
      if (file.is_open())
      {
        (ecef_M_enu * enu_M_cv).save(file);
      }
    }
#endif
    }
  }
}

TEST_CASE("Equidistributed sphere point", "[math_equi_sphere_pts]")
{
  const unsigned int maxPoints = 200;
  std::vector<std::pair<double, double> > lonlatVec = vpMath::computeRegularPointsOnSphere(maxPoints);
  const double radius = 5;

  SECTION("NED")
  {
    std::vector<vpHomogeneousMatrix> ecef_M_ned_vec =
      vpMath::getLocalTangentPlaneTransformations(lonlatVec, radius, vpMath::ned2ecef);
    CHECK(!ecef_M_ned_vec.empty());
    for (const auto &ecef_M_ned : ecef_M_ned_vec) {
#ifdef VERBOSE
      std::cout << "Equidistributed ecef_M_ned:\n" << ecef_M_ned << std::endl;
#endif
      CHECK(ecef_M_ned.isValid());
      CHECK(ecef_M_ned.getRotationMatrix().isARotationMatrix());
      CHECK(vpMath::equal(ecef_M_ned.getTranslationVector().sumSquare(), radius*radius));
    }

#ifdef DEBUG
    vpHomogeneousMatrix ned_M_cv;
    ned_M_cv[0][0] = 0;
    ned_M_cv[0][1] = -1;
    ned_M_cv[1][0] = 1;
    ned_M_cv[1][1] = 0;
    const std::string folder = "NED/equi/";
    vpIoTools::makeDirectory(folder);
    int i = 0;
    for (const auto &ecef_M_ned : ecef_M_ned_vec)
    {
      char buffer[80];
      sprintf(buffer, std::string(folder + "ecef_M_cv_%04d.txt").c_str(), i++);
      std::string filename = buffer;
      std::ofstream file(filename);
      if (file.is_open())
      {
        (ecef_M_ned * ned_M_cv).save(file);
      }
    }
#endif
  }

  SECTION("ENU")
  {
    std::vector<vpHomogeneousMatrix> ecef_M_enu_vec =
      vpMath::getLocalTangentPlaneTransformations(lonlatVec, radius, vpMath::enu2ecef);
    CHECK(!ecef_M_enu_vec.empty());
    for (const auto &ecef_M_enu : ecef_M_enu_vec) {
#ifdef VERBOSE
      std::cout << "Equidistributed ecef_M_enu:\n" << ecef_M_enu << std::endl;
#endif
      CHECK(ecef_M_enu.isValid());
      CHECK(ecef_M_enu.getRotationMatrix().isARotationMatrix());
      CHECK(vpMath::equal(ecef_M_enu.getTranslationVector().sumSquare(), radius*radius));
    }

#ifdef DEBUG
    vpHomogeneousMatrix enu_M_cv;
    enu_M_cv[1][1] = -1;
    enu_M_cv[2][2] = -1;
    const std::string folder = "ENU/equi/";
    vpIoTools::makeDirectory(folder);
    int i = 0;
    for (const auto &ecef_M_enu : ecef_M_enu_vec)
    {
      char buffer[80];
      sprintf(buffer, std::string(folder + "ecef_M_cv_%04d.txt").c_str(), i++);
      std::string filename = buffer;
      std::ofstream file(filename);
      if (file.is_open())
      {
        (ecef_M_enu * enu_M_cv).save(file);
      }
    }
#endif
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
