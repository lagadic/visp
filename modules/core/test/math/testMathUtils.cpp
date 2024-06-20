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
 * Test additional math functions such as lon-lat generator or look-at function.
 */

/*!
  \example testMathUtils.cpp

  Test additional math functions such as lon-lat generator or look-at function.
*/
#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_CATCH2

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>

#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

// #define VERBOSE
// #define DEBUG

#ifdef DEBUG
#include <visp3/core/vpIoTools.h>
#endif

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

TEST_CASE("Lon-Lat generator", "[math_lonlat]")
{
  const int lonStart = 0, lonEnd = 360, nlon = 20;
  const int latStart = 0, latEnd = 90, nLat = 10;
  std::vector<double> longitudes = vpMath::linspace(lonStart, lonEnd, nlon);
  std::vector<double> latitudes = vpMath::linspace(latStart, latEnd, nLat);
  const double radius = 5;

  std::vector<std::pair<double, double> > lonlatVec;
  lonlatVec.reserve(longitudes.size() * latitudes.size());
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
      CHECK(vpMath::equal(ecef_M_ned.getTranslationVector().sumSquare(), radius * radius));
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
    for (const auto &ecef_M_ned : ecef_M_ned_vec) {
      std::stringstream buffer;
      buffer << folder << "ecef_M_cv_" << std::setw(4) << std::setfill('0') << i++ << ".txt";
      std::string filename = buffer.str();
      std::ofstream file(filename);
      if (file.is_open()) {
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
      CHECK(vpMath::equal(ecef_M_enu.getTranslationVector().sumSquare(), radius * radius));

#ifdef DEBUG
      vpHomogeneousMatrix enu_M_cv;
      enu_M_cv[1][1] = -1;
      enu_M_cv[2][2] = -1;
      const std::string folder = "ENU/lon-lat/";
      vpIoTools::makeDirectory(folder);
      int i = 0;
      for (const auto &ecef_M_enu : ecef_M_enu_vec) {
        std::stringstream buffer;
        buffer << folder << "ecef_M_cv_" << std::setw(4) << std::setfill('0') << i++ << ".txt";
        std::string filename = buffer.str();
        std::ofstream file(filename);
        if (file.is_open()) {
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
      CHECK(vpMath::equal(ecef_M_ned.getTranslationVector().sumSquare(), radius * radius));
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
    for (const auto &ecef_M_ned : ecef_M_ned_vec) {
      std::stringstream buffer;
      buffer << folder << "ecef_M_cv_" << std::setw(4) << std::setfill('0') << i++ << ".txt";
      std::string filename = buffer.str();
      std::ofstream file(filename);
      if (file.is_open()) {
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
      CHECK(vpMath::equal(ecef_M_enu.getTranslationVector().sumSquare(), radius * radius));
    }

#ifdef DEBUG
    vpHomogeneousMatrix enu_M_cv;
    enu_M_cv[1][1] = -1;
    enu_M_cv[2][2] = -1;
    const std::string folder = "ENU/equi/";
    vpIoTools::makeDirectory(folder);
    int i = 0;
    for (const auto &ecef_M_enu : ecef_M_enu_vec) {
      std::stringstream buffer;
      buffer << folder << "ecef_M_cv_" << std::setw(4) << std::setfill('0') << i++ << ".txt";
      std::string filename = buffer.str();
      std::ofstream file(filename);
      if (file.is_open()) {
        (ecef_M_enu * enu_M_cv).save(file);
      }
    }
#endif
  }
}

TEST_CASE("Look-at", "[math_look_at]")
{
  // Set camera to an arbitrary pose (only translation)
  vpColVector from_blender = { 8.867762565612793, -1.1965436935424805, 2.1211400032043457 };
  // Transformation from OpenGL to Blender frame
  vpHomogeneousMatrix blender_M_gl;
  blender_M_gl[0][0] = 0;
  blender_M_gl[0][2] = 1;
  blender_M_gl[1][0] = 1;
  blender_M_gl[1][1] = 0;
  blender_M_gl[2][1] = 1;
  blender_M_gl[2][2] = 0;

  // From is the current camera pose expressed in the OpenGL coordinate system
  vpColVector from = (blender_M_gl.getRotationMatrix().t() * from_blender);
  // To is the desired point toward the camera must look
  vpColVector to = { 0, 0, 0 };
  // Up is an arbitrary vector
  vpColVector up = { 0, 1, 0 };

  // Compute the look-at transformation
  vpHomogeneousMatrix gl_M_cam = vpMath::lookAt(from, to, up);
  std::cout << "\ngl_M_cam:\n" << gl_M_cam << std::endl;

  // Transformation from the computer vision frame to the Blender camera frame
  vpHomogeneousMatrix cam_M_cv;
  cam_M_cv[1][1] = -1;
  cam_M_cv[2][2] = -1;
  // Transformation from the computer vision frame to the Blender frame
  vpHomogeneousMatrix bl_M_cv = blender_M_gl * gl_M_cam * cam_M_cv;
  std::cout << "\nbl_M_cv:\n" << bl_M_cv << std::endl;

  // Ground truth using Blender look-at
  vpHomogeneousMatrix bl_M_cv_gt = vpHomogeneousMatrix({ 0.13372008502483368, 0.22858507931232452,  -0.9642965197563171,
                                    8.867762565612793,   0.9910191297531128,   -0.030843468382954597,
                                    0.13011434674263,    -1.1965436935424805,  -5.4016709327697754e-08,
                                    -0.9730352163314819, -0.23065657913684845, 2.121140241622925 });
  std::cout << "\nbl_M_cv_gt:\n" << bl_M_cv_gt << std::endl;

  const double tolerance = 1e-6;
  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 4; j++) {
      CHECK(vpMath::equal(bl_M_cv[i][j], bl_M_cv_gt[i][j], tolerance));
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

int main() { return EXIT_SUCCESS; }
#endif
