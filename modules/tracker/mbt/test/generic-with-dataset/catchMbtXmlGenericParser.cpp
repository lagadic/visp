/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2026 by Inria. All rights reserved.
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
 * Test vpMbtXmlGenericParser parse / save.
 */

/*!
  \example catchMbtXmlGenericParser.cpp

  Test vpMbtXmlGenericParser parse / save.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2) && defined(VISP_HAVE_PUGIXML) && \
    (defined(VISP_HAVE_LAPACK) || defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_OPENCV))

#if defined(VISP_BUILD_CATCH2)
#include <catch_amalgamated.hpp>
#else // Since v3.1.1
#include <catch2/catch_all.hpp>
#endif

#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/mbt/vpMbtXmlGenericParser.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

static const std::string g_visp_images_dir = vpIoTools::getViSPImagesDataPath();

TEST_CASE("vpMbtXmlGenericParser - Edge and KLT parsing (chateau.xml)", "[MbtXmlGenericParser]")
{
  const std::string filename = g_visp_images_dir + "/xml/chateau.xml";
  if (!vpIoTools::checkDirectory(g_visp_images_dir + "/xml")) {
    std::cout << "Skipping test: " << g_visp_images_dir << "/xml not found" << std::endl;
    return;
  }

  INFO("Parsing: " << filename);
  vpMbtXmlGenericParser xml(vpMbtXmlGenericParser::EDGE_PARSER | vpMbtXmlGenericParser::KLT_PARSER);
  REQUIRE_NOTHROW(xml.parse(filename));

  const double eps = std::numeric_limits<double>::epsilon();

  SECTION("Moving edges parameters")
  {
    vpMe me_ref;
    me_ref.setMaskSize(5);
    me_ref.setMaskNumber(180);
    me_ref.setRange(8);
    me_ref.setMu1(0.5);
    me_ref.setMu2(0.5);
    me_ref.setSampleStep(5);

    vpMe me;
    xml.getEdgeMe(me);

    // visp-images/xml/chateau.xml may use either normalized or old threshold
    // depending on the dataset version — both are valid
    if (me.getLikelihoodThresholdType() == vpMe::NORMALIZED_THRESHOLD) {
      me_ref.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
      me_ref.setThreshold(5);
    }
    else {
      me_ref.setLikelihoodThresholdType(vpMe::OLD_THRESHOLD);
      me_ref.setThreshold(10000);
    }

    CHECK(me.getMaskSize() == me_ref.getMaskSize());
    CHECK(me.getMaskNumber() == me_ref.getMaskNumber());
    CHECK(me.getRange() == me_ref.getRange());
    CHECK(vpMath::equal(me.getThreshold(), me_ref.getThreshold(), eps));
    CHECK(vpMath::equal(me.getMu1(), me_ref.getMu1(), eps));
    CHECK(vpMath::equal(me.getMu2(), me_ref.getMu2(), eps));
    CHECK(vpMath::equal(me.getSampleStep(), me_ref.getSampleStep(), eps));
  }

  SECTION("KLT parameters")
  {
    CHECK(xml.getKltMaskBorder() == 5);
    CHECK(xml.getKltMaxFeatures() == 10000);
    CHECK(xml.getKltWindowSize() == 5);
    CHECK(vpMath::equal(xml.getKltQuality(), 0.01, eps));
    CHECK(vpMath::equal(xml.getKltMinDistance(), 5.0, eps));
    CHECK(vpMath::equal(xml.getKltHarrisParam(), 0.02, eps));
    CHECK(xml.getKltBlockSize() == 3);
    CHECK(xml.getKltPyramidLevels() == 3);
  }

  SECTION("Camera parameters")
  {
    vpCameraParameters cam_ref;
    cam_ref.initPersProjWithoutDistortion(615.1674804688, 615.1675415039, 312.1889953613, 243.4373779297);
    vpCameraParameters cam;
    xml.getCameraParameters(cam);
    CHECK(cam == cam_ref);
  }

  SECTION("Visibility parameters")
  {
    CHECK(vpMath::equal(xml.getAngleAppear(), 70.0, eps));
    CHECK(vpMath::equal(xml.getAngleDisappear(), 80.0, eps));
    CHECK(vpMath::equal(xml.getNearClippingDistance(), 0.01, eps));
    CHECK(vpMath::equal(xml.getFarClippingDistance(), 2.0, eps));
    CHECK(xml.getFovClipping());
  }
}

TEST_CASE("vpMbtXmlGenericParser - Projection error parsing (chateau.xml)", "[MbtXmlGenericParser]")
{
  if (!vpIoTools::checkDirectory(g_visp_images_dir + "/xml")) {
    std::cout << "Skipping test: " << g_visp_images_dir << "/xml not found" << std::endl;
    return;
  }

  const std::string filename = g_visp_images_dir + "/xml/chateau.xml";
  INFO("Parsing: " << filename);
  vpMbtXmlGenericParser xml(vpMbtXmlGenericParser::PROJECTION_ERROR_PARSER);
  REQUIRE_NOTHROW(xml.parse(filename));

  const double eps = std::numeric_limits<double>::epsilon();

  SECTION("Projection error parameters")
  {
    vpMe me_proj;
    xml.getProjectionErrorMe(me_proj);
    CHECK(vpMath::equal(me_proj.getSampleStep(), 12.0, eps));
    CHECK(xml.getProjectionErrorKernelSize() == 3);
  }
}

TEST_CASE("vpMbtXmlGenericParser - Depth Normal and Dense parsing (chateau_depth.xml)", "[MbtXmlGenericParser]")
{
  if (!vpIoTools::checkDirectory(g_visp_images_dir + "/xml")) {
    std::cout << "Skipping test: " << g_visp_images_dir << "/xml not found" << std::endl;
    return;
  }

  const std::string filename = g_visp_images_dir + "/xml/chateau_depth.xml";
  INFO("Parsing: " << filename);
  vpMbtXmlGenericParser xml(vpMbtXmlGenericParser::DEPTH_NORMAL_PARSER | vpMbtXmlGenericParser::DEPTH_DENSE_PARSER);
  REQUIRE_NOTHROW(xml.parse(filename));

  const double eps = std::numeric_limits<double>::epsilon();

  SECTION("Depth normal parameters")
  {
    CHECK(xml.getDepthNormalFeatureEstimationMethod() == 0);
    CHECK(xml.getDepthNormalPclPlaneEstimationMethod() == 2);
    CHECK(xml.getDepthNormalPclPlaneEstimationRansacMaxIter() == 200);
    CHECK(vpMath::equal(xml.getDepthNormalPclPlaneEstimationRansacThreshold(), 0.001, eps));
    CHECK(xml.getDepthNormalSamplingStepX() == 2);
    CHECK(xml.getDepthNormalSamplingStepY() == 2);
  }

  SECTION("Depth dense parameters")
  {
    CHECK(xml.getDepthDenseSamplingStepX() == 4);
    CHECK(xml.getDepthDenseSamplingStepY() == 4);
  }

  SECTION("Camera parameters")
  {
    vpCameraParameters cam_ref;
    cam_ref.initPersProjWithoutDistortion(476.0536193848, 476.0534973145, 311.4845581055, 246.2832336426);
    vpCameraParameters cam;
    xml.getCameraParameters(cam);
    CHECK(cam == cam_ref);
  }

  SECTION("Visibility parameters")
  {
    CHECK(vpMath::equal(xml.getAngleAppear(), 70.0, eps));
    CHECK(vpMath::equal(xml.getAngleDisappear(), 80.0, eps));
    CHECK(vpMath::equal(xml.getNearClippingDistance(), 0.01, eps));
    CHECK(vpMath::equal(xml.getFarClippingDistance(), 2.0, eps));
    CHECK(xml.getFovClipping());
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
#include <iostream>

int main()
{
#if !(defined(VISP_HAVE_PUGIXML))
  std::cout << "Cannot run this example: enable pugixml built-in" << std::endl;
#elif !(defined(VISP_HAVE_LAPACK) || defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_OPENCV))
  std::cout << "Cannot run this example: install Lapack, Eigen3 or OpenCV" << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
