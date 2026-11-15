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

#include <future>
#include <thread>

#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/mbt/vpMbtFaceDepthNormal.h>
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

// ---------------------------------------------------------------------------
// Setter / getter round-trip: ensure every set* is reflected by its get*
// without any XML file involved.
// ---------------------------------------------------------------------------
TEST_CASE("vpMbtXmlGenericParser - Setter/getter round-trip (no XML)", "[MbtXmlGenericParser]")
{
  const double eps = std::numeric_limits<double>::epsilon();

  SECTION("Edge parser setters")
  {
    vpMbtXmlGenericParser xml(vpMbtXmlGenericParser::EDGE_PARSER);

    vpMe me;
    me.setMaskSize(7);
    me.setMaskNumber(360);
    me.setRange(10);
    me.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
    me.setThreshold(20.0);
    me.setMu1(0.3);
    me.setMu2(0.7);
    me.setSampleStep(6);
    xml.setEdgeMe(me);

    vpMe me_out;
    xml.getEdgeMe(me_out);
    CHECK(me_out.getMaskSize() == 7u);
    CHECK(me_out.getMaskNumber() == 360u);
    CHECK(me_out.getRange() == 10u);
    CHECK(vpMath::equal(me_out.getThreshold(), 20.0, eps));
    CHECK(vpMath::equal(me_out.getMu1(), 0.3, eps));
    CHECK(vpMath::equal(me_out.getMu2(), 0.7, eps));
    CHECK(vpMath::equal(me_out.getSampleStep(), 6.0, eps));
  }

  SECTION("KLT parser setters")
  {
    vpMbtXmlGenericParser xml(vpMbtXmlGenericParser::KLT_PARSER);
    xml.setKltMaskBorder(8);
    xml.setKltMaxFeatures(500);
    xml.setKltWindowSize(7);
    xml.setKltQuality(0.05);
    xml.setKltMinDistance(10.0);
    xml.setKltHarrisParam(0.03);
    xml.setKltBlockSize(5);
    xml.setKltPyramidLevels(4);

    CHECK(xml.getKltMaskBorder() == 8u);
    CHECK(xml.getKltMaxFeatures() == 500u);
    CHECK(xml.getKltWindowSize() == 7u);
    CHECK(vpMath::equal(xml.getKltQuality(), 0.05, eps));
    CHECK(vpMath::equal(xml.getKltMinDistance(), 10.0, eps));
    CHECK(vpMath::equal(xml.getKltHarrisParam(), 0.03, eps));
    CHECK(xml.getKltBlockSize() == 5u);
    CHECK(xml.getKltPyramidLevels() == 4u);
  }

  SECTION("Camera, angle and clipping setters")
  {
    vpMbtXmlGenericParser xml(vpMbtXmlGenericParser::EDGE_PARSER);

    vpCameraParameters cam_in;
    cam_in.initPersProjWithoutDistortion(600.0, 601.0, 320.0, 240.0);
    xml.setCameraParameters(cam_in);
    vpCameraParameters cam_out;
    xml.getCameraParameters(cam_out);
    CHECK(cam_out == cam_in);

    xml.setAngleAppear(50.0);
    xml.setAngleDisappear(60.0);
    CHECK(vpMath::equal(xml.getAngleAppear(), 50.0, eps));
    CHECK(vpMath::equal(xml.getAngleDisappear(), 60.0, eps));

    xml.setNearClippingDistance(0.05);
    xml.setFarClippingDistance(1.5);
    CHECK(vpMath::equal(xml.getNearClippingDistance(), 0.05, eps));
    CHECK(vpMath::equal(xml.getFarClippingDistance(), 1.5, eps));
  }

  SECTION("Depth normal setters")
  {
    vpMbtXmlGenericParser xml(vpMbtXmlGenericParser::DEPTH_NORMAL_PARSER);
    xml.setDepthNormalFeatureEstimationMethod(vpMbtFaceDepthNormal::ROBUST_FEATURE_ESTIMATION);
    xml.setDepthNormalPclPlaneEstimationMethod(1);
    xml.setDepthNormalPclPlaneEstimationRansacMaxIter(100);
    xml.setDepthNormalPclPlaneEstimationRansacThreshold(0.005);
    xml.setDepthNormalSamplingStepX(4);
    xml.setDepthNormalSamplingStepY(4);

    CHECK(xml.getDepthNormalFeatureEstimationMethod() == vpMbtFaceDepthNormal::ROBUST_FEATURE_ESTIMATION);
    CHECK(xml.getDepthNormalPclPlaneEstimationMethod() == 1);
    CHECK(xml.getDepthNormalPclPlaneEstimationRansacMaxIter() == 100);
    CHECK(vpMath::equal(xml.getDepthNormalPclPlaneEstimationRansacThreshold(), 0.005, eps));
    CHECK(xml.getDepthNormalSamplingStepX() == 4u);
    CHECK(xml.getDepthNormalSamplingStepY() == 4u);
  }

  SECTION("Depth dense setters")
  {
    vpMbtXmlGenericParser xml(vpMbtXmlGenericParser::DEPTH_DENSE_PARSER);
    xml.setDepthDenseSamplingStepX(8);
    xml.setDepthDenseSamplingStepY(6);
    CHECK(xml.getDepthDenseSamplingStepX() == 8u);
    CHECK(xml.getDepthDenseSamplingStepY() == 6u);
  }

  SECTION("Projection error setters")
  {
    vpMbtXmlGenericParser xml(vpMbtXmlGenericParser::PROJECTION_ERROR_PARSER);

    vpMe me;
    me.setSampleStep(8.0);
    xml.setProjectionErrorMe(me);
    xml.setProjectionErrorKernelSize(4);

    vpMe me_out;
    xml.getProjectionErrorMe(me_out);
    CHECK(vpMath::equal(me_out.getSampleStep(), 8.0, eps));
    CHECK(xml.getProjectionErrorKernelSize() == 4u);
  }
}

// ---------------------------------------------------------------------------
// LOD getters: exercise getLodState / getLodMinLine / getLodMinPolygon
// which are never reached by the XML-based tests above.
// ---------------------------------------------------------------------------
TEST_CASE("vpMbtXmlGenericParser - LOD getters default values", "[MbtXmlGenericParser]")
{
  vpMbtXmlGenericParser xml(vpMbtXmlGenericParser::EDGE_PARSER);
  // Default values set in Impl constructor
  CHECK(xml.getLodState() == false);
  CHECK(xml.getLodMinLineLengthThreshold() > 0.0);
  CHECK(xml.getLodMinPolygonAreaThreshold() > 0.0);
}

// ---------------------------------------------------------------------------
// hasFarClippingDistance / hasNearClippingDistance:
// Only set to true after read_face() parses the corresponding XML nodes.
// Without parsing, defaults are false.
// ---------------------------------------------------------------------------
TEST_CASE("vpMbtXmlGenericParser - Clipping distance flags default to false", "[MbtXmlGenericParser]")
{
  vpMbtXmlGenericParser xml(vpMbtXmlGenericParser::EDGE_PARSER);
  CHECK(xml.hasFarClippingDistance() == false);
  CHECK(xml.hasNearClippingDistance() == false);
}

TEST_CASE("vpMbtXmlGenericParser - Clipping distance flags set after XML parsing (chateau.xml)", "[MbtXmlGenericParser]")
{
  if (!vpIoTools::checkDirectory(g_visp_images_dir + "/xml")) {
    std::cout << "Skipping test: " << g_visp_images_dir << "/xml not found" << std::endl;
    return;
  }
  const std::string filename = g_visp_images_dir + "/xml/chateau.xml";
  vpMbtXmlGenericParser xml(vpMbtXmlGenericParser::EDGE_PARSER);
  xml.parse(filename);
  // chateau.xml defines near and far clipping — flags must be true
  CHECK(xml.hasNearClippingDistance() == true);
  CHECK(xml.hasFarClippingDistance() == true);
}

// ---------------------------------------------------------------------------
// setVerbose(false): exercises the verbose=false branches inside all
// read_* methods (lines currently missed because m_verbose defaults to true).
// ---------------------------------------------------------------------------
TEST_CASE("vpMbtXmlGenericParser - Parse with verbose disabled", "[MbtXmlGenericParser]")
{
  if (!vpIoTools::checkDirectory(g_visp_images_dir + "/xml")) {
    std::cout << "Skipping test: " << g_visp_images_dir << "/xml not found" << std::endl;
    return;
  }

  const double eps = std::numeric_limits<double>::epsilon();

  SECTION("Edge + KLT parser, verbose off (chateau.xml)")
  {
    const std::string filename = g_visp_images_dir + "/xml/chateau.xml";
    vpMbtXmlGenericParser xml(vpMbtXmlGenericParser::EDGE_PARSER | vpMbtXmlGenericParser::KLT_PARSER);
    xml.setVerbose(false);
    REQUIRE_NOTHROW(xml.parse(filename));
    // Spot-check a value to confirm parsing still worked
    CHECK(vpMath::equal(xml.getAngleAppear(), 70.0, eps));
    CHECK(xml.getKltMaskBorder() == 5u);
  }

  SECTION("Depth parser, verbose off (chateau_depth.xml)")
  {
    const std::string filename = g_visp_images_dir + "/xml/chateau_depth.xml";
    vpMbtXmlGenericParser xml(vpMbtXmlGenericParser::DEPTH_NORMAL_PARSER | vpMbtXmlGenericParser::DEPTH_DENSE_PARSER);
    xml.setVerbose(false);
    REQUIRE_NOTHROW(xml.parse(filename));
    CHECK(xml.getDepthDenseSamplingStepX() == 4u);
  }

  SECTION("Projection error parser, verbose off (chateau.xml)")
  {
    const std::string filename = g_visp_images_dir + "/xml/chateau.xml";
    vpMbtXmlGenericParser xml(vpMbtXmlGenericParser::PROJECTION_ERROR_PARSER);
    xml.setVerbose(false);
    REQUIRE_NOTHROW(xml.parse(filename));
    CHECK(xml.getProjectionErrorKernelSize() == 3u);
  }
}

// ---------------------------------------------------------------------------
// Single-feature parser types: cover branches where only one feature type
// flag is set (EDGE only, KLT only, DEPTH_NORMAL only, DEPTH_DENSE only).
// ---------------------------------------------------------------------------
TEST_CASE("vpMbtXmlGenericParser - Single parser type (chateau.xml)", "[MbtXmlGenericParser]")
{
  if (!vpIoTools::checkDirectory(g_visp_images_dir + "/xml")) {
    std::cout << "Skipping test: " << g_visp_images_dir << "/xml not found" << std::endl;
    return;
  }

  const std::string filename = g_visp_images_dir + "/xml/chateau.xml";

  SECTION("EDGE_PARSER only")
  {
    vpMbtXmlGenericParser xml(vpMbtXmlGenericParser::EDGE_PARSER);
    REQUIRE_NOTHROW(xml.parse(filename));
    vpMe me;
    xml.getEdgeMe(me);
    CHECK(me.getMaskSize() == 5u);
  }

  SECTION("KLT_PARSER only")
  {
    vpMbtXmlGenericParser xml(vpMbtXmlGenericParser::KLT_PARSER);
    REQUIRE_NOTHROW(xml.parse(filename));
    CHECK(xml.getKltMaskBorder() == 5u);
  }
}

TEST_CASE("vpMbtXmlGenericParser - Single parser type (chateau_depth.xml)", "[MbtXmlGenericParser]")
{
  if (!vpIoTools::checkDirectory(g_visp_images_dir + "/xml")) {
    std::cout << "Skipping test: " << g_visp_images_dir << "/xml not found" << std::endl;
    return;
  }

  const std::string filename = g_visp_images_dir + "/xml/chateau_depth.xml";

  SECTION("DEPTH_NORMAL_PARSER only")
  {
    vpMbtXmlGenericParser xml(vpMbtXmlGenericParser::DEPTH_NORMAL_PARSER);
    REQUIRE_NOTHROW(xml.parse(filename));
    CHECK(xml.getDepthNormalSamplingStepX() == 2u);
    CHECK(xml.getDepthNormalSamplingStepY() == 2u);
  }

  SECTION("DEPTH_DENSE_PARSER only")
  {
    vpMbtXmlGenericParser xml(vpMbtXmlGenericParser::DEPTH_DENSE_PARSER);
    REQUIRE_NOTHROW(xml.parse(filename));
    CHECK(xml.getDepthDenseSamplingStepX() == 4u);
    CHECK(xml.getDepthDenseSamplingStepY() == 4u);
  }
}

// ---------------------------------------------------------------------------
// Error path: parse() on a non-existent file must throw vpException.
// ---------------------------------------------------------------------------
TEST_CASE("vpMbtXmlGenericParser - Parse non-existent file throws", "[MbtXmlGenericParser]")
{
  vpMbtXmlGenericParser xml(vpMbtXmlGenericParser::EDGE_PARSER);
  CHECK_THROWS_AS(xml.parse("/non/existent/path/file.xml"), vpException);
}

// ---------------------------------------------------------------------------
// Concurrency: two parsers constructed and used simultaneously must not
// produce a data race (exercises the std::call_once fix on m_setlocale_flag).
// Each instance parses independently and must return consistent values.
// ---------------------------------------------------------------------------
#if defined(VISP_HAVE_THREADS)
TEST_CASE("vpMbtXmlGenericParser - Concurrent construction and parsing", "[MbtXmlGenericParser]")
{
  if (!vpIoTools::checkDirectory(g_visp_images_dir + "/xml")) {
    std::cout << "Skipping test: " << g_visp_images_dir << "/xml not found" << std::endl;
    return;
  }

  const std::string filename = g_visp_images_dir + "/xml/chateau.xml";
  const double eps = std::numeric_limits<double>::epsilon();

  auto parse_and_get_angle = [&filename]() {
    vpMbtXmlGenericParser xml(vpMbtXmlGenericParser::EDGE_PARSER);
    xml.setVerbose(false);
    xml.parse(filename);
    return xml.getAngleAppear();
    };

  std::future<double> f1 = std::async(std::launch::async, parse_and_get_angle);
  std::future<double> f2 = std::async(std::launch::async, parse_and_get_angle);

  double angle1 = f1.get();
  double angle2 = f2.get();

  // Both threads must read the same value from the same XML file
  CHECK(vpMath::equal(angle1, 70.0, eps));
  CHECK(vpMath::equal(angle2, 70.0, eps));
}
#endif

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
