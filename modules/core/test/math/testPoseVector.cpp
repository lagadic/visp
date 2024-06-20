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
 * Test some vpColVector functionalities.
 */

/*!
  \example testPoseVector.cpp

  Test some vpPoseVector functionalities.
*/
#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_CATCH2
#include <limits>
#include <vector>

#include <visp3/core/vpPoseVector.h>

#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
void checkSize(const vpPoseVector &pose, const std::vector<double> &ref)
{
  REQUIRE(pose.size() == 6);
  REQUIRE(pose.getRows() == 6);
  REQUIRE(pose.getCols() == 1);
  REQUIRE(pose.size() == ref.size());
}

void checkData(const vpPoseVector &pose, const std::vector<double> &ref)
{
  for (unsigned int i = 0; i < pose.size(); i++) {
    REQUIRE(pose[i] == Approx(ref[i]).epsilon(std::numeric_limits<double>::epsilon()));
  }
}
} // namespace

TEST_CASE("vpPoseVector size", "[vpColVector]")
{
  vpPoseVector pose;
  REQUIRE(pose.size() == 6);
  REQUIRE(pose.getRows() == 6);
  REQUIRE(pose.getCols() == 1);

  for (unsigned int i = 0; i < pose.getRows(); i++) {
    REQUIRE(pose[i] == Approx(0).epsilon(std::numeric_limits<double>::epsilon()));
  }
}

TEST_CASE("vpPoseVector value assignment", "[vpColVector]")
{
  vpPoseVector pose;
  std::vector<double> ref(6);
  pose[0] = ref[0] = 0.1;
  pose[1] = ref[1] = 0.2;
  pose[2] = ref[2] = 0.3;
  pose[3] = ref[3] = vpMath::rad(10);
  pose[4] = ref[4] = vpMath::rad(20);
  pose[5] = ref[5] = vpMath::rad(30);

  checkSize(pose, ref);
  checkData(pose, ref);
}

TEST_CASE("vpPoseVector constructor", "[vpColVector]")
{
  std::vector<double> ref(6);
  ref[0] = 0.1;
  ref[1] = 0.2;
  ref[2] = 0.3;
  ref[3] = vpMath::rad(10);
  ref[4] = vpMath::rad(20);
  ref[5] = vpMath::rad(30);

  vpPoseVector pose(ref[0], ref[1], ref[2], ref[3], ref[4], ref[5]);

  checkSize(pose, ref);
  checkData(pose, ref);
}

TEST_CASE("vpPoseVector copy constructor", "[vpColVector]")
{
  std::vector<double> ref = { 0.1, 0.2, 0.3, vpMath::rad(10), vpMath::rad(20), vpMath::rad(30) };

  vpPoseVector pose1(ref[0], ref[1], ref[2], ref[3], ref[4], ref[5]);
  vpPoseVector pose2(pose1);

  checkSize(pose2, ref);
  checkData(pose2, ref);
}

TEST_CASE("vpPoseVector object assignment", "[vpColVector]")
{
  std::vector<double> ref = { 0.1, 0.2, 0.3, vpMath::rad(10), vpMath::rad(20), vpMath::rad(30) };

  vpPoseVector pose1(ref[0], ref[1], ref[2], ref[3], ref[4], ref[5]);
  vpPoseVector pose2 = pose1;

  checkSize(pose2, ref);
  checkData(pose2, ref);
}

TEST_CASE("vpPoseVector set", "[vpColVector]")
{
  std::vector<double> ref = { 0.1, 0.2, 0.3, vpMath::rad(10), vpMath::rad(20), vpMath::rad(30) };

  vpPoseVector pose1(ref[0], ref[1], ref[2], ref[3], ref[4], ref[5]);
  vpPoseVector pose2;
  pose2.set(pose1[0], pose1[1], pose1[2], pose1[3], pose1[4], pose1[5]);

  checkSize(pose2, ref);
  checkData(pose2, ref);
}

TEST_CASE("vpPoseVector constructor t, tu", "[vpColVector]")
{
  std::vector<double> ref = { 0.1, 0.2, 0.3, vpMath::rad(10), vpMath::rad(20), vpMath::rad(30) };
  vpTranslationVector t(ref[0], ref[1], ref[2]);
  vpThetaUVector tu(ref[3], ref[4], ref[5]);

  vpPoseVector pose(t, tu);

  checkSize(pose, ref);
  checkData(pose, ref);
}

TEST_CASE("vpPoseVector build t, tu", "[vpColVector]")
{
  std::vector<double> ref = { 0.1, 0.2, 0.3, vpMath::rad(10), vpMath::rad(20), vpMath::rad(30) };
  vpTranslationVector t(ref[0], ref[1], ref[2]);
  vpThetaUVector tu(ref[3], ref[4], ref[5]);

  vpPoseVector pose;
  pose.build(t, tu);

  checkSize(pose, ref);
  checkData(pose, ref);
}

TEST_CASE("vpPoseVector constructor vpHomogeneousMatrix", "[vpColVector]")
{
  std::vector<double> ref = { 0.1, 0.2, 0.3, vpMath::rad(10), vpMath::rad(20), vpMath::rad(30) };
  vpTranslationVector t(ref[0], ref[1], ref[2]);
  vpThetaUVector tu(ref[3], ref[4], ref[5]);
  vpHomogeneousMatrix M(t, tu);

  vpPoseVector pose(M);

  checkSize(pose, ref);
  checkData(pose, ref);
}

TEST_CASE("vpPoseVector build vpHomogeneousMatrix", "[vpColVector]")
{
  std::vector<double> ref = { 0.1, 0.2, 0.3, vpMath::rad(10), vpMath::rad(20), vpMath::rad(30) };
  vpTranslationVector t(ref[0], ref[1], ref[2]);
  vpThetaUVector tu(ref[3], ref[4], ref[5]);
  vpHomogeneousMatrix M(t, tu);

  vpPoseVector pose;
  pose.build(M);

  checkSize(pose, ref);
  checkData(pose, ref);
}

TEST_CASE("vpPoseVector constructor t, R", "[vpColVector]")
{
  std::vector<double> ref = { 0.1, 0.2, 0.3, vpMath::rad(10), vpMath::rad(20), vpMath::rad(30) };
  vpTranslationVector t(ref[0], ref[1], ref[2]);
  vpThetaUVector tu(ref[3], ref[4], ref[5]);
  vpRotationMatrix R(tu);

  vpPoseVector pose(t, R);

  checkSize(pose, ref);
  checkData(pose, ref);
}

TEST_CASE("vpPoseVector build t, R", "[vpColVector]")
{
  std::vector<double> ref = { 0.1, 0.2, 0.3, vpMath::rad(10), vpMath::rad(20), vpMath::rad(30) };
  vpTranslationVector t(ref[0], ref[1], ref[2]);
  vpThetaUVector tu(ref[3], ref[4], ref[5]);
  vpRotationMatrix R(tu);

  vpPoseVector pose;
  pose.build(t, R);

  checkSize(pose, ref);
  checkData(pose, ref);
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
