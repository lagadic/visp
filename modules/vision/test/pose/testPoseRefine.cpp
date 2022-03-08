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
 * Test pose refinement (non-linear pose minimization).
 *
 *****************************************************************************/

/*!
  \example testPoseRefine.cpp

  \brief Test pose refinement (non-linear pose minimization).
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2)
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>
#include <visp3/vision/vpPose.h>
#include <visp3/vision/vpPoseException.h>

static const double L = 0.035;
static const double L2 = 0.1;

static const double tvec_thresh = 1e-6;
static const double tu_thresh = 1e-6;
static const double tvec_thresh_noise = 1e-3;
static const double tu_thresh_noise = 2e-3;

TEST_CASE("Pose refinement on 4 non co-planar points without noise", "[pose_refine]") {
  const int npt = 4;
  std::vector<vpPoint> P(npt); //  Point to be tracked
  P[0].setWorldCoordinates(-L2, -L2,  0);
  P[1].setWorldCoordinates( L2, -L2,  0.2);
  P[2].setWorldCoordinates( L2,  L2, -0.1);
  P[3].setWorldCoordinates(-L2,  L2,  0);

  vpTranslationVector tvec_true(0.1, 0.2, 0.3);
  vpThetaUVector tu_true(0.1, -0.2, 0.67);

  for (auto& pt : P) {
    pt.project(vpHomogeneousMatrix(tvec_true, tu_true));
  }

  vpTranslationVector tvec_init(0.15, 0.25, 0.35);
  vpThetaUVector tu_init(0.05, -0.25, 0.7);

  vpPose pose;
  pose.addPoints(P);
  vpHomogeneousMatrix cMo_est(tvec_init, tu_init);

  SECTION("VVS")
  {
    pose.computePose(vpPose::VIRTUAL_VS, cMo_est);

    vpTranslationVector tvec_refined(cMo_est);
    vpThetaUVector tu_refined(cMo_est);

    std::cout << "\nPose refinement on " << npt << " non co-planar points without noise using VVS method:" << std::endl;
    std::cout << "tvec_true: " << tvec_true.t() << std::endl;
    std::cout << "tvec_refined: " << tvec_refined.t() << std::endl;
    std::cout << "tvec error: " << std::sqrt((tvec_true - tvec_refined).sumSquare()) << std::endl;
    std::cout << "tu_true: " << tu_true.t() << std::endl;
    std::cout << "tu_refined: " << tu_refined.t() << std::endl;
    std::cout << "tu error: " << std::sqrt((vpColVector(tu_true) - vpColVector(tu_refined)).sumSquare()) << std::endl;

    CHECK((tvec_true - tvec_refined).sumSquare() < tvec_thresh);
    CHECK((vpColVector(tu_true) - vpColVector(tu_refined)).sumSquare() < tu_thresh);
  }

  SECTION("Lowe")
  {
    pose.computePose(vpPose::LOWE, cMo_est);

    vpTranslationVector tvec_refined(cMo_est);
    vpThetaUVector tu_refined(cMo_est);

    std::cout << "\nPose refinement on " << npt << " non co-planar points without noise using Lowe method:" << std::endl;
    std::cout << "tvec_true: " << tvec_true.t() << std::endl;
    std::cout << "tvec_refined: " << tvec_refined.t() << std::endl;
    std::cout << "tvec error: " << std::sqrt((tvec_true - tvec_refined).sumSquare()) << std::endl;
    std::cout << "tu_true: " << tu_true.t() << std::endl;
    std::cout << "tu_refined: " << tu_refined.t() << std::endl;
    std::cout << "tu error: " << std::sqrt((vpColVector(tu_true) - vpColVector(tu_refined)).sumSquare()) << std::endl;

    CHECK((tvec_true - tvec_refined).sumSquare() < tvec_thresh);
    CHECK((vpColVector(tu_true) - vpColVector(tu_refined)).sumSquare() < tu_thresh);
  }
}

TEST_CASE("Pose refinement on 4 non co-planar points with noise", "[pose_refine]") {
  const int npt = 4;
  std::vector<vpPoint> P(npt); //  Point to be tracked
  P[0].setWorldCoordinates(-L2, -L2,  0);
  P[1].setWorldCoordinates( L2, -L2,  0.2);
  P[2].setWorldCoordinates( L2,  L2, -0.1);
  P[3].setWorldCoordinates(-L2,  L2,  0);

  vpTranslationVector tvec_true(0.1, 0.2, 0.3);
  vpThetaUVector tu_true(0.1, -0.2, 0.67);

  vpUniRand rng(0x123456789);
  for (auto& pt : P) {
    pt.project(vpHomogeneousMatrix(tvec_true, tu_true));
    pt.set_x(pt.get_x() + rng.uniform(-1e-2, +1e-2));
    pt.set_y(pt.get_y() + rng.uniform(-1e-2, +1e-2));
  }

  vpTranslationVector tvec_init(0.15, 0.25, 0.35);
  vpThetaUVector tu_init(0.05, -0.25, 0.7);

  vpPose pose;
  pose.addPoints(P);
  vpHomogeneousMatrix cMo_est(tvec_init, tu_init);

  SECTION("VVS")
  {
    pose.computePose(vpPose::VIRTUAL_VS, cMo_est);

    vpTranslationVector tvec_refined(cMo_est);
    vpThetaUVector tu_refined(cMo_est);

    std::cout << "\nPose refinement on " << npt << " non co-planar points with noise using VVS method:" << std::endl;
    std::cout << "tvec_true: " << tvec_true.t() << std::endl;
    std::cout << "tvec_refined: " << tvec_refined.t() << std::endl;
    std::cout << "tvec error: " << std::sqrt((tvec_true - tvec_refined).sumSquare()) << std::endl;
    std::cout << "tu_true: " << tu_true.t() << std::endl;
    std::cout << "tu_refined: " << tu_refined.t() << std::endl;
    std::cout << "tu error: " << std::sqrt((vpColVector(tu_true) - vpColVector(tu_refined)).sumSquare()) << std::endl;

    CHECK((tvec_true - tvec_refined).sumSquare() < tvec_thresh_noise);
    CHECK((vpColVector(tu_true) - vpColVector(tu_refined)).sumSquare() < tu_thresh_noise);
  }

  SECTION("Lowe")
  {
    pose.computePose(vpPose::LOWE, cMo_est);

    vpTranslationVector tvec_refined(cMo_est);
    vpThetaUVector tu_refined(cMo_est);

    std::cout << "\nPose refinement on " << npt << " non co-planar points with noise using Lowe method:" << std::endl;
    std::cout << "tvec_true: " << tvec_true.t() << std::endl;
    std::cout << "tvec_refined: " << tvec_refined.t() << std::endl;
    std::cout << "tvec error: " << std::sqrt((tvec_true - tvec_refined).sumSquare()) << std::endl;
    std::cout << "tu_true: " << tu_true.t() << std::endl;
    std::cout << "tu_refined: " << tu_refined.t() << std::endl;
    std::cout << "tu error: " << std::sqrt((vpColVector(tu_true) - vpColVector(tu_refined)).sumSquare()) << std::endl;

    CHECK((tvec_true - tvec_refined).sumSquare() < tvec_thresh_noise);
    CHECK((vpColVector(tu_true) - vpColVector(tu_refined)).sumSquare() < tu_thresh_noise);
  }
}

TEST_CASE("Pose refinement on 4 co-planar points without noise", "[pose_refine]") {
  const int npt = 4;
  std::vector<vpPoint> P(npt); //  Point to be tracked
  double Z = 0.03;

  P[0].setWorldCoordinates(-L, -L, Z);
  P[1].setWorldCoordinates( L, -L, Z);
  P[2].setWorldCoordinates( L,  L, Z);
  P[3].setWorldCoordinates(-L,  L, Z);

  vpTranslationVector tvec_true(0.1, 0.2, 0.3);
  vpThetaUVector tu_true(0.1, -0.2, 0.67);

  for (auto& pt : P) {
    pt.project(vpHomogeneousMatrix(tvec_true, tu_true));
  }

  vpTranslationVector tvec_init(0.15, 0.25, 0.35);
  vpThetaUVector tu_init(0.05, -0.25, 0.7);

  vpPose pose;
  pose.addPoints(P);
  vpHomogeneousMatrix cMo_est(tvec_init, tu_init);

  SECTION("VVS")
  {
    pose.computePose(vpPose::VIRTUAL_VS, cMo_est);

    vpTranslationVector tvec_refined(cMo_est);
    vpThetaUVector tu_refined(cMo_est);

    std::cout << "\nPose refinement on " << npt << " co-planar points without noise using VVS method:" << std::endl;
    std::cout << "tvec_true: " << tvec_true.t() << std::endl;
    std::cout << "tvec_refined: " << tvec_refined.t() << std::endl;
    std::cout << "tvec error: " << std::sqrt((tvec_true - tvec_refined).sumSquare()) << std::endl;
    std::cout << "tu_true: " << tu_true.t() << std::endl;
    std::cout << "tu_refined: " << tu_refined.t() << std::endl;
    std::cout << "tu error: " << std::sqrt((vpColVector(tu_true) - vpColVector(tu_refined)).sumSquare()) << std::endl;

    CHECK((tvec_true - tvec_refined).sumSquare() < tvec_thresh);
    CHECK((vpColVector(tu_true) - vpColVector(tu_refined)).sumSquare() < tu_thresh);
  }

  SECTION("Lowe")
  {
    pose.computePose(vpPose::LOWE, cMo_est);

    vpTranslationVector tvec_refined(cMo_est);
    vpThetaUVector tu_refined(cMo_est);

    std::cout << "\nPose refinement on " << npt << " co-planar points without noise using Lowe method:" << std::endl;
    std::cout << "tvec_true: " << tvec_true.t() << std::endl;
    std::cout << "tvec_refined: " << tvec_refined.t() << std::endl;
    std::cout << "tvec error: " << std::sqrt((tvec_true - tvec_refined).sumSquare()) << std::endl;
    std::cout << "tu_true: " << tu_true.t() << std::endl;
    std::cout << "tu_refined: " << tu_refined.t() << std::endl;
    std::cout << "tu error: " << std::sqrt((vpColVector(tu_true) - vpColVector(tu_refined)).sumSquare()) << std::endl;

    CHECK((tvec_true - tvec_refined).sumSquare() < tvec_thresh);
    CHECK((vpColVector(tu_true) - vpColVector(tu_refined)).sumSquare() < tu_thresh);
  }
}

TEST_CASE("Pose refinement on 4 co-planar points with noise", "[pose_refine]") {
  const int npt = 4;
  std::vector<vpPoint> P(npt); //  Point to be tracked
  double Z = 0.03;

  P[0].setWorldCoordinates(-L, -L, Z);
  P[1].setWorldCoordinates( L, -L, Z);
  P[2].setWorldCoordinates( L,  L, Z);
  P[3].setWorldCoordinates(-L,  L, Z);

  vpTranslationVector tvec_true(0.1, 0.2, 0.3);
  vpThetaUVector tu_true(0.1, -0.2, 0.67);

  vpUniRand rng(0x123456789);
  for (auto& pt : P) {
    pt.project(vpHomogeneousMatrix(tvec_true, tu_true));
    pt.set_x(pt.get_x() + rng.uniform(-1e-2, +1e-2));
    pt.set_y(pt.get_y() + rng.uniform(-1e-2, +1e-2));
  }

  vpTranslationVector tvec_init(0.15, 0.25, 0.35);
  vpThetaUVector tu_init(0.05, -0.25, 0.7);

  vpPose pose;
  pose.addPoints(P);
  vpHomogeneousMatrix cMo_est(tvec_init, tu_init);

  SECTION("VVS")
  {
    pose.computePose(vpPose::VIRTUAL_VS, cMo_est);

    vpTranslationVector tvec_refined(cMo_est);
    vpThetaUVector tu_refined(cMo_est);

    std::cout << "\nPose refinement on " << npt << " co-planar points with noise using VVS method:" << std::endl;
    std::cout << "tvec_true: " << tvec_true.t() << std::endl;
    std::cout << "tvec_refined: " << tvec_refined.t() << std::endl;
    std::cout << "tvec error: " << std::sqrt((tvec_true - tvec_refined).sumSquare()) << std::endl;
    std::cout << "tu_true: " << tu_true.t() << std::endl;
    std::cout << "tu_refined: " << tu_refined.t() << std::endl;
    std::cout << "tu error: " << std::sqrt((vpColVector(tu_true) - vpColVector(tu_refined)).sumSquare()) << std::endl;

    CHECK((tvec_true - tvec_refined).sumSquare() < tvec_thresh_noise);
    CHECK((vpColVector(tu_true) - vpColVector(tu_refined)).sumSquare() < tu_thresh_noise);
  }

  SECTION("Lowe")
  {
    pose.computePose(vpPose::LOWE, cMo_est);

    vpTranslationVector tvec_refined(cMo_est);
    vpThetaUVector tu_refined(cMo_est);

    std::cout << "\nPose refinement on " << npt << " co-planar points with noise using Lowe method:" << std::endl;
    std::cout << "tvec_true: " << tvec_true.t() << std::endl;
    std::cout << "tvec_refined: " << tvec_refined.t() << std::endl;
    std::cout << "tvec error: " << std::sqrt((tvec_true - tvec_refined).sumSquare()) << std::endl;
    std::cout << "tu_true: " << tu_true.t() << std::endl;
    std::cout << "tu_refined: " << tu_refined.t() << std::endl;
    std::cout << "tu error: " << std::sqrt((vpColVector(tu_true) - vpColVector(tu_refined)).sumSquare()) << std::endl;

    CHECK((tvec_true - tvec_refined).sumSquare() < tvec_thresh_noise);
    CHECK((vpColVector(tu_true) - vpColVector(tu_refined)).sumSquare() < tu_thresh_noise);
  }
}

TEST_CASE("Pose refinement on 3 points", "[pose_refine]") {
  int npt = 3;
  std::vector<vpPoint> P(npt); //  Point to be tracked
  double Z = 0.03;

  P[0].setWorldCoordinates(-L, -L, Z);
  P[1].setWorldCoordinates( L, -L, Z);
  P[2].setWorldCoordinates( L,  L, Z);

  vpTranslationVector tvec_true(0.1, 0.2, 0.3);
  vpThetaUVector tu_true(0.1, -0.2, 0.67);

  for (auto& pt : P) {
    pt.project(vpHomogeneousMatrix(tvec_true, tu_true));
  }

  vpTranslationVector tvec_init(0.15, 0.25, 0.35);
  vpThetaUVector tu_init(0.05, -0.25, 0.7);

  vpPose pose;
  pose.addPoints(P);
  vpHomogeneousMatrix cMo_est(tvec_init, tu_init);

  SECTION("VVS")
  {
    pose.computePose(vpPose::VIRTUAL_VS, cMo_est);

    vpTranslationVector tvec_refined(cMo_est);
    vpThetaUVector tu_refined(cMo_est);

    std::cout << "\nPose refinement on " << npt << " points using VVS method:" << std::endl;
    std::cout << "tvec_true: " << tvec_true.t() << std::endl;
    std::cout << "tvec_refined: " << tvec_refined.t() << std::endl;
    std::cout << "tvec error: " << std::sqrt((tvec_true - tvec_refined).sumSquare()) << std::endl;
    std::cout << "tu_true: " << tu_true.t() << std::endl;
    std::cout << "tu_refined: " << tu_refined.t() << std::endl;
    std::cout << "tu error: " << std::sqrt((vpColVector(tu_true) - vpColVector(tu_refined)).sumSquare()) << std::endl;

    CHECK((tvec_true - tvec_refined).sumSquare() < tvec_thresh);
    CHECK((vpColVector(tu_true) - vpColVector(tu_refined)).sumSquare() < tu_thresh);
  }

  SECTION("Lowe")
  {
    pose.computePose(vpPose::LOWE, cMo_est);

    vpTranslationVector tvec_refined(cMo_est);
    vpThetaUVector tu_refined(cMo_est);

    std::cout << "\nPose refinement on " << npt << " points using Lowe method:" << std::endl;
    std::cout << "tvec_true: " << tvec_true.t() << std::endl;
    std::cout << "tvec_refined: " << tvec_refined.t() << std::endl;
    std::cout << "tvec error: " << std::sqrt((tvec_true - tvec_refined).sumSquare()) << std::endl;
    std::cout << "tu_true: " << tu_true.t() << std::endl;
    std::cout << "tu_refined: " << tu_refined.t() << std::endl;
    std::cout << "tu error: " << std::sqrt((vpColVector(tu_true) - vpColVector(tu_refined)).sumSquare()) << std::endl;

    CHECK((tvec_true - tvec_refined).sumSquare() < tvec_thresh);
    CHECK((vpColVector(tu_true) - vpColVector(tu_refined)).sumSquare() < tu_thresh);
  }
}

TEST_CASE("Pose computation/refinement on invalid number of points", "[pose_invalid]") {
  SECTION("Pose computation")
  {
    std::vector<vpPoint> P(3);
    vpPose pose;
    pose.addPoints(P);

    std::vector<vpPose::vpPoseMethodType> poseMethods {
      vpPose::LAGRANGE, vpPose::DEMENTHON, vpPose::RANSAC, vpPose::LAGRANGE_LOWE, vpPose::DEMENTHON_LOWE,
      vpPose::DEMENTHON_VIRTUAL_VS, vpPose::LAGRANGE_VIRTUAL_VS
    };

    for (const auto& method : poseMethods) {
      vpHomogeneousMatrix cMo_est;
      CHECK_THROWS_AS(pose.computePose(method, cMo_est), vpPoseException);
    }
  }

  SECTION("Pose refinement")
  {
    std::vector<vpPoint> P(2);
    vpPose pose;
    pose.addPoints(P);

    std::vector<vpPose::vpPoseMethodType> poseMethods {
      vpPose::LOWE, vpPose::VIRTUAL_VS
    };

    for (const auto& method : poseMethods) {
      vpHomogeneousMatrix cMo_est;
      CHECK_THROWS_AS(pose.computePose(method, cMo_est), vpPoseException);
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
  return 0;
}
#endif
