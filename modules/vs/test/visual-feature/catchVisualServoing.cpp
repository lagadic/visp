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
 * Test visual features and visual servoing.
 */

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2)

#if defined(VISP_BUILD_CATCH2)
#include <catch_amalgamated.hpp>
#else // Since v3.1.1
#include <catch2/catch_all.hpp>
#endif

#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/vs/vpServo.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

TEST_CASE("Feature point")
{
  vpFeaturePoint feat_p_2d, feat_p_2d_des;
  feat_p_2d.set_x(0.1);
  feat_p_2d.set_y(0.2);
  feat_p_2d.set_Z(1.3);
  feat_p_2d_des.set_x(0);
  feat_p_2d_des.set_y(0);
  feat_p_2d_des.set_Z(1);

  vpServo task;
  task.setServo(vpServo::EYEINHAND_CAMERA);
  task.setInteractionMatrixType(vpServo::CURRENT);
  task.setLambda(0.5);

  SECTION("Feature point 2D (x,y)")
  {
    task.addFeature(feat_p_2d, feat_p_2d_des);
    REQUIRE(task.computeControlLaw().size() == 6);
    REQUIRE(task.getDimension() == 2);
    REQUIRE(task.getError().size() == 2);
    vpColVector error = feat_p_2d.error(feat_p_2d_des);
    REQUIRE(task.getError() == error);
  }
  SECTION("Feature point 2D (x)")
  {
    task.addFeature(feat_p_2d, feat_p_2d_des, vpFeaturePoint::selectX());
    REQUIRE(task.computeControlLaw().size() == 6);
    REQUIRE(task.getDimension() == 1);
    REQUIRE(task.getError().size() == 1);
    vpColVector error = feat_p_2d.error(feat_p_2d_des, vpFeaturePoint::selectX());
    REQUIRE(task.getError() == error);
  }
  SECTION("Feature point 2D (y)")
  {
    task.addFeature(feat_p_2d, feat_p_2d_des, vpFeaturePoint::selectY());
    REQUIRE(task.computeControlLaw().size() == 6);
    REQUIRE(task.getDimension() == 1);
    REQUIRE(task.getError().size() == 1);
    vpColVector error = feat_p_2d.error(feat_p_2d_des, vpFeaturePoint::selectY());
    REQUIRE(task.getError() == error);
  }
  SECTION("Feature point 2D (x,y) + depth (log(Z/Z*))")
  {
    vpFeatureDepth feat_depth, feat_depth_des;
    feat_depth.buildFrom(feat_p_2d.get_x(), feat_p_2d.get_y(), feat_p_2d.get_Z(), log(feat_p_2d.get_Z() / feat_p_2d_des.get_Z()));
    feat_depth_des.buildFrom(feat_p_2d_des.get_x(), feat_p_2d_des.get_y(), feat_p_2d_des.get_Z(), 0);
    task.addFeature(feat_p_2d, feat_p_2d_des);
    task.addFeature(feat_depth, feat_depth_des);
    REQUIRE(task.computeControlLaw().size() == 6);
    REQUIRE(task.getDimension() == 3);
    REQUIRE(task.getError().size() == 3);
    vpColVector error_feat_p_2d = feat_p_2d.error(feat_p_2d_des);
    vpColVector error_feat_depth = feat_depth.error(feat_depth_des);
    vpColVector error = vpColVector::stack(error_feat_p_2d, error_feat_depth);
    REQUIRE(task.getError() == error);
  }
}

TEST_CASE("Feature translation + theta_u")
{
  vpFeatureTranslation feat_translation, feat_translation_des;
  vpFeatureThetaU feat_theta_u, feat_theta_u_des;
  feat_translation.set_Tx(0.1);
  feat_translation.set_Ty(0.2);
  feat_translation.set_Tz(1.3);
  feat_translation_des.set_Tx(0.0);
  feat_translation_des.set_Ty(0.0);
  feat_translation_des.set_Tz(0.0);
  feat_theta_u.set_TUx(0.1);
  feat_theta_u.set_TUy(0.2);
  feat_theta_u.set_TUz(0.3);
  feat_theta_u_des.set_TUx(0);
  feat_theta_u_des.set_TUy(0);
  feat_theta_u_des.set_TUz(0);
  SECTION("Feature translation + theta_u")
  {
    vpServo task;
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(0.5);
    task.addFeature(feat_translation, feat_translation_des);
    task.addFeature(feat_theta_u, feat_theta_u_des);
    REQUIRE(task.computeControlLaw().size() == 6);
    REQUIRE(task.getDimension() == 6);
    REQUIRE(task.getError().size() == 6);
    vpColVector error_feat_translation = feat_translation.error(feat_translation_des);
    vpColVector error_feat_theta_u = feat_theta_u.error(feat_theta_u_des);
    vpColVector error = vpColVector::stack(error_feat_translation, error_feat_theta_u);
    REQUIRE(task.getError() == error);
  }
}

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();
  return numFailed;
}

#else

int main() { return EXIT_SUCCESS; }

#endif
