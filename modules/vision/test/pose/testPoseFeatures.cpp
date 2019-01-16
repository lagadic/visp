/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * Compute the pose from visual features by virtual visual servoing.
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

#include <iostream>
#include <limits>
#include <vector>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpPoint.h>
#include <visp3/vision/vpPose.h>
#include <visp3/vision/vpPoseFeatures.h>

/*!
  \example testPoseFeatures.cpp

  Compute the pose from different visual features.

*/

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#ifdef VISP_HAVE_CPP11_COMPATIBILITY
class vp_createPointClass
{
public:
  int value;

  vp_createPointClass() : value(0) {}

  int vp_createPoint(vpFeaturePoint &fp, const vpPoint &v)
  {
    value += 1;
    vpFeatureBuilder::create(fp, v);
    return value;
  }
};

void vp_createPoint(vpFeaturePoint &fp, const vpPoint &v) { vpFeatureBuilder::create(fp, v); }

void vp_createLine(vpFeatureLine &fp, const vpLine &v) { vpFeatureBuilder::create(fp, v); }
#endif
#endif

int test_pose(bool use_robust)
{
  if (use_robust)
    std::cout << "** Test robust pose estimation from features\n" << std::endl;
  else
    std::cout << "** Test pose estimation from features\n" << std::endl;

  vpImage<unsigned char> I(600, 600);

  vpHomogeneousMatrix cMo_ref(0., 0., 1., vpMath::rad(0), vpMath::rad(0), vpMath::rad(60));
  vpPoseVector pose_ref = vpPoseVector(cMo_ref);

  std::cout << "Reference pose used to create the visual features : " << std::endl;
  std::cout << pose_ref.t() << std::endl;

  vpPoseFeatures pose;

  std::vector<vpPoint> pts;

  double val = 0.25;
  double val2 = 0.0;

  // 2D Point Feature
  pts.push_back(vpPoint(0.0, -val, val2));
  pts.push_back(vpPoint(0.0, val, val2));
  pts.push_back(vpPoint(-val, val, val2));

  // Segment Feature
  pts.push_back(vpPoint(-val, -val / 2.0, val2));
  pts.push_back(vpPoint(val, val / 2.0, val2));

  // 3D point Feature
  pts.push_back(vpPoint(0.0, 0.0, -1.5));

  // Line Feature
  vpLine line;
  line.setWorldCoordinates(0.0, 1.0, 0.0, .0, 0.0, 0.0, 1.0, 0.0);

  // Vanishing Point Feature
  vpLine l1;
  l1.setWorldCoordinates(0.0, 1.0, 0.2, 0.0, 1.0, 0.0, 0.0, -0.25);

  vpLine l2;
  l2.setWorldCoordinates(0.0, 1.0, 0.2, 0.0, -1.0, 0.0, 0.0, -0.25);

  // Ellipse Feature
  vpCircle circle;
  circle.setWorldCoordinates(0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.25);

  pts[0].project(cMo_ref);
  pts[1].project(cMo_ref);
  pts[2].project(cMo_ref);

  pts[3].project(cMo_ref);
  pts[4].project(cMo_ref);

  pts[5].project(cMo_ref);

  line.project(cMo_ref);

  l1.project(cMo_ref);
  l2.project(cMo_ref);

  circle.project(cMo_ref);

  pose.addFeaturePoint(pts[0]);
  //   pose.addFeaturePoint(pts[1]);
  pose.addFeaturePoint(pts[2]);

  pose.addFeaturePoint3D(pts[5]);

  pose.addFeatureVanishingPoint(l1, l2);

  //   pose.addFeatureSegment(pts[3],pts[4]);
  //
  //   pose.addFeatureLine(line);

  pose.addFeatureEllipse(circle);

#ifdef VISP_HAVE_CPP11_COMPATIBILITY
  vpFeaturePoint fp;
  vpFeatureLine fl;
  vpFeatureSegment fs;
  void (*ptr)(vpFeatureSegment &, vpPoint &, vpPoint &) = &vpFeatureBuilder::create;
  vp_createPointClass cpClass;
  int (vp_createPointClass::*ptrClass)(vpFeaturePoint &, const vpPoint &) = &vp_createPointClass::vp_createPoint;
  pose.addSpecificFeature(&cpClass, ptrClass, fp, pts[1]);
  pose.addSpecificFeature(&vp_createLine, fl, line);
  pose.addSpecificFeature(ptr, fs, pts[3], pts[4]);
#endif

  pose.setVerbose(true);
  pose.setLambda(0.6);
  pose.setVVSIterMax(200);
  pose.setCovarianceComputation(true);

  vpHomogeneousMatrix cMo_est(0.4, 0.3, 1.5, vpMath::rad(0), vpMath::rad(0), vpMath::rad(0));
  vpPoseVector pose_est = vpPoseVector(cMo_est);
  std::cout << "\nPose used as initialisation of the pose computation : " << std::endl;
  std::cout << pose_est.t() << std::endl;

  if (!use_robust)
    pose.computePose(cMo_est);
  else
    pose.computePose(cMo_est, vpPoseFeatures::ROBUST_VIRTUAL_VS);

  if (!use_robust)
    std::cout << "\nEstimated pose from visual features : " << std::endl;
  else
    std::cout << "\nRobust estimated pose from visual features : " << std::endl;

  pose_est.buildFrom(cMo_est);
  std::cout << pose_est.t() << std::endl;

  std::cout << "\nResulting covariance (Diag): " << std::endl;
  vpMatrix covariance = pose.getCovarianceMatrix();
  std::cout << covariance[0][0] << " " << covariance[1][1] << " " << covariance[2][2] << " " << covariance[3][3] << " "
            << covariance[4][4] << " " << covariance[5][5] << " " << std::endl;

  int test_fail = 0;
  for (unsigned int i = 0; i < 6; i++) {
    if (std::fabs(pose_ref[i] - pose_est[i]) > 0.001)
      test_fail = 1;
  }

  std::cout << "\nPose is " << (test_fail ? "badly" : "well") << " estimated\n" << std::endl;

  return test_fail;
}

int main()
{
  try {
    if (test_pose(false))
      return -1;

    if (test_pose(true))
      return -1;

    return 0;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getStringMessage() << std::endl;
    return -1;
  }
}
