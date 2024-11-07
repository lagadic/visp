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
 * Compute the pose of a 3D object using the Dementhon, Lagrange and
 * Non-Linear approach.
 */

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpGaussRand.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpRxyzVector.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/vision/vpPose.h>

#include <stdio.h>
#include <stdlib.h>

#define L 0.035
#define L2 0.1

/*!
  \example testPose.cpp

  Compute the pose of a 3D object using the Dementhon, Lagrange and
  Non-Linear approach.

*/

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

void print_pose(const vpHomogeneousMatrix &cMo, const std::string &legend);
int compare_pose(const vpPose &pose, const vpHomogeneousMatrix &cMo_ref, const vpHomogeneousMatrix &cMo_est, const vpCameraParameters &cam,
  const std::string &legend, const double &translation3DThresh, const double &rotationRadian3DThresh, const double &pose2DThresh, const double &posePixThresh);

int compare_pose(const vpPose &pose, const vpHomogeneousMatrix &cMo_ref, const vpHomogeneousMatrix &cMo_est, const vpCameraParameters &cam,
  const std::string &legend)
{
  return compare_pose(pose, cMo_ref, cMo_est, cam,
    legend, 0.001, 0.001, 0.001, 1.);
}

// print the resulting estimated pose
void print_pose(const vpHomogeneousMatrix &cMo, const std::string &legend)
{
  vpPoseVector cpo = vpPoseVector(cMo);

  std::cout << std::endl
    << legend << "\n "
    << "tx  = " << cpo[0] << "\n "
    << "ty  = " << cpo[1] << "\n "
    << "tz  = " << cpo[2] << "\n "
    << "tux = vpMath::rad(" << vpMath::deg(cpo[3]) << ")\n "
    << "tuy = vpMath::rad(" << vpMath::deg(cpo[4]) << ")\n "
    << "tuz = vpMath::rad(" << vpMath::deg(cpo[5]) << ")\n"
    << std::endl;
}

// test if pose is well estimated
int compare_pose(const vpPose &pose, const vpHomogeneousMatrix &cMo_ref, const vpHomogeneousMatrix &cMo_est, const vpCameraParameters &cam,
  const std::string &legend, const double &translation3DThresh, const double &rotation3DThresh, const double &pose2DThresh, const double &posePixThresh)
{
  vpPoseVector pose_ref = vpPoseVector(cMo_ref);
  vpPoseVector pose_est = vpPoseVector(cMo_est);

  int fail_3d = 0;

  // Test done on the 3D pose
  for (unsigned int i = 0; i < 6; i++) {
    double pose3DThresh = 0.;
    if (i < 3) {
      pose3DThresh = translation3DThresh;
    }
    else {
      pose3DThresh = rotation3DThresh;
    }
    if (std::fabs(pose_ref[i] - pose_est[i]) > pose3DThresh) {
      fail_3d = 1;
      std::cout << "ref[" << i << "] - est[" << i << "] = " << pose_ref[i] - pose_est[i] << " > " << pose3DThresh << std::endl;
    }
  }

  std::cout << "Based on 3D parameters " << legend << " is " << (fail_3d ? "badly" : "well") << " estimated" << std::endl;

  // // Test done on the residual

  // Residual expressed in meters
  double r = pose.computeResidual(cMo_est);
  if (pose.listP.size() < 4) {
    fail_3d = 1;
    std::cout << "Not enough point" << std::endl;
    return fail_3d;
  }
  r = sqrt(r / pose.listP.size());
  // std::cout << "Residual on each point (meter): " << r << std::endl;
  int fail_2d = (r > pose2DThresh) ? 1 : 0;
  std::cout << "Based on 2D residual (" << r << ") " << legend << " is " << (fail_2d ? "badly" : "well") << " estimated"
    << std::endl;

  // Residual expressed in pixels
  double r_pix = pose.computeResidual(cMo_est, cam);
  r_pix = sqrt(r_pix / pose.listP.size());
  // std::cout << "Residual on each point (pixel): " << r << std::endl;
  int fail_pix = (r_pix > posePixThresh) ? 1 : 0;
  std::cout << "Based on pixel residual (" << r_pix << ") " << legend << " is " << (fail_pix ? "badly" : "well") << " estimated"
    << std::endl;
  return fail_3d + fail_2d + fail_pix;
}

int main()
{
#if (defined(VISP_HAVE_LAPACK) || defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_OPENCV))
  try {
    int test_planar_fail = 0, test_non_planar_fail = 0, fail = 0;
    const double translation3DthreshWhenNoise = 0.005;
    const double rotation3DthreshWhenNoise = vpMath::rad(1.);
    const double residual2DWhenNoise = 0.001;
    const double residualPixelWhenNoise = 1.;

    vpHomogeneousMatrix cMo; // will contain the estimated pose
    vpCameraParameters cam; // Default camera parameters to compute the residual in terms of pixel

    {
      //
      // Test planar case with 4 points
      //

      std::cout << "Start test considering planar case with 4 points..." << std::endl;
      std::cout << "===================================================" << std::endl;

      // vpPoseVector cpo_ref = vpPoseVector(0.01, 0.02, 0.25, vpMath::rad(5), 0, vpMath::rad(10));
      vpPoseVector cpo_ref = vpPoseVector(-0.01, -0.02, 0.3, vpMath::rad(20), vpMath::rad(-20), vpMath::rad(10));
      vpHomogeneousMatrix cMo_ref(cpo_ref);

      int npt = 4;
      std::vector<vpPoint> P(npt); //  Point to be tracked
      double Z = 0.05;             // FS: Dementhon estimation is not good when Z=0.3

      P[0].setWorldCoordinates(-L, -L, Z);
      P[1].setWorldCoordinates(L, -L, Z);
      P[2].setWorldCoordinates(L, L, Z);
      P[3].setWorldCoordinates(-L, L, Z);

      vpPose pose;

      for (int i = 0; i < npt; i++) {
        P[i].project(cMo_ref);
        // P[i].print();
        pose.addPoint(P[i]); // and added to the pose computation class
      }

      // Let's go ...

      print_pose(cMo_ref, std::string("Reference pose"));

      std::cout << "-------------------------------------------------" << std::endl;
      pose.computePose(vpPose::LAGRANGE, cMo);

      print_pose(cMo, std::string("Pose estimated by Lagrange"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Lagrange");
      test_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Dementhon");
      test_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;

      pose.setRansacNbInliersToReachConsensus(4);
      pose.setRansacThreshold(0.01);
      pose.computePose(vpPose::RANSAC, cMo);

      print_pose(cMo, std::string("Pose estimated by Ransac"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Ransac");
      test_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::LAGRANGE_LOWE, cMo);

      print_pose(cMo, std::string("Pose estimated by Lagrange then Lowe"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Lagrange then Lowe");
      test_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON_LOWE, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon then Lowe"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Dementhon then Lowe");
      test_planar_fail |= fail;

      // Now Virtual Visual servoing
      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by VVS");
      test_planar_fail |= fail;

      std::cout << "-------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON_VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon then by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Dementhon then by VVS");
      test_planar_fail |= fail;

      std::cout << "-------------------------------------------------" << std::endl;
      pose.computePose(vpPose::LAGRANGE_VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated by Lagrange then by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Lagrange then by VVS");
      test_planar_fail |= fail;

      std::cout << "-------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON_LAGRANGE_VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated either by Dementhon or Lagrange then by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose either by Dementhon or Lagrange then by VVS");
      test_planar_fail |= fail;
    }

    {
      //
      // Test non-planar case with 6 points (at least 6 points for Lagrange non planar)
      //

      std::cout << "\nStart test considering non-planar case with 6 points..." << std::endl;
      std::cout << "=======================================================" << std::endl;

      vpPoseVector cpo_ref = vpPoseVector(0.01, 0.02, 0.25, vpMath::rad(5), 0, vpMath::rad(10));
      vpHomogeneousMatrix cMo_ref(cpo_ref);

      int npt = 6;
      std::vector<vpPoint> P(npt);         //  Point to be tracked
      P[0].setWorldCoordinates(-L, -L, 0); // Lagrange not accurate...
      P[0].setWorldCoordinates(-L, -L, -0.02);
      P[1].setWorldCoordinates(L, -L, 0);
      P[2].setWorldCoordinates(L, L, 0);
      P[3].setWorldCoordinates(-2 * L, 3 * L, 0);
      P[4].setWorldCoordinates(-L, L, 0.01);
      P[5].setWorldCoordinates(L, L / 2., 0.03);

      vpPose pose;

      for (int i = 0; i < npt; i++) {
        P[i].project(cMo_ref);
        // P[i].print();
        pose.addPoint(P[i]); // and added to the pose computation class
      }

      // Let's go ...
      print_pose(cMo_ref, std::string("Reference pose"));

      std::cout << "-------------------------------------------------" << std::endl;
      pose.computePose(vpPose::LAGRANGE, cMo);

      print_pose(cMo, std::string("Pose estimated by Lagrange"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Lagrange");
      test_non_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Dementhon");
      test_non_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.setRansacNbInliersToReachConsensus(4);
      pose.setRansacThreshold(0.01);
      pose.computePose(vpPose::RANSAC, cMo);

      print_pose(cMo, std::string("Pose estimated by Ransac"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Ransac");
      test_non_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::LAGRANGE_LOWE, cMo);

      print_pose(cMo, std::string("Pose estimated by Lagrange then Lowe"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Lagrange then Lowe");
      test_non_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON_LOWE, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon then Lowe"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Dementhon then Lowe");
      test_non_planar_fail |= fail;

      // Now Virtual Visual servoing

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by VVS");
      test_non_planar_fail |= fail;

      std::cout << "-------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON_VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon then by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Dementhon then by VVS");
      test_non_planar_fail |= fail;

      std::cout << "-------------------------------------------------" << std::endl;
      pose.computePose(vpPose::LAGRANGE_VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated by Lagrange then by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Lagrange then by VVS");
      test_non_planar_fail |= fail;

      std::cout << "-------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON_LAGRANGE_VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated  either by Dementhon or Lagrange then by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose either by Dementhon or Lagrange then by VVS");
      test_non_planar_fail |= fail;
    }

    //
    // Test non-planar case with 4 points (Lagrange can not be used)
    //

    std::cout << "\nStart test considering non-planar case with 4 points..." << std::endl;
    std::cout << "=======================================================" << std::endl;

    {
      int npt = 4;
      std::vector<vpPoint> P(npt); //  Point to be tracked
      P[0].setWorldCoordinates(-L2, -L2, 0);
      P[1].setWorldCoordinates(L2, -L2, 0.2);
      P[2].setWorldCoordinates(L2, L2, -0.1);
      P[3].setWorldCoordinates(-L2, L2, 0);

      vpPose pose;

      vpPoseVector cpo_ref = vpPoseVector(-0.1, -0.2, 0.8, vpMath::rad(10), vpMath::rad(-10), vpMath::rad(25));
      vpHomogeneousMatrix cMo_ref(cpo_ref);

      for (int i = 0; i < npt; i++) {
        P[i].project(cMo_ref);
        //  P[i].print(); printf("\n");
        pose.addPoint(P[i]); // and added to the pose computation class
      }

      // Let's go ...
      print_pose(cMo_ref, std::string("Reference pose"));

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Dementhon");
      test_non_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.setRansacNbInliersToReachConsensus(4);
      pose.setRansacThreshold(0.01);
      pose.computePose(vpPose::RANSAC, cMo);

      print_pose(cMo, std::string("Pose estimated by Ransac"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Ransac");
      test_non_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON_LOWE, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon then Lowe"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Dementhon then Lowe");
      test_non_planar_fail |= fail;

      // Now Virtual Visual servoing
      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by VVS");
      test_non_planar_fail |= fail;

      std::cout << "-------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON_VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon then by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Dementhon then by VVS");
      test_non_planar_fail |= fail;

      std::cout << "-------------------------------------------------" << std::endl;

      pose.computePose(vpPose::DEMENTHON_LAGRANGE_VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated either by Dementhon or Lagrange then by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose either by Dementhon or Lagrange then by VVS");
      test_non_planar_fail |= fail;

      std::cout << "-------------------------------------------------" << std::endl;
    }

    //
    // Test computeResidual with results expressed in pixel
    //

    std::cout << "Start test considering planar case with 4 points and noise on the projection..." << std::endl;
    std::cout << "===================================================" << std::endl;
    {
      vpPoseVector cpo_ref = vpPoseVector(-0.01, -0.02, 0.3, vpMath::rad(20), vpMath::rad(-20), vpMath::rad(10));
      vpHomogeneousMatrix cMo_ref(cpo_ref);

      int npt = 4;
      std::vector<vpPoint> P(npt); //  Point to be tracked
      double Z = 0.05;             // FS: Dementhon estimation is not good when Z=0.3

      P[0].setWorldCoordinates(-L, -L, Z);
      P[1].setWorldCoordinates(L, -L, Z);
      P[2].setWorldCoordinates(L, L, Z);
      P[3].setWorldCoordinates(-L, L, Z);

      vpPose pose;
      vpGaussRand random(0.08, 0., 42); // Gaussian noise of mean = 0. and sigma = 1.

      for (int i = 0; i < npt; i++) {
        // Projecting point in camera frame
        P[i].project(cMo_ref);

        // Computing theoretical u and v based on the 2D coordinates
        double x_theo = P[i].get_X() / P[i].get_Z();
        double y_theo = P[i].get_Y() / P[i].get_Z();
        double u_theo = 0., v_theo = 0.;
        vpMeterPixelConversion::convertPoint(cam, x_theo, y_theo, u_theo, v_theo);

        // Adding noise to u, v
        double u_noisy = u_theo + random();
        double v_noisy = v_theo + random();

        // Computing corresponding x, y
        double x_noisy = 0., y_noisy = 0.;
        vpPixelMeterConversion::convertPoint(cam, u_noisy, v_noisy, x_noisy, y_noisy);

        P[i].set_x(x_noisy);
        P[i].set_y(y_noisy);

        pose.addPoint(P[i]); // and added to the pose computation class
        std::cout << "P[" << i << "]:\n\tu_theo = " << u_theo << "\tu_noisy = " << u_noisy << std::endl;
        std::cout << "\tv_theo = " << v_theo << "\tv_noisy = " << v_noisy << std::endl;
        std::cout << "\tx_theo = " << x_theo << "\ty_noisy = " << x_noisy << std::endl;
        std::cout << "\ty_theo = " << y_theo << "\tx_noisy = " << y_noisy << std::endl;
      }

      print_pose(cMo_ref, std::string("Reference pose"));

      std::cout << "-------------------------------------------------" << std::endl;
      pose.computePose(vpPose::LAGRANGE, cMo);

      print_pose(cMo, std::string("Pose estimated by Lagrange"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Lagrange"
        , translation3DthreshWhenNoise, rotation3DthreshWhenNoise, residual2DWhenNoise, residualPixelWhenNoise);
      test_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Dementhon"
        , translation3DthreshWhenNoise, rotation3DthreshWhenNoise, residual2DWhenNoise, residualPixelWhenNoise);
      test_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;

      pose.setRansacNbInliersToReachConsensus(4);
      pose.setRansacThreshold(0.01);
      pose.computePose(vpPose::RANSAC, cMo);

      print_pose(cMo, std::string("Pose estimated by Ransac"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Ransac"
        , translation3DthreshWhenNoise, rotation3DthreshWhenNoise, residual2DWhenNoise, residualPixelWhenNoise);
      test_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::LAGRANGE_LOWE, cMo);

      print_pose(cMo, std::string("Pose estimated by Lagrange then Lowe"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Lagrange then Lowe"
        , translation3DthreshWhenNoise, rotation3DthreshWhenNoise, residual2DWhenNoise, residualPixelWhenNoise);
      test_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON_LOWE, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon then Lowe"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Dementhon then Lowe"
        , translation3DthreshWhenNoise, rotation3DthreshWhenNoise, residual2DWhenNoise, residualPixelWhenNoise);
      test_planar_fail |= fail;

      // Now Virtual Visual servoing
      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by VVS"
        , translation3DthreshWhenNoise, rotation3DthreshWhenNoise, residual2DWhenNoise, residualPixelWhenNoise);
      test_planar_fail |= fail;

      std::cout << "-------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON_VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon then by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Dementhon then by VVS"
        , translation3DthreshWhenNoise, rotation3DthreshWhenNoise, residual2DWhenNoise, residualPixelWhenNoise);
      test_planar_fail |= fail;

      std::cout << "-------------------------------------------------" << std::endl;
      pose.computePose(vpPose::LAGRANGE_VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated by Lagrange then by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Lagrange then by VVS"
        , translation3DthreshWhenNoise, rotation3DthreshWhenNoise, residual2DWhenNoise, residualPixelWhenNoise);
      test_planar_fail |= fail;

      std::cout << "-------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON_LAGRANGE_VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated either by Dementhon or Lagrange then by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose either by Dementhon or Lagrange then by VVS"
        , translation3DthreshWhenNoise, rotation3DthreshWhenNoise, residual2DWhenNoise, residualPixelWhenNoise);
      test_planar_fail |= fail;
    }

    //
    // Test non-planar case with 6 points (at least 6 points for Lagrange non planar)
    //

    std::cout << "\nStart test considering non-planar case with 6 points and noise on the projection..." << std::endl;
    std::cout << "=======================================================" << std::endl;

    {
      vpPoseVector cpo_ref = vpPoseVector(0.01, 0.02, 0.25, vpMath::rad(5), 0, vpMath::rad(10));
      vpHomogeneousMatrix cMo_ref(cpo_ref);

      int npt = 6;
      std::vector<vpPoint> P(npt);         //  Point to be tracked
      P[0].setWorldCoordinates(-L, -L, 0); // Lagrange not accurate...
      P[0].setWorldCoordinates(-L, -L, -0.02);
      P[1].setWorldCoordinates(L, -L, 0);
      P[2].setWorldCoordinates(L, L, 0);
      P[3].setWorldCoordinates(-2 * L, 3 * L, 0);
      P[4].setWorldCoordinates(-L, L, 0.01);
      P[5].setWorldCoordinates(L, L / 2., 0.03);

      vpPose pose;
      vpGaussRand random(0.08, 0., 42); // Gaussian noise of mean = 0. and sigma = 1.

      for (int i = 0; i < npt; i++) {
        // Projecting point in camera frame
        P[i].project(cMo_ref);

        // Computing theoretical u and v based on the 2D coordinates
        double x_theo = P[i].get_X() / P[i].get_Z();
        double y_theo = P[i].get_Y() / P[i].get_Z();
        double u_theo = 0., v_theo = 0.;
        vpMeterPixelConversion::convertPoint(cam, x_theo, y_theo, u_theo, v_theo);

        // Adding noise to u, v
        double u_noisy = u_theo + random();
        double v_noisy = v_theo + random();

        // Computing corresponding x, y
        double x_noisy = 0., y_noisy = 0.;
        vpPixelMeterConversion::convertPoint(cam, u_noisy, v_noisy, x_noisy, y_noisy);

        P[i].set_x(x_noisy);
        P[i].set_y(y_noisy);

        pose.addPoint(P[i]); // and added to the pose computation class
        std::cout << "P[" << i << "]:\n\tu_theo = " << u_theo << "\tu_noisy = " << u_noisy << std::endl;
        std::cout << "\tv_theo = " << v_theo << "\tv_noisy = " << v_noisy << std::endl;
        std::cout << "\tx_theo = " << x_theo << "\ty_noisy = " << x_noisy << std::endl;
        std::cout << "\ty_theo = " << y_theo << "\tx_noisy = " << y_noisy << std::endl;
      }

      // Let's go ...
      print_pose(cMo_ref, std::string("Reference pose"));

      std::cout << "-------------------------------------------------" << std::endl;
      pose.computePose(vpPose::LAGRANGE, cMo);

      print_pose(cMo, std::string("Pose estimated by Lagrange"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Lagrange"
        , translation3DthreshWhenNoise, rotation3DthreshWhenNoise, residual2DWhenNoise, residualPixelWhenNoise);
      test_non_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Dementhon"
        , translation3DthreshWhenNoise, rotation3DthreshWhenNoise, residual2DWhenNoise, residualPixelWhenNoise);
      test_non_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.setRansacNbInliersToReachConsensus(4);
      pose.setRansacThreshold(0.01);
      pose.computePose(vpPose::RANSAC, cMo);

      print_pose(cMo, std::string("Pose estimated by Ransac"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Ransac"
        , translation3DthreshWhenNoise, rotation3DthreshWhenNoise, residual2DWhenNoise, residualPixelWhenNoise);
      test_non_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::LAGRANGE_LOWE, cMo);

      print_pose(cMo, std::string("Pose estimated by Lagrange then Lowe"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Lagrange then Lowe"
        , translation3DthreshWhenNoise, rotation3DthreshWhenNoise, residual2DWhenNoise, residualPixelWhenNoise);
      test_non_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON_LOWE, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon then Lowe"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Dementhon then Lowe"
        , translation3DthreshWhenNoise, rotation3DthreshWhenNoise, residual2DWhenNoise, residualPixelWhenNoise);
      test_non_planar_fail |= fail;

      // Now Virtual Visual servoing

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by VVS"
        , translation3DthreshWhenNoise, rotation3DthreshWhenNoise, residual2DWhenNoise, residualPixelWhenNoise);
      test_non_planar_fail |= fail;

      std::cout << "-------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON_VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon then by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Dementhon then by VVS"
        , translation3DthreshWhenNoise, rotation3DthreshWhenNoise, residual2DWhenNoise, residualPixelWhenNoise);
      test_non_planar_fail |= fail;

      std::cout << "-------------------------------------------------" << std::endl;
      pose.computePose(vpPose::LAGRANGE_VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated by Lagrange then by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Lagrange then by VVS"
        , translation3DthreshWhenNoise, rotation3DthreshWhenNoise, residual2DWhenNoise, residualPixelWhenNoise);
      test_non_planar_fail |= fail;

      std::cout << "-------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON_LAGRANGE_VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated  either by Dementhon or Lagrange then by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose either by Dementhon or Lagrange then by VVS"
        , translation3DthreshWhenNoise, rotation3DthreshWhenNoise, residual2DWhenNoise, residualPixelWhenNoise);
      test_non_planar_fail |= fail;
    }

    //
    // Test non-planar case with 4 points (Lagrange can not be used)
    //

    std::cout << "\nStart test considering non-planar case with 4 points and noise on the projection..." << std::endl;
    std::cout << "=======================================================" << std::endl;

    {
      int npt = 4;
      std::vector<vpPoint> P(npt); //  Point to be tracked
      P[0].setWorldCoordinates(-L2, -L2, 0.2);
      P[1].setWorldCoordinates(L2, -L2, 0.4);
      P[2].setWorldCoordinates(L2, L2, 0.1);
      P[3].setWorldCoordinates(-L2, L2, 0.4);

      vpPose pose;

      vpPoseVector cpo_ref = vpPoseVector(-0.1, -0.2, 0.8, vpMath::rad(10), vpMath::rad(-10), vpMath::rad(25));
      vpHomogeneousMatrix cMo_ref(cpo_ref);

      vpGaussRand random(0.08, 0., 42); // Gaussian noise of mean = 0. and sigma = 1.

      for (int i = 0; i < npt; i++) {
        // Projecting point in camera frame
        P[i].project(cMo_ref);

        // Computing theoretical u and v based on the 2D coordinates
        double x_theo = P[i].get_X() / P[i].get_Z();
        double y_theo = P[i].get_Y() / P[i].get_Z();
        double u_theo = 0., v_theo = 0.;
        vpMeterPixelConversion::convertPoint(cam, x_theo, y_theo, u_theo, v_theo);

        // Adding noise to u, v
        double u_noisy = u_theo + random();
        double v_noisy = v_theo + random();

        // Computing corresponding x, y
        double x_noisy = 0., y_noisy = 0.;
        vpPixelMeterConversion::convertPoint(cam, u_noisy, v_noisy, x_noisy, y_noisy);

        P[i].set_x(x_noisy);
        P[i].set_y(y_noisy);

        pose.addPoint(P[i]); // and added to the pose computation class
        std::cout << "P[" << i << "]:\n\tu_theo = " << u_theo << "\tu_noisy = " << u_noisy << std::endl;
        std::cout << "\tv_theo = " << v_theo << "\tv_noisy = " << v_noisy << std::endl;
        std::cout << "\tx_theo = " << x_theo << "\ty_noisy = " << x_noisy << std::endl;
        std::cout << "\ty_theo = " << y_theo << "\tx_noisy = " << y_noisy << std::endl;
      }

      // Let's go ...
      print_pose(cMo_ref, std::string("Reference pose"));

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Dementhon"
        , translation3DthreshWhenNoise, rotation3DthreshWhenNoise, residual2DWhenNoise, residualPixelWhenNoise);
      test_non_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.setRansacNbInliersToReachConsensus(4);
      pose.setRansacThreshold(0.01);
      pose.computePose(vpPose::RANSAC, cMo);

      print_pose(cMo, std::string("Pose estimated by Ransac"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Ransac"
        , translation3DthreshWhenNoise, rotation3DthreshWhenNoise, residual2DWhenNoise, residualPixelWhenNoise);
      test_non_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON_LOWE, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon then Lowe"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Dementhon then Lowe"
        , translation3DthreshWhenNoise, rotation3DthreshWhenNoise, residual2DWhenNoise, residualPixelWhenNoise);
      test_non_planar_fail |= fail;

      // Now Virtual Visual servoing
      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by VVS"
        , translation3DthreshWhenNoise, rotation3DthreshWhenNoise, residual2DWhenNoise, residualPixelWhenNoise);
      test_non_planar_fail |= fail;

      std::cout << "-------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON_VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon then by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose by Dementhon then by VVS"
        , translation3DthreshWhenNoise, rotation3DthreshWhenNoise, residual2DWhenNoise, residualPixelWhenNoise);
      test_non_planar_fail |= fail;

      std::cout << "-------------------------------------------------" << std::endl;

      pose.computePose(vpPose::DEMENTHON_LAGRANGE_VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated either by Dementhon or Lagrange then by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, cam, "pose either by Dementhon or Lagrange then by VVS"
        , translation3DthreshWhenNoise, rotation3DthreshWhenNoise, residual2DWhenNoise, residualPixelWhenNoise);
      test_non_planar_fail |= fail;

      std::cout << "-------------------------------------------------" << std::endl;
    }

    std::cout << "=======================================================" << std::endl;
    std::cout << "Pose estimation test from planar points: " << (test_planar_fail ? "fail" : "is ok") << std::endl;
    std::cout << "Pose estimation test from non-planar points: " << (test_non_planar_fail ? "fail" : "is ok")
      << std::endl;
    std::cout << "Global pose estimation test: " << ((test_planar_fail | test_non_planar_fail) ? "fail" : "is ok")
      << std::endl;

    return ((test_planar_fail | test_non_planar_fail) ? EXIT_FAILURE : EXIT_SUCCESS);
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
#else
  std::cout << "Cannot run this example: install Lapack, Eigen3 or OpenCV" << std::endl;
  return EXIT_SUCCESS;
#endif
}
