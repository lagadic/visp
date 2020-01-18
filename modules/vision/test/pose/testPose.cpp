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
 * Compute the pose of a 3D object using the Dementhon, Lagrange and
 * Non-Linear approach.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/core/vpDebug.h>
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

void print_pose(const vpHomogeneousMatrix &cMo, const std::string &legend);
int compare_pose(const vpPose &pose, const vpHomogeneousMatrix &cMo_ref, const vpHomogeneousMatrix &cMo_est,
                 const std::string &legend);

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
int compare_pose(const vpPose &pose, const vpHomogeneousMatrix &cMo_ref, const vpHomogeneousMatrix &cMo_est,
                 const std::string &legend)
{
  vpPoseVector pose_ref = vpPoseVector(cMo_ref);
  vpPoseVector pose_est = vpPoseVector(cMo_est);

  int fail = 0;

  // Test done on the 3D pose
  for (unsigned int i = 0; i < 6; i++) {
    if (std::fabs(pose_ref[i] - pose_est[i]) > 0.001)
      fail = 1;
  }

  std::cout << "Based on 3D parameters " << legend << " is " << (fail ? "badly" : "well") << " estimated" << std::endl;

  // Test done on the residual
  double r = pose.computeResidual(cMo_est);
  if (pose.listP.size() < 4) {
    fail = 1;
    std::cout << "Not enough point" << std::endl;
    return fail;
  }
  r = sqrt(r / pose.listP.size());
  // std::cout << "Residual on each point (meter): " << r << std::endl;
  fail = (r > 0.001) ? 1 : 0;
  std::cout << "Based on 2D residual (" << r << ") " << legend << " is " << (fail ? "badly" : "well") << " estimated"
            << std::endl;
  return fail;
}

int main()
{
  try {
    int test_planar_fail = 0, test_non_planar_fail = 0, fail = 0;

    vpHomogeneousMatrix cMo; // will contain the estimated pose

    {
      //
      // Test planar case with 4 points
      //

      std::cout << "Start test considering planar case with 4 points..." << std::endl;
      std::cout << "===================================================" << std::endl;

      //vpPoseVector cpo_ref = vpPoseVector(0.01, 0.02, 0.25, vpMath::rad(5), 0, vpMath::rad(10));
      vpPoseVector cpo_ref = vpPoseVector(-0.01, -0.02, 0.3, vpMath::rad(20),  vpMath::rad(-20), vpMath::rad(10));
      vpHomogeneousMatrix cMo_ref(cpo_ref);

      int npt = 4;
      std::vector<vpPoint> P(npt); //  Point to be tracked
      double Z = 0.05; // FS: Dementhon estimation is not good when Z=0.3

      P[0].setWorldCoordinates(-L, -L, Z);
      P[1].setWorldCoordinates( L, -L, Z);
      P[2].setWorldCoordinates( L,  L, Z);
      P[3].setWorldCoordinates(-L,  L, Z);

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
      fail = compare_pose(pose, cMo_ref, cMo, "pose by Lagrange");
      test_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon"));
      fail = compare_pose(pose, cMo_ref, cMo, "pose by Dementhon");
      test_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;

      pose.setRansacNbInliersToReachConsensus(4);
      pose.setRansacThreshold(0.01);
      pose.computePose(vpPose::RANSAC, cMo);

      print_pose(cMo, std::string("Pose estimated by Ransac"));
      fail = compare_pose(pose, cMo_ref, cMo, "pose by Ransac");
      test_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::LAGRANGE_LOWE, cMo);

      print_pose(cMo, std::string("Pose estimated by Lagrange then Lowe"));
      fail = compare_pose(pose, cMo_ref, cMo, "pose by Lagrange then Lowe");
      test_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON_LOWE, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon then Lowe"));
      fail = compare_pose(pose, cMo_ref, cMo, "pose by Dementhon then Lowe");
      test_planar_fail |= fail;

      // Now Virtual Visual servoing
      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, "pose by VVS");
      test_planar_fail |= fail;

      std::cout << "-------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON_VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon then by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, "pose by Dementhon then by VVS");
      test_planar_fail |= fail;

      std::cout << "-------------------------------------------------" << std::endl;
      pose.computePose(vpPose::LAGRANGE_VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated by Lagrange then by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, "pose by Lagrange then by VVS");
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
      std::vector<vpPoint> P(npt); //  Point to be tracked
      P[0].setWorldCoordinates(-L, -L,    0); // Lagrange not accurate...
      P[0].setWorldCoordinates(-L, -L,   -0.02);
      P[1].setWorldCoordinates( L, -L,    0);
      P[2].setWorldCoordinates( L,  L,    0);
      P[3].setWorldCoordinates(-2 * L,    3 * L, 0);
      P[4].setWorldCoordinates(-L,  L,    0.01);
      P[5].setWorldCoordinates( L,  L/2., 0.03);

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
      fail = compare_pose(pose, cMo_ref, cMo, "pose by Lagrange");
      test_non_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon"));
      fail = compare_pose(pose, cMo_ref, cMo, "pose by Dementhon");
      test_non_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.setRansacNbInliersToReachConsensus(4);
      pose.setRansacThreshold(0.01);
      pose.computePose(vpPose::RANSAC, cMo);

      print_pose(cMo, std::string("Pose estimated by Ransac"));
      fail = compare_pose(pose, cMo_ref, cMo, "pose by Ransac");
      test_non_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::LAGRANGE_LOWE, cMo);

      print_pose(cMo, std::string("Pose estimated by Lagrange then Lowe"));
      fail = compare_pose(pose, cMo_ref, cMo, "pose by Lagrange then Lowe");
      test_non_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON_LOWE, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon then Lowe"));
      fail = compare_pose(pose, cMo_ref, cMo, "pose by Dementhon then Lowe");
      test_non_planar_fail |= fail;

      // Now Virtual Visual servoing

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, "pose by VVS");
      test_non_planar_fail |= fail;

      std::cout << "-------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON_VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon then by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, "pose by Dementhon then by VVS");
      test_non_planar_fail |= fail;

      std::cout << "-------------------------------------------------" << std::endl;
      pose.computePose(vpPose::LAGRANGE_VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated by Lagrange then by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, "pose by Lagrange then by VVS");
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
      P[0].setWorldCoordinates(-L2, -L2,  0);
      P[1].setWorldCoordinates( L2, -L2,  0.2);
      P[2].setWorldCoordinates( L2,  L2, -0.1);
      P[3].setWorldCoordinates(-L2,  L2,  0);

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
      fail = compare_pose(pose, cMo_ref, cMo, "pose by Dementhon");
      test_non_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.setRansacNbInliersToReachConsensus(4);
      pose.setRansacThreshold(0.01);
      pose.computePose(vpPose::RANSAC, cMo);

      print_pose(cMo, std::string("Pose estimated by Ransac"));
      fail = compare_pose(pose, cMo_ref, cMo, "pose by Ransac");
      test_non_planar_fail |= fail;

      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON_LOWE, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon then Lowe"));
      fail = compare_pose(pose, cMo_ref, cMo, "pose by Dementhon then Lowe");
      test_non_planar_fail |= fail;

      // Now Virtual Visual servoing
      std::cout << "--------------------------------------------------" << std::endl;
      pose.computePose(vpPose::VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, "pose by VVS");
      test_non_planar_fail |= fail;

      std::cout << "-------------------------------------------------" << std::endl;
      pose.computePose(vpPose::DEMENTHON_VIRTUAL_VS, cMo);

      print_pose(cMo, std::string("Pose estimated by Dementhon then by VVS"));
      fail = compare_pose(pose, cMo_ref, cMo, "pose by Dementhon then by VVS");
      test_non_planar_fail |= fail;

      std::cout << "-------------------------------------------------" << std::endl;
    }

    std::cout << "=======================================================" << std::endl;
    std::cout << "Pose estimation test from planar points: " << (test_planar_fail ? "fail" : "is ok") << std::endl;
    std::cout << "Pose estimation test from non-planar points: " << (test_non_planar_fail ? "fail" : "is ok") << std::endl;
    std::cout << "Global pose estimation test: " << ((test_planar_fail | test_non_planar_fail)  ? "fail" : "is ok") << std::endl;

    return ((test_planar_fail | test_non_planar_fail)  ? EXIT_FAILURE : EXIT_SUCCESS);
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
