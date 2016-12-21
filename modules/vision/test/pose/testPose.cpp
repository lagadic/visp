/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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

#include <visp3/vision/vpPose.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/core/vpRxyzVector.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpDebug.h>

#include <stdlib.h>
#include <stdio.h>

#define L 0.035

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

  std::cout << std::endl << legend << "\n "
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
  for(unsigned int i=0; i<6; i++) {
    if (std::fabs(pose_ref[i]-pose_est[i]) > 0.001)
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
  r = sqrt(r)/pose.listP.size();
  //std::cout << "Residual on each point (meter): " << r << std::endl;
  fail = (r > 0.1) ? 1 : 0;
  std::cout << "Based on 2D residual (" << r << ") " << legend << " is " << (fail ? "badly" : "well") << " estimated" << std::endl;
  return fail;
}

int main()
{
  try {
    vpPoint P[5]  ;  //  Point to be tracked
    vpPose pose ;
    pose.clearPoint() ;

    P[0].setWorldCoordinates(-L,-L, 0 ) ;
    P[1].setWorldCoordinates(L,-L, 0 ) ;
    P[2].setWorldCoordinates(L,L, 0 ) ;
    P[3].setWorldCoordinates(-2*L, 3*L, 0 ) ;
    P[4].setWorldCoordinates(-L,L, 0.01 ) ;

    int test_fail = 0, fail = 0;
    vpPoseVector cpo_ref = vpPoseVector(0.01, 0.02, 0.25, vpMath::rad(5), 0,vpMath::rad(10));
    vpHomogeneousMatrix cMo_ref(cpo_ref) ;
    vpHomogeneousMatrix cMo ; // will contain the estimated pose

    for(int i=0 ; i < 5 ; i++) {
      P[i].project(cMo_ref) ;
      //P[i].print();
      pose.addPoint(P[i]) ; // and added to the pose computation class
    }

    // Let's go ...
    print_pose(cMo_ref, std::string("Reference pose"));  // print the reference pose

    std::cout <<"-------------------------------------------------"<<std::endl ;
    pose.computePose(vpPose::LAGRANGE, cMo) ;

    print_pose(cMo, std::string("Pose estimated by Lagrange"));
    fail = compare_pose(pose, cMo_ref, cMo, "pose by Lagrange");
    test_fail |= fail;

    std::cout <<"--------------------------------------------------"<<std::endl ;
    pose.computePose(vpPose::DEMENTHON, cMo) ;

    print_pose(cMo, std::string("Pose estimated by Dementhon"));
    fail = compare_pose(pose, cMo_ref, cMo, "pose by Dementhon");
    test_fail |= fail;

    std::cout <<"--------------------------------------------------"<<std::endl ;
    pose.setRansacNbInliersToReachConsensus(4);
    pose.setRansacThreshold(0.01);
    pose.computePose(vpPose::RANSAC, cMo) ;

    print_pose(cMo, std::string("Pose estimated by Ransac"));
    fail = compare_pose(pose, cMo_ref, cMo, "pose by Ransac");
    test_fail |= fail;

    std::cout <<"--------------------------------------------------"<<std::endl ;
    pose.computePose(vpPose::LAGRANGE_LOWE, cMo) ;

    print_pose(cMo, std::string("Pose estimated by Lagrange than Lowe"));
    fail = compare_pose(pose, cMo_ref, cMo, "pose by Lagrange than Lowe");
    test_fail |= fail;

    std::cout <<"--------------------------------------------------"<<std::endl ;
    pose.computePose(vpPose::DEMENTHON_LOWE, cMo) ;

    print_pose(cMo, std::string("Pose estimated by Dementhon than Lowe"));
    fail = compare_pose(pose, cMo_ref, cMo, "pose by Dementhon than Lowe");
    test_fail |= fail;

    // Now Virtual Visual servoing

    std::cout <<"--------------------------------------------------"<<std::endl ;
    pose.computePose(vpPose::VIRTUAL_VS, cMo) ;

    print_pose(cMo, std::string("Pose estimated by VVS"));
    fail = compare_pose(pose, cMo_ref, cMo, "pose by VVS");
    test_fail |= fail;

    std::cout <<"-------------------------------------------------"<<std::endl ;
    pose.computePose(vpPose::DEMENTHON_VIRTUAL_VS, cMo) ;

    print_pose(cMo, std::string("Pose estimated by Dementhon than by VVS"));
    fail = compare_pose(pose, cMo_ref, cMo, "pose by Dementhon than by VVS");
    test_fail |= fail;

    std::cout <<"-------------------------------------------------"<<std::endl ;
    pose.computePose(vpPose::LAGRANGE_VIRTUAL_VS, cMo) ;

    print_pose(cMo, std::string("Pose estimated by Lagrange than by VVS"));
    fail = compare_pose(pose, cMo_ref, cMo, "pose by Lagrange than by VVS");
    test_fail |= fail;

    std::cout << "\nGlobal pose estimation test " << (test_fail ? "fail" : "is ok") << std::endl;

    return test_fail;
  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}
