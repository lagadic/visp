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
 * Compute the pose of a 3D object using the Dementhon method. Assuming that
 * the correspondance between 2D points and 3D points is not done, we use
 * the RANSAC algorithm to achieve this task
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPoint.h>
#include <visp3/vision/vpPose.h>

#include <stdio.h>
#include <stdlib.h>

#define L 0.1

/*!
  \example testPoseRansac.cpp

  Compute the pose of a 3D object using the Ransac method.

*/

int main()
{
  try {
    std::cout << "Pose computation with matched points" << std::endl;
    std::vector<vpPoint> P; //  Point to be tracked

    P.push_back(vpPoint(-L, -L, 0));
    P.push_back(vpPoint(L, -L, 0));
    P.push_back(vpPoint(L, L, 0));
    P.push_back(vpPoint(-L, L, 0));

    double L2 = L * 3.0;
    P.push_back(vpPoint(0, -L2, 0));
    P.push_back(vpPoint(L2, 0, 0));
    P.push_back(vpPoint(0, L2, 0));
    P.push_back(vpPoint(-L2, 0, 0));

    vpHomogeneousMatrix cMo_ref(0, 0.2, 1, 0, 0, 0);
    for (size_t i = 0; i < P.size(); i++) {
      P[i].project(cMo_ref);
      P[i].print();
      std::cout << std::endl;
    }

    // Introduce an error
    double error = 0.01;
    P[3].set_y(P[3].get_y() + 2 * error);
    P[6].set_x(P[6].get_x() + error);

    vpPose pose;
    for (size_t i = 0; i < P.size(); i++)
      pose.addPoint(P[i]);

    unsigned int nbInlierToReachConsensus = (unsigned int)(75.0 * (double)(P.size()) / 100.0);
    double threshold = 0.001;

    pose.setRansacNbInliersToReachConsensus(nbInlierToReachConsensus);
    pose.setRansacThreshold(threshold);

    vpHomogeneousMatrix cMo;
    // vpPose::ransac(lp,lP, 5, 1e-6, ninliers, lPi, cMo) ;
    pose.computePose(vpPose::RANSAC, cMo);

    std::vector<vpPoint> inliers = pose.getRansacInliers();

    std::cout << "Inliers: " << std::endl;
    for (unsigned int i = 0; i < inliers.size(); i++) {
      inliers[i].print();
      std::cout << std::endl;
    }

    vpPoseVector pose_ref = vpPoseVector(cMo_ref);
    vpPoseVector pose_est = vpPoseVector(cMo);

    std::cout << std::endl;
    std::cout << "reference cMo :\n" << pose_ref.t() << std::endl << std::endl;
    std::cout << "estimated cMo :\n" << pose_est.t() << std::endl << std::endl;

    int test_fail = 0;
    for (unsigned int i = 0; i < 6; i++) {
      if (std::fabs(pose_ref[i] - pose_est[i]) > 0.001)
        test_fail = 1;
    }

    std::cout << "Pose is " << (test_fail ? "badly" : "well") << " estimated" << std::endl;
    return test_fail;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}
