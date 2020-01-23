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
  \example testFindMatch.cpp

  Find Matches using Ransac.

*/

int main()
{
  try {
    std::cout << "Find Matches using Ransac" << std::endl;
    std::vector<vpPoint> P;

    P.push_back(vpPoint(-L, -L, 0));
    P.push_back(vpPoint(L, -L, 0));
    P.push_back(vpPoint(L, L, 0));
    P.push_back(vpPoint(-L, L, 0));
    P.push_back(vpPoint(-0, L / 2., L));

    vpHomogeneousMatrix cMo_ref(0, 0.2, 1, vpMath::rad(3), vpMath::rad(-2), vpMath::rad(10));

    std::vector<vpPoint> p(P.size());
    for (unsigned int i = 0; i < P.size(); i++) {
      vpPoint pt = P[i];
      pt.project(cMo_ref);
      p[i].set_x(pt.get_x());
      p[i].set_y(pt.get_y());
    }

    unsigned int ninliers;
    std::vector<vpPoint> inliers;
    double threshold = 1e-6;
    unsigned int nbInlierToReachConsensus = (unsigned int)(P.size());

    vpHomogeneousMatrix cMo;

    vpPose::findMatch(p, P, nbInlierToReachConsensus, threshold, ninliers, inliers, cMo);

    std::cout << "Inliers: " << std::endl;
    for (unsigned int i = 0; i < inliers.size(); i++) {
      inliers[i].print();
      std::cout << std::endl;
    }

    std::cout << "cMo :\n" << vpPoseVector(cMo).t() << std::endl << std::endl;

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

    std::cout << "Matching is " << (test_fail ? "badly" : "well") << " performed" << std::endl;

    return test_fail;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}
