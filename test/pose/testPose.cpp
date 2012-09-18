/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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

#include <visp/vpPose.h>
#include <visp/vpPoint.h>
#include <visp/vpMath.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDebug.h>
#include <visp/vpParseArgv.h>

#include <stdlib.h>
#include <stdio.h>

// List of allowed command line options
#define GETOPTARGS	"h"

#define L 0.035

/*!
  \example testPose.cpp

  Compute the pose of a 3D object using the Dementhon, Lagrange and
  Non-Linear approach.

*/
/*!

  Print the program options.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Compute the pose of a 3D object using the Dementhon, Lagrange and\n\
Non-Linear approach.\n\
\n\
SYNOPSIS\n\
  %s [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -h\n\
     Print the help.\n");

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}
/*!

  Set the program options.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv)
{
  const char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'h': usage(argv[0], NULL); return false; break;

    default:
      usage(argv[0], optarg);
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}

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
int compare_pose(const vpPose &pose, const vpHomogeneousMatrix &cMo_ref, const vpHomogeneousMatrix &cMo_est, const std::string &legend)
{
  vpPoseVector pose_ref = vpPoseVector(cMo_ref);
  vpPoseVector pose_est = vpPoseVector(cMo_est);

  int fail = 0;

  // Test done on the 3D pose
  for(int i=0; i<6; i++) {
    if (std::fabs(pose_ref[i]-pose_est[i]) > 0.001)
      fail = 1;
  }
  std::cout << "Based on 3D parameters " << legend << " is " << (fail ? "badly" : "well") << " estimated" << std::endl;

  // Test done on the residual
  double r = pose.computeResidual(cMo_est);
  r = sqrt(r)/pose.listP.size();
  //std::cout << "Residual on each point (meter): " << r << std::endl;
  fail = (r > 0.1) ? 1 : 0;
  std::cout << "Based on 2D residual (" << r << ") " << legend << " is " << (fail ? "badly" : "well") << " estimated" << std::endl;
  return fail;
}

int
main(int argc, const char ** argv)
{
  // Read the command line options
  if (getOptions(argc, argv) == false) {
    exit (-1);
  }

  vpPoint P[5]  ;  //  Point to be tracked
  vpPose pose ;
  pose.clearPoint() ;

  P[0].setWorldCoordinates(-L,-L, 0 ) ;
  P[1].setWorldCoordinates(L,-L, 0 ) ;
  P[2].setWorldCoordinates(L,L, 0 ) ;
  P[3].setWorldCoordinates(-2*L, 3*L, 0 ) ;
  P[4].setWorldCoordinates(-L,L, 0.01 ) ;
  //P[3].setWorldCoordinates(-L,L, 0 ) ;

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
