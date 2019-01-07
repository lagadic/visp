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
 * Simulation of a 2 1/2 D visual servoing.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example servoSimuPoint2DhalfCamVelocity1.cpp
  Simulation of a 2 1/2 D visual servoing (theta U):
  - (x,y,Z,theta U) features,
  - eye-in-hand control law,
  - velocity computed in the camera frame,
  - no display.

*/

#include <stdio.h>
#include <stdlib.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPoint.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpGenericFeature.h>
#include <visp3/vs/vpServo.h>

// List of allowed command line options
#define GETOPTARGS "h"

void usage(const char *name, const char *badparam);
bool getOptions(int argc, const char **argv);

/*!

Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Simulation of a 2 1/2 D visual servoing (x,y,Z,theta U):\n\
- eye-in-hand control law,\n\
- velocity computed in the camera frame,\n\
- without display.\n\
          \n\
SYNOPSIS\n\
  %s [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
                  \n\
  -h\n\
     Print the help.\n");

  if (badparam) {
    fprintf(stderr, "ERROR: \n");
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);
  }
}

/*!

Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'h':
      usage(argv[0], NULL);
      return false;
      break;

    default:
      usage(argv[0], optarg_);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

int main(int argc, const char **argv)
{
  try {
    // Read the command line options
    if (getOptions(argc, argv) == false) {
      exit(-1);
    }

    vpServo task;
    vpSimulatorCamera robot;

    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << " Test program for vpServo " << std::endl;
    std::cout << " task :  2 1/2 D visual servoing " << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << std::endl;

    // sets the initial camera location
    vpPoseVector c_r_o(0.1, 0.2, 2, vpMath::rad(20), vpMath::rad(10), vpMath::rad(50));

    vpHomogeneousMatrix cMo(c_r_o);
    // Compute the position of the object in the world frame
    vpHomogeneousMatrix wMc, wMo;
    robot.getPosition(wMc);
    wMo = wMc * cMo;

    // sets the desired camera location
    vpPoseVector cd_r_o(0, 0, 1, vpMath::rad(0), vpMath::rad(0), vpMath::rad(0));
    vpHomogeneousMatrix cdMo(cd_r_o);

    // sets the point coordinates in the world frame
    vpPoint point(0, 0, 0);
    // computes the point coordinates in the camera frame and its 2D
    // coordinates
    point.track(cMo);

    vpPoint pointd(0, 0, 0);
    pointd.track(cdMo);
    //------------------------------------------------------------------
    // 1st feature (x,y)
    // want to it at (0,0)
    vpFeaturePoint p;
    vpFeatureBuilder::create(p, point);

    vpFeaturePoint pd;
    vpFeatureBuilder::create(pd, pointd);

    //------------------------------------------------------------------
    // 2nd feature (Z)
    // not necessary to project twice (reuse p)
    vpFeaturePoint3D Z;
    vpFeatureBuilder::create(Z, point); // retrieve x,y and Z of the vpPoint structure

    // want to see it one meter away (here again use pd)
    vpFeaturePoint3D Zd;
    vpFeatureBuilder::create(Zd, pointd); // retrieve x,y and Z of the vpPoint structure

    //------------------------------------------------------------------
    // 3rd feature ThetaU
    // compute the rotation that the camera has to achieve
    vpHomogeneousMatrix cdMc;
    cdMc = cdMo * cMo.inverse();

    vpFeatureThetaU tu(vpFeatureThetaU::cdRc);
    tu.buildFrom(cdMc);

    // sets the desired rotation (always zero !)
    // since s is the rotation that the camera has to achieve

    //------------------------------------------------------------------
    // define the task
    // - we want an eye-in-hand control law
    // - robot is controlled in the camera frame
    task.setServo(vpServo::EYEINHAND_CAMERA);

    task.addFeature(p, pd);
    task.addFeature(Z, Zd, vpFeaturePoint3D::selectZ());
    task.addFeature(tu);

    // set the gain
    task.setLambda(1);

    // Display task information
    task.print();

    unsigned int iter = 0;
    // loop
    while (iter++ < 200) {
      std::cout << "---------------------------------------------" << iter << std::endl;
      vpColVector v;

      // get the robot position
      robot.getPosition(wMc);
      // Compute the position of the camera wrt the object frame
      cMo = wMc.inverse() * wMo;

      // update the feature
      point.track(cMo);
      vpFeatureBuilder::create(p, point);
      vpFeatureBuilder::create(Z, point);

      cdMc = cdMo * cMo.inverse();
      tu.buildFrom(cdMc);

      // compute the control law
      v = task.computeControlLaw();
      // send the camera velocity to the controller ") ;
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

      std::cout << "|| s - s* || = " << (task.getError()).sumSquare() << std::endl;
    }

    // Display task information
    task.print();
    task.kill();
    std::cout << "Final camera location:\n " << cMo << std::endl;
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch a ViSP exception: " << e << std::endl;
    return EXIT_SUCCESS;
  }
}
