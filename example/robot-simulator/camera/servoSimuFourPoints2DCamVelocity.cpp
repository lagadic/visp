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
 * Simulation of a 2D visual servoing using 4 points as visual feature.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example servoSimuFourPoints2DCamVelocity.cpp
  \brief Simulation of a 2D visual servoing:
  Simulation of a 2D visual servoing:
  - servo on 4 points,
  - eye-in-hand control law,
  - camera velocity are computed,
  - no display.

  Interaction matrix is computed as the mean of the current and desired
  interaction matrix.

*/

#include <stdio.h>
#include <stdlib.h>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
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
Simulation of a 2D visual servoing:\n\
- servo on 4 points,\n\
- eye-in-hand control law,\n\
- articular velocity are computed,\n\
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

    int i;
    vpServo task;
    vpSimulatorCamera robot;

    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << " Test program for vpServo " << std::endl;
    std::cout << " Eye-in-hand task control,  articular velocity are computed" << std::endl;
    std::cout << " Simulation " << std::endl;
    std::cout << " task : servo 4 points " << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << std::endl;

    // sets the initial camera location with respect to the object
    vpHomogeneousMatrix cMo;
    cMo[0][3] = 0.1;
    cMo[1][3] = 0.2;
    cMo[2][3] = 2;

    // Compute the position of the object in the world frame
    vpHomogeneousMatrix wMc, wMo;
    robot.getPosition(wMc);
    wMo = wMc * cMo;

    // sets the point coordinates in the object frame
    vpPoint point[4];
    point[0].setWorldCoordinates(-1, -1, 0);
    point[1].setWorldCoordinates(1, -1, 0);
    point[2].setWorldCoordinates(1, 1, 0);
    point[3].setWorldCoordinates(-1, 1, 0);

    // computes  the point coordinates in the camera frame and its 2D
    // coordinates
    for (i = 0; i < 4; i++)
      point[i].track(cMo);

    // sets the desired position of the point
    vpFeaturePoint p[4];
    for (i = 0; i < 4; i++)
      vpFeatureBuilder::create(p[i], point[i]); // retrieve x,y and Z of the vpPoint structure

    // sets the desired position of the point
    vpFeaturePoint pd[4];

    pd[0].buildFrom(-0.1, -0.1, 1);
    pd[1].buildFrom(0.1, -0.1, 1);
    pd[2].buildFrom(0.1, 0.1, 1);
    pd[3].buildFrom(-0.1, 0.1, 1);

    // define the task
    // - we want an eye-in-hand control law
    // - articular velocity are computed
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    task.setInteractionMatrixType(vpServo::MEAN);

    // Set the position of the camera in the end-effector frame
    vpHomogeneousMatrix cMe;
    vpVelocityTwistMatrix cVe(cMe);
    task.set_cVe(cVe);

    // Set the Jacobian (expressed in the end-effector frame)
    vpMatrix eJe;
    robot.get_eJe(eJe);
    task.set_eJe(eJe);

    // we want to see a point on a point
    for (i = 0; i < 4; i++)
      task.addFeature(p[i], pd[i]);

    // set the gain
    task.setLambda(1);

    // Display task information
    task.print();

    unsigned int iter = 0;
    // loop
    while (iter++ < 1500) {
      std::cout << "---------------------------------------------" << iter << std::endl;
      vpColVector v;

      // Set the Jacobian (expressed in the end-effector frame)
      // since q is modified eJe is modified
      robot.get_eJe(eJe);
      task.set_eJe(eJe);

      // get the robot position
      robot.getPosition(wMc);
      // Compute the position of the camera wrt the object frame
      cMo = wMc.inverse() * wMo;

      // update new point position and corresponding features
      for (i = 0; i < 4; i++) {
        point[i].track(cMo);
        // retrieve x,y and Z of the vpPoint structure
        vpFeatureBuilder::create(p[i], point[i]);
      }
      // since vpServo::MEAN interaction matrix is used, we need also to
      // update the desired features at each iteration
      pd[0].buildFrom(-0.1, -0.1, 1);
      pd[1].buildFrom(0.1, -0.1, 1);
      pd[2].buildFrom(0.1, 0.1, 1);
      pd[3].buildFrom(-0.1, 0.1, 1);

      // compute the control law ") ;
      v = task.computeControlLaw();

      // send the camera velocity to the controller ") ;
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

      std::cout << "|| s - s* || = " << (task.getError()).sumSquare() << std::endl;
    }

    // Display task information
    task.print();
    task.kill();
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch a ViSP exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
