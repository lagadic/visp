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
 * Simulation of a 3D visual servoing.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \example servoSimu3D_cdMc_CamVelocity.cpp

  Simulation of a 3D visual servoing where the current visual feature
  is given by \f$s=({^{c^*}}{\bf t}_c, \; \theta u_{{^{c^*}}{\bf
  R}_c})\f$ and the desired one \f$s^*=(0, \; 0)\f$.

  The control law is set as:
  - an eye-in-hand control law,
  - where velocities are computed in the camera frame.

  To compute the camera velocities, we use here the vpServo class.

  This example is to make into relation with
  servoSimu3D_cMcd_CamVelocity.cpp where the current visual feature is
  \f$s=({^{c}}{\bf t}_{c^*}, \; \theta u_{{^{c}}{\bf
  R}_{c^*}})\f$.

*/

#include <stdio.h>
#include <stdlib.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
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
Simulation of a 3D visual servoing:\n\
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

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
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

    // Log file creation in /tmp/$USERNAME/log.dat
    // This file contains by line:
    // - the 6 computed camera velocities (m/s, rad/s) to achieve the task
    // - the 6 values of s - s*
    std::string username;
    // Get the user login name
    vpIoTools::getUserName(username);

    // Create a log filename to save velocities...
    std::string logdirname;
#if defined(_WIN32)
    logdirname = "C:/temp/" + username;
#else
    logdirname = "/tmp/" + username;
#endif
    // Test if the output path exist. If no try to create it
    if (vpIoTools::checkDirectory(logdirname) == false) {
      try {
        // Create the dirname
        vpIoTools::makeDirectory(logdirname);
      } catch (...) {
        std::cerr << std::endl << "ERROR:" << std::endl;
        std::cerr << "  Cannot create " << logdirname << std::endl;
        exit(-1);
      }
    }
    std::string logfilename;
    logfilename = logdirname + "/log.dat";

    // Open the log file name
    std::ofstream flog(logfilename.c_str());

    vpServo task;
    vpSimulatorCamera robot;

    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << " Test program for vpServo " << std::endl;
    std::cout << " Eye-in-hand task control, velocity computed in the camera frame" << std::endl;
    std::cout << " Simulation " << std::endl;
    std::cout << " task :  3D visual servoing " << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << std::endl;

    // Sets the initial camera location
    vpPoseVector c_r_o( // Translation tx,ty,tz
        0.1, 0.2, 2,
        // ThetaU rotation
        vpMath::rad(20), vpMath::rad(10), vpMath::rad(50));

    // From the camera pose build the corresponding homogeneous matrix
    vpHomogeneousMatrix cMo(c_r_o);

    // Set the robot initial position
    vpHomogeneousMatrix wMc, wMo;
    robot.getPosition(wMc);
    wMo = wMc * cMo; // Compute the position of the object in the world frame

    // Sets the desired camera location
    vpPoseVector cd_r_o( // Translation tx,ty,tz
        0, 0, 1,
        // ThetaU rotation
        vpMath::rad(0), vpMath::rad(0), vpMath::rad(0));
    // From the camera desired pose build the corresponding homogeneous matrix
    vpHomogeneousMatrix cdMo(cd_r_o);

    // Compute the homogeneous transformation from the desired camera position
    // to the initial one
    vpHomogeneousMatrix cdMc;
    cdMc = cdMo * cMo.inverse();

    // Build the current visual features s = (c*tc, thetaU_c*Rc)^T
    vpFeatureTranslation t(vpFeatureTranslation::cdMc);
    vpFeatureThetaU tu(vpFeatureThetaU::cdRc); // current feature
    t.buildFrom(cdMc);
    tu.buildFrom(cdMc);

    // Sets the desired rotation (always zero !)  since s is the
    // rotation that the camera has to achieve. Here s* = (0, 0)^T
    vpFeatureTranslation td(vpFeatureTranslation::cdMc);
    vpFeatureThetaU tud(vpFeatureThetaU::cdRc); // desired feature

    // Define the task
    // - we want an eye-in-hand control law
    // - the robot is controlled in the camera frame
    task.setServo(vpServo::EYEINHAND_CAMERA);
    // - we use here the interaction matrix computed with the
    //   current features
    task.setInteractionMatrixType(vpServo::CURRENT);

    // Add the current and desired visual features
    task.addFeature(t, td);   // 3D translation
    task.addFeature(tu, tud); // 3D rotation

    // - set the constant gain to 1.0
    task.setLambda(1);

    // Display task information
    task.print();

    unsigned int iter = 0;
    // Start the visual servoing loop. We stop the servo after 200 iterations
    while (iter++ < 200) {
      std::cout << "-----------------------------------" << iter << std::endl;
      vpColVector v;

      // get the robot position
      robot.getPosition(wMc);
      // Compute the position of the camera wrt the object frame
      cMo = wMc.inverse() * wMo;

      // new displacement to achieve
      cdMc = cdMo * cMo.inverse();

      // Update the current visual features
      t.buildFrom(cdMc);
      tu.buildFrom(cdMc);

      // Compute the control law
      v = task.computeControlLaw();

      // Display task information
      if (iter == 1)
        task.print();

      // Send the camera velocity to the controller
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

      // Retrieve the error
      std::cout << "|| s - s* || = " << (task.getError()).sumSquare() << std::endl;

      // Save log
      flog << v.t() << " " << (task.getError()).t() << std::endl;
    }
    // Display task information
    task.print();

    // Kill the task
    task.kill();

    // Close the log file
    flog.close();
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch a ViSP exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
