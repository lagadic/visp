/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * Simulation of a 2D visual servoing using 4 points as visual feature.
 *
*****************************************************************************/

/*!
  \example servoSimuFourPoints2DCamVelocityDisplay.cpp

  \brief Simulation of a 2D visual servoing:

  Simulation of a 2D visual servoing:
  - servo on 4 points with cartesian coordinates,
  - eye-in-hand control law,
  - camera velocities are computed,
  - display internal camera view and an external view.

  Interaction matrix is computed as the mean of the current and desired
  interaction matrix.

*/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_DISPLAY) &&       \
    (defined(VISP_HAVE_LAPACK) || defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_OPENCV))

#include <stdio.h>
#include <stdlib.h>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpMath.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/gui/vpProjectionDisplay.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

// List of allowed command line options
#define GETOPTARGS "cdh"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

void usage(const char *name, const char *badparam);
bool getOptions(int argc, const char **argv, bool &click_allowed, bool &display);

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Tests a control law with the following characteristics:\n\
- eye-in-hand control\n\
- articular velocity are computed\n\
- servo on 4 points,\n\
- internal and external camera view displays.\n\
          \n\
SYNOPSIS\n\
  %s [-c] [-d] [-h]\n",
          name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -c\n\
     Disable the mouse click. Useful to automate the \n\
     execution of this program without human intervention.\n\
                  \n\
  -d \n\
     Turn off the display.\n\
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
  \param display : Display activation.
  \param click_allowed : Click activation.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, bool &click_allowed, bool &display)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'c':
      click_allowed = false;
      break;
    case 'd':
      display = false;
      break;
    case 'h':
      usage(argv[0], nullptr);
      return false;

    default:
      usage(argv[0], optarg_);
      return false;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], nullptr);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

int main(int argc, const char **argv)
{
    // We declare the windows variables to be able to free the memory in the catch sections if needed
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> displayInt;
  std::shared_ptr<vpDisplay> displayExt;
#else
  vpDisplay *displayInt = nullptr;
  vpDisplay *displayExt = nullptr;
#endif

  try {
    bool opt_click_allowed = true;
    bool opt_display = true;

    // Read the command line options
    if (getOptions(argc, argv, opt_click_allowed, opt_display) == false) {
      return EXIT_FAILURE;
    }
      // open a display for the visualization

    vpImage<unsigned char> Iint(300, 300, 0);
    vpImage<unsigned char> Iext(300, 300, 0);

    if (opt_display) {
      // We open two displays, one for the internal camera view, the other one for
      // the external view
      // Display size is automatically defined by the image (Iint) and
      // (Iext) size
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
      displayInt = vpDisplayFactory::createDisplay(Iint, 0, 0, "Internal view");
      displayExt = vpDisplayFactory::createDisplay(Iext, 330, 000, "External view");
#else
      displayInt = vpDisplayFactory::allocateDisplay(Iint, 0, 0, "Internal view");
      displayExt = vpDisplayFactory::allocateDisplay(Iext, 330, 000, "External view");
#endif
    }
    vpProjectionDisplay externalview;

    double px = 500, py = 500;
    double u0 = 150, v0 = 160;

    vpCameraParameters cam(px, py, u0, v0);

    vpServo task;
    vpSimulatorCamera robot;

    std::cout << std::endl;
    std::cout << "----------------------------------------------" << std::endl;
    std::cout << " Test program for vpServo " << std::endl;
    std::cout << " Eye-in-hand task control, articular velocity are computed" << std::endl;
    std::cout << " Simulation " << std::endl;
    std::cout << " task : servo 4 points " << std::endl;
    std::cout << "----------------------------------------------" << std::endl;
    std::cout << std::endl;

    // sets the initial camera location
    vpHomogeneousMatrix cMo(-0.1, -0.1, 1, vpMath::rad(40), vpMath::rad(10), vpMath::rad(60));

    // Compute the position of the object in the world frame
    vpHomogeneousMatrix wMc, wMo;
    robot.getPosition(wMc);
    wMo = wMc * cMo;

    vpHomogeneousMatrix cextMo(0, 0, 2, 0, 0, 0); // vpMath::rad(40),  vpMath::rad(10),  vpMath::rad(60));

    // sets the point coordinates in the object frame
    vpPoint point[4];
    point[0].setWorldCoordinates(-0.1, -0.1, 0);
    point[1].setWorldCoordinates(0.1, -0.1, 0);
    point[2].setWorldCoordinates(0.1, 0.1, 0);
    point[3].setWorldCoordinates(-0.1, 0.1, 0);

    for (unsigned i = 0; i < 4; i++)
      externalview.insert(point[i]);

    // computes  the point coordinates in the camera frame and its 2D
    // coordinates
    for (unsigned i = 0; i < 4; i++)
      point[i].track(cMo);

    // sets the desired position of the point
    vpFeaturePoint p[4];
    for (unsigned i = 0; i < 4; i++)
      vpFeatureBuilder::create(p[i], point[i]); // retrieve x,y and Z of the vpPoint structure

    // sets the desired position of the feature point s*
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

    // Set the position of the end-effector frame in the camera frame as identity
    vpHomogeneousMatrix cMe;
    vpVelocityTwistMatrix cVe(cMe);
    task.set_cVe(cVe);

    // Set the Jacobian (expressed in the end-effector frame
    vpMatrix eJe;
    robot.get_eJe(eJe);
    task.set_eJe(eJe);

    // we want to see a point on a point
    for (unsigned i = 0; i < 4; i++)
      task.addFeature(p[i], pd[i]);

    // set the gain
    task.setLambda(1);

    // Display task information
    task.print();

    unsigned int iter = 0;
    // loop
    while (iter++ < 200) {
      std::cout << "---------------------------------------------" << iter << std::endl;
      vpColVector v;

      // Set the Jacobian (expressed in the end-effector frame)
      // since q is modified eJe is modified
      robot.get_eJe(eJe);
      task.set_eJe(eJe);

      // get the robot position
      robot.getPosition(wMc);
      // Compute the position of the object frame in the camera frame
      cMo = wMc.inverse() * wMo;

      // update new point position and corresponding features
      for (unsigned i = 0; i < 4; i++) {
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

      if (opt_display) {
        vpDisplay::display(Iint);
        vpDisplay::display(Iext);
        vpServoDisplay::display(task, cam, Iint);
        externalview.display(Iext, cextMo, cMo, cam, vpColor::green);
        vpDisplay::flush(Iint);
        vpDisplay::flush(Iext);
      }

      // compute the control law
      v = task.computeControlLaw();

      // send the camera velocity to the controller
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

      std::cout << "|| s - s* || = " << (task.getError()).sumSquare() << std::endl;
    }

    // Display task information
    task.print();

    std::cout << "Final robot position with respect to the object frame:\n";
    cMo.print();

    if (opt_display && opt_click_allowed) {
      vpDisplay::displayText(Iint, 20, 20, "Click to quit...", vpColor::white);
      vpDisplay::flush(Iint);
      vpDisplay::getClick(Iint);
    }
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
    if (displayInt != nullptr) {
      delete displayInt;
    }
    if (displayExt != nullptr) {
      delete displayExt;
    }
#endif
    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Catch a ViSP exception: " << e << std::endl;
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
    if (displayInt != nullptr) {
      delete displayInt;
    }
    if (displayExt != nullptr) {
      delete displayExt;
    }
#endif
    return EXIT_FAILURE;
  }
}
#elif !(defined(VISP_HAVE_LAPACK) || defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_OPENCV))
int main()
{
  std::cout << "Cannot run this example: install Lapack, Eigen3 or OpenCV" << std::endl;
  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cout << "You do not have X11, or GTK, or GDI (Graphical Device Interface) functionalities to display images..."
    << std::endl;
  std::cout << "Tip if you are on a unix-like system:" << std::endl;
  std::cout << "- Install X11, configure again ViSP using cmake and build again this example" << std::endl;
  std::cout << "Tip if you are on a windows-like system:" << std::endl;
  std::cout << "- Install GDI, configure again ViSP using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
}
#endif
