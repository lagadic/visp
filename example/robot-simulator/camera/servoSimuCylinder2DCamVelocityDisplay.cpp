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
 * Simulation of a 2D visual servoing on a cylinder.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example servoSimuCylinder2DCamVelocityDisplay.cpp
  \brief Servo a cylinder:
  Servo a cylinder:
  - eye-in-hand control law,
  - velocity computed in the camera frame,
  - display the camera view.

*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>

#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))

#include <stdio.h>
#include <stdlib.h>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpCylinder.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpMath.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureLine.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

// List of allowed command line options
#define GETOPTARGS "cdh"

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
Simulation of a 2D visual servoing on a cylinder:\n\
- eye-in-hand control law,\n\
- velocity computed in the camera frame,\n\
- display the camera view.\n\
          \n\
SYNOPSIS\n\
  %s [-c] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
                  \n\
  -c\n\
     Disable the mouse click. Useful to automaze the \n\
     execution of this program without humain intervention.\n\
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
  \param click_allowed : false if mouse click is not allowed.
  \param display : false if the display is to turn off.
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
    bool opt_display = true;
    bool opt_click_allowed = true;

    // Read the command line options
    if (getOptions(argc, argv, opt_click_allowed, opt_display) == false) {
      exit(-1);
    }

    vpImage<unsigned char> I(512, 512, 255);

// We open a window using either X11, GTK or GDI.
#if defined VISP_HAVE_X11
    vpDisplayX display;
#elif defined VISP_HAVE_GTK
    vpDisplayGTK display;
#elif defined VISP_HAVE_GDI
    vpDisplayGDI display;
#elif defined VISP_HAVE_OPENCV
    vpDisplayOpenCV display;
#endif

    if (opt_display) {
      try {
        // Display size is automatically defined by the image (I) size
        display.init(I, 100, 100, "Camera view...");
        // Display the image
        // The image class has a member that specify a pointer toward
        // the display that has been initialized in the display declaration
        // therefore is is no longuer necessary to make a reference to the
        // display variable.
        vpDisplay::display(I);
        vpDisplay::flush(I);
      } catch (...) {
        vpERROR_TRACE("Error while displaying the image");
        exit(-1);
      }
    }

    double px, py;
    px = py = 600;
    double u0, v0;
    u0 = v0 = 256;

    vpCameraParameters cam(px, py, u0, v0);

    vpServo task;
    vpSimulatorCamera robot;

    // sets the initial camera location
    vpHomogeneousMatrix cMo(-0.2, 0.1, 2, vpMath::rad(5), vpMath::rad(5), vpMath::rad(20));

    vpHomogeneousMatrix wMc, wMo;
    robot.getPosition(wMc);
    wMo = wMc * cMo; // Compute the position of the object in the world frame

    // sets the final camera location (for simulation purpose)
    vpHomogeneousMatrix cMod(0, 0, 1, vpMath::rad(-60), vpMath::rad(0), vpMath::rad(0));

    // sets the cylinder coordinates in the world frame
    vpCylinder cylinder(0, 1, 0, // direction
                        0, 0, 0, // point of the axis
                        0.1);    // radius

    // sets the desired position of the visual feature
    cylinder.track(cMod);
    cylinder.print();

    vpFeatureLine ld[2];
    int i;
    for (i = 0; i < 2; i++)
      vpFeatureBuilder::create(ld[i], cylinder, i);

    // computes  the cylinder coordinates in the camera frame and its 2D
    // coordinates sets the current position of the visual feature
    cylinder.track(cMo);
    cylinder.print();

    vpFeatureLine l[2];
    for (i = 0; i < 2; i++) {
      vpFeatureBuilder::create(l[i], cylinder, i);
      l[i].print();
    }

    // define the task
    // - we want an eye-in-hand control law
    // - robot is controlled in the camera frame
    task.setServo(vpServo::EYEINHAND_CAMERA);
    // task.setInteractionMatrixType(vpServo::CURRENT,
    // vpServo::PSEUDO_INVERSE) ;
    //  it can also be interesting to test these possibilities
    //    task.setInteractionMatrixType(vpServo::MEAN,
    //    vpServo::PSEUDO_INVERSE) ;
    task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);
    // task.setInteractionMatrixType(vpServo::DESIRED,  vpServo::TRANSPOSE) ;
    // task.setInteractionMatrixType(vpServo::CURRENT,  vpServo::TRANSPOSE) ;

    // - we want to see 2 lines on 2 lines
    task.addFeature(l[0], ld[0]);
    task.addFeature(l[1], ld[1]);

    vpServoDisplay::display(task, cam, I);
    vpDisplay::flush(I);

    // Display task information
    task.print();

    if (opt_display && opt_click_allowed) {
      std::cout << "\n\nClick in the camera view window to start..." << std::endl;
      vpDisplay::getClick(I);
    }

    // - set the gain
    task.setLambda(1);

    // Display task information
    task.print();

    unsigned int iter = 0;
    // loop
    do {
      std::cout << "---------------------------------------------" << iter++ << std::endl;
      vpColVector v;

      // get the robot position
      robot.getPosition(wMc);
      // Compute the position of the camera wrt the object frame
      cMo = wMc.inverse() * wMo;

      // new line position
      // retrieve x,y and Z of the vpLine structure
      cylinder.track(cMo);
      //    cylinder.print() ;
      for (i = 0; i < 2; i++) {
        vpFeatureBuilder::create(l[i], cylinder, i);
        //   l[i].print() ;
      }

      if (opt_display) {
        vpDisplay::display(I);
        vpServoDisplay::display(task, cam, I);
        vpDisplay::flush(I);
      }

      // compute the control law
      v = task.computeControlLaw();

      // send the camera velocity to the controller
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

      std::cout << "|| s - s* || = " << (task.getError()).sumSquare() << std::endl;
      ;

      //   vpDisplay::getClick(I) ;
    } while ((task.getError()).sumSquare() > 1e-9);

    if (opt_display && opt_click_allowed) {
      std::cout << "\nClick in the camera view window to end..." << std::endl;
      vpDisplay::getClick(I);
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

#else
int main()
{
  std::cout << "You do not have X11, or GTK, or GDI (Graphical Device Interface) functionalities to display images..." << std::endl;
  std::cout << "Tip if you are on a unix-like system:" << std::endl;
  std::cout << "- Install X11, configure again ViSP using cmake and build again this example" << std::endl;
  std::cout << "Tip if you are on a windows-like system:" << std::endl;
  std::cout << "- Install GDI, configure again ViSP using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
}
#endif
