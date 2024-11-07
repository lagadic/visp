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
 * Simulation of a 2D visual servoing on a sphere.
 *
*****************************************************************************/

/*!
  \example servoSimuSphere2DCamVelocityDisplaySecondaryTask.cpp
  Servo a sphere:
  - eye-in-hand control law,
  - velocity computed in the camera frame,
  - display the camera view,
  - a secondary task is the added.

*/

#include <stdio.h>
#include <stdlib.h>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpSphere.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpProjectionDisplay.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureEllipse.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

// List of allowed command line options
#define GETOPTARGS "cdho"

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
Simulation of a 2D visual servoing on a sphere:\n\
- eye-in-hand control law,\n\
- velocity computed in the camera frame,\n\
- display the camera view,\n\
- a secondary task is the added.\n\
          \n\
SYNOPSIS\n\
  %s [-c] [-d] [-o] [-h]\n",
          name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
   \n\
  -c\n\
     Disable the mouse click. Useful to automate the \n\
     execution of this program without human intervention.\n\
                                  \n\
  -d \n\
     Turn off the display.\n\
     \n\
  -o \n\
     Disable new projection operator usage for secondary task.\n\
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
  \param new_proj_operator : If true, use new projection operator for secondary task.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, bool &click_allowed, bool &display, bool &new_proj_operator)
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
    case 'o':
      new_proj_operator = false;
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
#if (defined(VISP_HAVE_LAPACK) || defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_OPENCV))
  try {
    bool opt_display = true;
    bool opt_click_allowed = true;
    bool opt_new_proj_operator = true;

    // Read the command line options
    if (getOptions(argc, argv, opt_click_allowed, opt_display, opt_new_proj_operator) == false) {
      return (EXIT_FAILURE);
    }

    vpImage<unsigned char> I(512, 512, 0);
    vpImage<unsigned char> Iext(512, 512, 0);

    // We open a window if a display is available
#ifdef VISP_HAVE_DISPLAY
#if defined(VISP_HAVE_X11)
    vpDisplayX displayI;
    vpDisplayX displayExt;
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK displayI;
    vpDisplayGTK displayExt;
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI displayI;
    vpDisplayGDI displayExt;
#elif defined(HAVE_OPENCV_HIGHGUI)
    vpDisplayOpenCV displayI;
    vpDisplayOpenCV displayExt;
#elif defined(VISP_HAVE_D3D9)
    vpDisplayD3D displayI;
    vpDisplayD3D displayExt;
#endif
#endif

    if (opt_display) {
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV)
      // Display size is automatically defined by the image (I) size
      displayI.init(I, 100, 100, "Camera view...");
      displayExt.init(Iext, 130 + static_cast<int>(I.getWidth()), 100, "External view");
#endif
      // Display the image
      // The image class has a member that specify a pointer toward
      // the display that has been initialized in the display declaration
      // therefore is is no longer necessary to make a reference to the
      // display variable.
      vpDisplay::display(I);
      vpDisplay::display(Iext);
      vpDisplay::flush(I);
      vpDisplay::flush(Iext);
    }

#ifdef VISP_HAVE_DISPLAY
    vpProjectionDisplay externalview;
#endif

    double px = 600, py = 600;
    double u0 = I.getWidth() / 2., v0 = I.getHeight() / 2.;

    vpCameraParameters cam(px, py, u0, v0);

    vpServo task;
    vpSimulatorCamera robot;

    // sets the initial camera location
    vpHomogeneousMatrix cMo;
    cMo[0][3] = 0.1;
    cMo[1][3] = 0.2;
    cMo[2][3] = 2;
    // Compute the position of the object in the world frame
    vpHomogeneousMatrix wMc, wMo;
    robot.getPosition(wMc);
    wMo = wMc * cMo;

    vpHomogeneousMatrix cMod;
    cMod[0][3] = 0;
    cMod[1][3] = 0;
    cMod[2][3] = 1;

    // sets the sphere coordinates in the world frame
    vpSphere sphere;
    sphere.setWorldCoordinates(0, 0, 0, 0.1);

#ifdef VISP_HAVE_DISPLAY
    externalview.insert(sphere);
#endif
    // sets the desired position of the visual feature
    vpFeatureEllipse pd;
    sphere.track(cMod);
    vpFeatureBuilder::create(pd, sphere);

    // computes  the sphere coordinates in the camera frame and its 2D
    // coordinates sets the current position of the visual feature
    vpFeatureEllipse p;
    sphere.track(cMo);
    vpFeatureBuilder::create(p, sphere);

    // define the task
    // - we want an eye-in-hand control law
    // - robot is controlled in the camera frame
    task.setServo(vpServo::EYEINHAND_CAMERA);

    // we want to see a sphere on a sphere
    std::cout << std::endl;
    task.addFeature(p, pd);

    // set the gain
    task.setLambda(1);

    // Set the point of view of the external view
    vpHomogeneousMatrix cextMo(0, 0, 4, vpMath::rad(40), vpMath::rad(10), vpMath::rad(60));

    // Display the initial scene
    vpServoDisplay::display(task, cam, I);
#ifdef VISP_HAVE_DISPLAY
    externalview.display(Iext, cextMo, cMo, cam, vpColor::red);
#endif
    vpDisplay::flush(I);
    vpDisplay::flush(Iext);

    // Display task information
    task.print();

    if (opt_display && opt_click_allowed) {
      vpDisplay::displayText(I, 20, 20, "Click to start visual servo...", vpColor::white);
      vpDisplay::flush(I);
      vpDisplay::getClick(I);
    }

    unsigned int iter = 0;
    bool stop = false;
    bool start_secondary_task = false;

    // loop
    while (iter++ < 2000 && !stop) {
      std::cout << "---------------------------------------------" << iter << std::endl;

      // get the robot position
      robot.getPosition(wMc);
      // Compute the position of the object frame in the camera frame
      cMo = wMc.inverse() * wMo;

      // new sphere position: retrieve x,y and Z of the vpSphere structure
      sphere.track(cMo);
      vpFeatureBuilder::create(p, sphere);

      if (opt_display) {
        vpDisplay::display(I);
        vpDisplay::display(Iext);
        vpServoDisplay::display(task, cam, I);
#ifdef VISP_HAVE_DISPLAY
        externalview.display(Iext, cextMo, cMo, cam, vpColor::red);
#endif
      }

      // compute the control law
      vpColVector v = task.computeControlLaw();

      // Wait primary task convergence before considering secondary task
      if (task.getError().sumSquare() < 1e-6) {
        start_secondary_task = true;
      }

      if (start_secondary_task) {
        // Only 3 dof are required to achieve primary task: vz, wx, wy
        // It remains 3 free dof (vx, vy, wz) that could be used in a secondary task for example to move around the
        // sphere
        vpColVector de2dt(6);
        de2dt[0] = 0.50;            // vx = 0.50 m/s should also generate a motion on wy = (I-WpW)de2dt[4]
        de2dt[1] = 0.25;            // vy = 0.25 m/s should generate a motion on wx = (I-WpW)de2dt[3]
        de2dt[2] = 1;               // vz = 1 m/s should be zero in vz = (I-WpW)de2dt[2]
        de2dt[5] = vpMath::rad(10); // wz = 10 rad/s should generate a motion on (I-WpW)de2dt[5]

        std::cout << "de2dt :" << de2dt.t() << std::endl;
        vpColVector sec = task.secondaryTask(de2dt, opt_new_proj_operator);
        std::cout << "(I-WpW)de2dt :" << sec.t() << std::endl;

        v += sec;

        if (opt_display && opt_click_allowed) {
          std::stringstream ss;
          ss << std::string("New projection operator: ") +
            (opt_new_proj_operator ? std::string("yes (use option -o to use old one)") : std::string("no"));
          vpDisplay::displayText(I, 20, 20, "Secondary task enabled: yes", vpColor::white);
          vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::white);
        }
      }
      else {
        if (opt_display && opt_click_allowed) {
          vpDisplay::displayText(I, 20, 20, "Secondary task enabled: no", vpColor::white);
        }
      }

      // send the camera velocity to the controller
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

      std::cout << "|| s - s* || = " << (task.getError()).sumSquare() << std::endl;

      if (opt_display) {
        vpDisplay::displayText(I, 60, 20, "Click to stop visual servo...", vpColor::white);
        if (vpDisplay::getClick(I, false)) {
          stop = true;
        }
        vpDisplay::flush(I);
        vpDisplay::flush(Iext);
      }
    }

    if (opt_display && opt_click_allowed) {
      vpDisplay::display(I);
      vpServoDisplay::display(task, cam, I);
      vpDisplay::displayText(I, 20, 20, "Click to quit...", vpColor::white);
      vpDisplay::flush(I);
      vpDisplay::getClick(I);
    }

    // Display task information
    task.print();
    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Catch a ViSP exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
#else
  (void)argc;
  (void)argv;
  std::cout << "Cannot run this example: install Lapack, Eigen3 or OpenCV" << std::endl;
  return EXIT_SUCCESS;
#endif
  }
