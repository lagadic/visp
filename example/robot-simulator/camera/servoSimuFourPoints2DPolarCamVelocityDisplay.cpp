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
 * Simulation of a 2D visual servoing using 4 points with polar
 * coordinates as visual feature.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example servoSimuFourPoints2DPolarCamVelocityDisplay.cpp

  \brief Simulation of a 2D visual servoing:

  Simulation of a 2D visual servoing:
  - servo on 4 points with polar coordinates,
  - eye-in-hand control law,
  - camera velocities are computed,
  - display internal camera view and an external view.

  Interaction matrix is computed as the mean of the current and desired
  interaction matrix.

*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>

#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))

#include <stdio.h>
#include <stdlib.h>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpProjectionDisplay.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePointPolar.h>
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
Tests a control law with the following characteristics:\n\
- eye-in-hand control\n\
- articular velocity are computed\n\
- servo on 4 points,\n\
- internal and external camera view displays.\n\
\n\
SYNOPSIS\n\
  %s [-c] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
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
    // Log file creation in /tmp/$USERNAME/log.dat
    // This file contains by line:
    // - the 6 computed camera velocities (m/s, rad/s) to achieve the task
    // - the 6 mesured camera velocities (m/s, rad/s)
    // - the 6 mesured joint positions (m, rad)
    // - the 8 values of s - s*
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

    bool opt_click_allowed = true;
    bool opt_display = true;

    // Read the command line options
    if (getOptions(argc, argv, opt_click_allowed, opt_display) == false) {
      exit(-1);
    }

// We open two displays, one for the internal camera view, the other one for
// the external view, using either X11, GTK or GDI.
#if defined VISP_HAVE_X11
    vpDisplayX displayInt;
    vpDisplayX displayExt;
#elif defined VISP_HAVE_GTK
    vpDisplayGTK displayInt;
    vpDisplayGTK displayExt;
#elif defined VISP_HAVE_GDI
    vpDisplayGDI displayInt;
    vpDisplayGDI displayExt;
#elif defined VISP_HAVE_OPENCV
    vpDisplayOpenCV displayInt;
    vpDisplayOpenCV displayExt;
#endif

    // open a display for the visualization

    vpImage<unsigned char> Iint(300, 300, 0);
    vpImage<unsigned char> Iext(300, 300, 0);

    if (opt_display) {
      displayInt.init(Iint, 0, 0, "Internal view");
      displayExt.init(Iext, 330, 000, "External view");
    }
    vpProjectionDisplay externalview;

    double px, py;
    px = py = 500;
    double u0, v0;
    u0 = 150, v0 = 160;

    vpCameraParameters cam(px, py, u0, v0);

    int i;
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

// #define TRANS_Z_PURE
// #define TRANS_X_PURE
// #define ROT_Z_PURE
// #define ROT_X_PURE
#define COMPLEX
//#define PROBLEM

#if defined(TRANS_Z_PURE)
    // sets the initial camera location
    vpHomogeneousMatrix cMo(0, 0, 3, vpMath::rad(0), vpMath::rad(0), vpMath::rad(0));
    // sets the desired camera location
    vpHomogeneousMatrix cMod(0, 0, 2, vpMath::rad(0), vpMath::rad(0), vpMath::rad(0));
#elif defined(TRANS_X_PURE)
    // sets the initial camera location
    vpHomogeneousMatrix cMo(0.3, 0.3, 3, vpMath::rad(0), vpMath::rad(0), vpMath::rad(0));
    // sets the desired camera location
    vpHomogeneousMatrix cMod(0.5, 0.3, 3, vpMath::rad(0), vpMath::rad(0), vpMath::rad(0));

#elif defined(ROT_Z_PURE)
    // sets the initial camera location
    vpHomogeneousMatrix cMo(0, 0, 3, vpMath::rad(0), vpMath::rad(0), vpMath::rad(0));
    // sets the desired camera location
    vpHomogeneousMatrix cMod(0, 0, 3, vpMath::rad(0), vpMath::rad(0), vpMath::rad(180));

#elif defined(ROT_X_PURE)
    // sets the initial camera location
    vpHomogeneousMatrix cMo(0, 0, 3, vpMath::rad(0), vpMath::rad(0), vpMath::rad(0));
    // sets the desired camera location
    vpHomogeneousMatrix cMod(0, 0, 3, vpMath::rad(45), vpMath::rad(0), vpMath::rad(0));

#elif defined(COMPLEX)
    // sets the initial camera location
    vpHomogeneousMatrix cMo(0.2, 0.2, 3, vpMath::rad(0), vpMath::rad(0), vpMath::rad(0));
    // sets the desired camera location
    vpHomogeneousMatrix cMod(0, 0, 2.5, vpMath::rad(45), vpMath::rad(10), vpMath::rad(30));

#elif defined(PROBLEM)
    // Bad behavior with an interaction matrix computed from the desired
    // features sets the initial camera location
    vpHomogeneousMatrix cMo(0.2, 0.2, 3, vpMath::rad(0), vpMath::rad(0), vpMath::rad(0));
    // sets the desired camera location
    vpHomogeneousMatrix cMod(0.4, 0.2, 3, vpMath::rad(0), vpMath::rad(0), vpMath::rad(0));

#endif
    // Compute the position of the object in the world frame
    vpHomogeneousMatrix wMc, wMo;
    robot.getPosition(wMc);
    wMo = wMc * cMo;

    vpHomogeneousMatrix cextMo(0, 0, 6, vpMath::rad(40), vpMath::rad(10), vpMath::rad(60));

    // sets the point coordinates in the object frame
    vpPoint point[4];
    point[0].setWorldCoordinates(-0.25, -0.25, 0);
    point[1].setWorldCoordinates(0.25, -0.25, 0);
    point[2].setWorldCoordinates(0.25, 0.25, 0);
    point[3].setWorldCoordinates(-0.25, 0.25, 0);

    for (i = 0; i < 4; i++)
      externalview.insert(point[i]);

    // sets the desired position of the feature point s*"
    vpFeaturePointPolar pd[4];

    // computes the point coordinates in the desired camera frame and
    // its 2D coordinates
    for (i = 0; i < 4; i++) {
      point[i].track(cMod);
      // Computes the polar coordinates from the image point
      // cartesian coordinates
      vpFeatureBuilder::create(pd[i], point[i]);
    }

    // computes the point coordinates in the camera frame and its 2D
    // coordinates
    for (i = 0; i < 4; i++)
      point[i].track(cMo);

    // sets the desired position of the point
    vpFeaturePointPolar p[4];
    for (i = 0; i < 4; i++) {
      // retrieve x,y and Z of the vpPoint structure to initialize the
      // visual feature
      vpFeatureBuilder::create(p[i], point[i]);
    }

    // Define the task;
    // - we want an eye-in-hand control law
    // - articular velocity are computed
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    //  task.setInteractionMatrixType(vpServo::MEAN) ;
    //  task.setInteractionMatrixType(vpServo::DESIRED) ;
    task.setInteractionMatrixType(vpServo::CURRENT);

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

    std::cout << "\nDisplay task information: " << std::endl;
    task.print();

    unsigned int iter = 0;
    // loop
    while (iter++ < 200) {
      std::cout << "---------------------------------------------" << iter << std::endl;
      vpColVector v;

      // Set the Jacobian (expressed in the end-effector frame)
      // Since q is modified eJe is modified
      robot.get_eJe(eJe);
      task.set_eJe(eJe);

      // get the robot position
      robot.getPosition(wMc);
      // Compute the position of the camera wrt the object frame
      cMo = wMc.inverse() * wMo;

      // Compute new point position
      for (i = 0; i < 4; i++) {
        point[i].track(cMo);
        // retrieve x,y and Z of the vpPoint structure to compute the feature
        vpFeatureBuilder::create(p[i], point[i]);
      }

      if (opt_display) {
        vpDisplay::display(Iint);
        vpDisplay::display(Iext);

        vpServoDisplay::display(task, cam, Iint);
        externalview.display(Iext, cextMo, cMo, cam, vpColor::green);
        vpDisplay::flush(Iint);
        vpDisplay::flush(Iext);
      }

      // Compute the control law
      v = task.computeControlLaw();

      if (iter == 1) {
        std::cout << "Display task information: " << std::endl;
        task.print();
      }

      task.print(vpServo::FEATURE_CURRENT);
      task.print(vpServo::FEATURE_DESIRED);

      // Send the camera velocity to the controller
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);
      // Save velocities applied to the robot in the log file
      // v[0], v[1], v[2] correspond to camera translation velocities in m/s
      // v[3], v[4], v[5] correspond to camera rotation velocities in rad/s
      flog << v[0] << " " << v[1] << " " << v[2] << " " << v[3] << " " << v[4] << " " << v[5] << " ";

      std::cout << "v: " << v.t() << std::endl;

      std::cout << "|| s - s* || = " << (task.getError()).sumSquare() << std::endl;

      // Save feature error (s-s*) for the 4 feature points. For each feature
      // point, we have 2 errors (along x and y axis).  This error is
      // expressed in meters in the camera frame
      flog << (task.getError()).t() << " "; // s-s* for point 4
      std::cout << "|| s - s* || = " << (task.getError()).sumSquare() << std::endl;

      // Save current visual feature s = (rho,theta)
      for (i = 0; i < 4; i++) {
        flog << p[i].get_rho() << " " << p[i].get_theta() << " ";
      }
      // Save current position of the points
      for (i = 0; i < 4; i++) {
        flog << point[i].get_x() << " " << point[i].get_y() << " ";
      }
      flog << std::endl;

      if (iter == 1) {
        vpImagePoint ip;
        ip.set_i(10);
        ip.set_j(10);

        std::cout << "\nClick in the internal camera view to continue..." << std::endl;
        vpDisplay::displayText(Iint, ip, "A click to continue...", vpColor::red);
        vpDisplay::flush(Iint);
        vpDisplay::getClick(Iint);
      }
    }

    flog.close(); // Close the log file

    // Display task information
    task.print();

    // Kill the task
    task.kill();

    std::cout << "Final robot position with respect to the object frame:\n";
    cMo.print();

    if (opt_display && opt_click_allowed) {
      // suppressed for automate test
      std::cout << "\n\nClick in the internal view to end..." << std::endl;
      vpDisplay::getClick(Iint);
    }
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
