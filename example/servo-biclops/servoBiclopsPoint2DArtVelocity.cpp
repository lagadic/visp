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
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in articular
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file servoBiclopsPoint2DArtVelocity.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the
  biclops robot (pan-tilt head provided by Traclabs). The velocity is computed
  in articular. The visual feature is the center of gravity of a point.

*/

/*!
  \example servoBiclopsPoint2DArtVelocity.cpp

  Example of eye-in-hand control law. We control here a real robot, the
  biclops robot (pan-tilt head provided by Traclabs). The velocity is computed
  in articular. The visual feature is the center of gravity of a point.

*/

#include <signal.h>
#include <stdlib.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h> // Debug trace
#include <visp3/core/vpTime.h>
#if (defined(VISP_HAVE_BICLOPS) && (defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_DIRECTSHOW)))

#ifdef VISP_HAVE_PTHREAD
#include <pthread.h>
#endif

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/sensor/vpDirectShowGrabber.h>

#include <visp3/blob/vpDot.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPoint.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/robot/vpRobotBiclops.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

// Exception
#include <visp3/core/vpException.h>

#ifdef VISP_HAVE_PTHREAD
pthread_mutex_t mutexEndLoop = PTHREAD_MUTEX_INITIALIZER;
#endif

void signalCtrC(int /* signumber */)
{
#ifdef VISP_HAVE_PTHREAD
  pthread_mutex_unlock(&mutexEndLoop);
#endif
  vpTime::wait(10);
  vpTRACE("Ctrl-C pressed...");
}

// List of allowed command line options
#define GETOPTARGS "c:d:h"

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param conf : Robot configuration file.
  \param debugdir : Debug file directory.
  \param user : Username.

 */
void usage(const char *name, const char *badparam, std::string &conf, std::string &debugdir, std::string &user)
{
  fprintf(stdout, "\n\
  Example of eye-in-hand control law. We control here a real robot, the biclops\n\
  robot (pan-tilt head provided by Traclabs). The velocity is\n\
  computed in articular. The visual feature is the center of gravity of a\n\
  point.\n\
\n\
SYNOPSIS\n\
  %s [-c <Biclops configuration file>] [-d <debug file directory>] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -c <Biclops configuration file>                      %s\n\
     Sets the biclops robot configuration file.\n\n\
  -d <debug file directory>                            %s\n\
     Sets the debug file directory.\n\
     From this directory, creates the\"%s\"\n\
     subdirectory depending on the username, where\n\
     it writes biclops.txt file.\n", conf.c_str(), debugdir.c_str(), user.c_str());

  if (badparam) {
    fprintf(stderr, "ERROR: \n");
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);
  }
}
/*!

Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param conf : Robot configuration file.
  \param debugdir : Debug file directory.
  \param user : Username.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &conf, std::string &debugdir, std::string &user)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'c':
      conf = optarg_;
      break;
    case 'd':
      debugdir = optarg_;
      break;
    case 'h':
      usage(argv[0], NULL, conf, debugdir, user);
      return false;
      break;

    default:
      usage(argv[0], optarg_, conf, debugdir, user);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, conf, debugdir, user);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

int main(int argc, const char **argv)
{
  std::cout << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << " Test program for vpServo " << std::endl;
  std::cout << " Eye-in-hand task control, velocity computed in the camera frame" << std::endl;
  std::cout << " Simulation " << std::endl;
  std::cout << " task : servo a point " << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << std::endl;

  try {

#ifdef VISP_HAVE_PTHREAD
    pthread_mutex_lock(&mutexEndLoop);
#endif
    signal(SIGINT, &signalCtrC);

    // default unix configuration file path
    std::string opt_conf = "/usr/share/BiclopsDefault.cfg";

    std::string username;
    std::string debugdir;
    std::string opt_debugdir;

// Set the default output path
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
    opt_debugdir = "/tmp";
#elif defined(_WIN32)
    opt_debugdir = "C:/temp";
#endif

    // Get the user login name
    vpIoTools::getUserName(username);

    // Read the command line options
    if (getOptions(argc, argv, opt_conf, opt_debugdir, username) == false) {
      exit(-1);
    }

    // Get the option value
    if (!opt_debugdir.empty())
      debugdir = opt_debugdir;

    // Append to the output path string, the login name of the user
    std::string dirname = debugdir + "/" + username;

    // Test if the output path exist. If no try to create it
    if (vpIoTools::checkDirectory(dirname) == false) {
      try {
        // Create the dirname
        vpIoTools::makeDirectory(dirname);
      } catch (...) {
        usage(argv[0], NULL, opt_conf, debugdir, username);
        std::cerr << std::endl << "ERROR:" << std::endl;
        std::cerr << "  Cannot create " << dirname << std::endl;
        std::cerr << "  Check your -d " << debugdir << " option " << std::endl;
        exit(-1);
      }
    }

    // Create the debug file: debugdir/$user/biclops.txt
    char filename[FILENAME_MAX];
    sprintf(filename, "%s/biclops.txt", debugdir.c_str());
    FILE *fd = fopen(filename, "w");

    vpRobotBiclops robot(opt_conf.c_str());
    robot.setDenavitHartenbergModel(vpBiclops::DH2);

    {
      vpColVector q(2);
      q = 0;
      robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
      robot.setPosition(vpRobot::ARTICULAR_FRAME, q);
    }

    vpImage<unsigned char> I;

#if defined VISP_HAVE_DC1394
    vp1394TwoGrabber g;
#elif defined VISP_HAVE_DIRECTSHOW
    vpDirectShowGrabber g;
#endif

    g.open(I);

    try {
      g.acquire(I);
    } catch (...) {
      vpERROR_TRACE(" Error caught");
      return (-1);
    }

// We open a window using either X11 or GTK or GDI.
// Its size is automatically defined by the image (I) size
#if defined VISP_HAVE_X11
    vpDisplayX display(I, 100, 100, "Display X...");
#elif defined VISP_HAVE_GTK
    vpDisplayGTK display(I, 100, 100, "Display GTK...");
#elif defined(_WIN32)
    vpDisplayGDI display(I, 100, 100, "Display GDI...");
#endif

    try {
      vpDisplay::display(I);
      vpDisplay::flush(I);
    } catch (...) {
      vpERROR_TRACE(" Error caught");
      return (-1);
    }

    vpServo task;

    vpDot dot;

    try {
      std::cout << "Click on a dot to initialize the tracking..." << std::endl;
      dot.setGraphics(true);
      dot.initTracking(I);
      dot.track(I);
      vpERROR_TRACE("after dot.initTracking(I) ");
    } catch (...) {
      vpERROR_TRACE(" Error caught");
      return (-1);
    }

    vpCameraParameters cam;

    // sets the current position of the visual feature
    vpFeaturePoint p;
    vpFeatureBuilder::create(p, cam, dot); // retrieve x,y and Z of the vpPoint structure

    p.set_Z(1);
    // sets the desired position of the visual feature
    vpFeaturePoint pd;
    pd.buildFrom(0, 0, 1);

    // define the task
    // - we want an eye-in-hand control law
    // - articular velocity are computed
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);

    vpTRACE("Set the position of the camera in the end-effector frame ");
    vpHomogeneousMatrix cMe;
    //  robot.get_cMe(cMe) ;

    vpVelocityTwistMatrix cVe;
    robot.get_cVe(cVe);
    std::cout << cVe << std::endl;
    task.set_cVe(cVe);

    std::cout << "Click in the image to start the servoing..." << std::endl;
    vpDisplay::getClick(I);

    // Set the Jacobian (expressed in the end-effector frame)
    vpMatrix eJe;
    robot.get_eJe(eJe);
    task.set_eJe(eJe);

    // we want to see a point on a point
    task.addFeature(p, pd);

    // set the gain
    task.setLambda(0.2);

    // Display task information
    task.print();

    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    unsigned int iter = 0;
    vpTRACE("\t loop");
#ifdef VISP_HAVE_PTHREAD
    while (0 != pthread_mutex_trylock(&mutexEndLoop))
#else
    for (;;)
#endif
    {
      std::cout << "---------------------------------------------" << iter << std::endl;

      g.acquire(I);
      vpDisplay::display(I);

      dot.track(I);

      //    vpDisplay::displayCross(I,(int)dot.I(), (int)dot.J(),
      //			   10,vpColor::green) ;

      vpFeatureBuilder::create(p, cam, dot);

      // get the jacobian
      robot.get_eJe(eJe);
      task.set_eJe(eJe);

      //  std::cout << (vpMatrix)cVe*eJe << std::endl ;

      vpColVector v;
      v = task.computeControlLaw();

      vpServoDisplay::display(task, cam, I);
      vpDisplay::flush(I);

      std::cout << "v: " << v.t();
      robot.setVelocity(vpRobot::ARTICULAR_FRAME, v);

      std::cout << "|| s - s* || = " << (task.getError()).sumSquare() << std::endl;

      {
        vpColVector s_minus_sStar(2);
        s_minus_sStar = task.s - task.sStar;
        fprintf(fd, "%f %f %f %f %f\n", v[0], v[1], s_minus_sStar[0], s_minus_sStar[1], (task.getError()).sumSquare());
      }
    }

    std::cout << "Display task information " << std::endl;
    task.print();
    task.kill();

    fclose(fd);

    return EXIT_SUCCESS
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
    return EXIT_FAILURE
  }
}

#else
int main()
{
  std::cout << "You do not have an biclops PT robot connected to your computer..." << std::endl;
  return EXIT_SUCCESS;
}
#endif
