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
 *   velocity computed in the camera frame
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example servoAfma4Point2DCamVelocityKalman.cpp

  \brief Example of eye-in-hand control law. We control here a real
  robot, the Afma4 robot (cylindrical robot, with 4 degrees of
  freedom). The velocity is computed in the camera frame. The visual
  feature is the center of gravity of a point.

  In this example we estimate the velocity of the target in order to reduce
  the tracking error when the target is moving. The velocity of the target is
  filtered by a Kalman filter with a constant velocity state model, or a
  constant acceleration state model.
*/

#include <stdlib.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h> // Debug trace
#if (defined(VISP_HAVE_AFMA4) && defined(VISP_HAVE_DC1394))

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vp1394TwoGrabber.h>

#include <visp3/blob/vpDot2.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpLinearKalmanFilterInstantiation.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPoint.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/robot/vpRobotAfma4.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpAdaptiveGain.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

// List of allowed command line options
#define GETOPTARGS "hK:l:"

typedef enum { K_NONE, K_VELOCITY, K_ACCELERATION } KalmanType;

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param kalman : Kalman state model selector.

*/
void usage(const char *name, const char *badparam, KalmanType &kalman)
{
  fprintf(stdout, "\n\
Tests a control law with the following characteristics:\n\
- eye-in-hand control\n\
- camera velocity are computed\n\
- servo on 1 points.\n\
- Kalman filtering\n\
          \n\
SYNOPSIS\n\
  %s [-K <0|1|2|3>] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -l <%%f>                                               \n\
     Set the constant gain. By default adaptive gain. \n\
                  \n\
  -K <0|1|2>                                             %d\n\
     Kalman filtering:\n\
       0: none\n\
       1: velocity model\n\
       2: acceleration model\n\
                  \n\
  -h\n\
     Print the help.\n", (int)kalman);

  if (badparam) {
    fprintf(stderr, "ERROR: \n");
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);
  }
}

/*!

Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param kalman : Kalman state model selector.
  \param doAdaptativeGain : If true use an adaptive gain. Otherwise,
  the gain is constant.
  \param lambda : Constant visual servo gain.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, KalmanType &kalman, bool &doAdaptativeGain,
                vpAdaptiveGain &lambda) // gain lambda
{
  const char *optarg;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'K':
      kalman = (KalmanType)atoi(optarg);
      break;
    case 'l':
      doAdaptativeGain = false;
      lambda.initFromConstant(atof(optarg));
      break;
    case 'h':
      usage(argv[0], NULL, kalman);
      return false;
      break;

    default:
      usage(argv[0], optarg, kalman);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, kalman);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}

int main(int argc, const char **argv)
{
  try {
    KalmanType opt_kalman = K_NONE;
    vpAdaptiveGain lambda;        // Gain de la commande
    bool doAdaptativeGain = true; // Compute adaptative gain
    lambda.initStandard(4, 0.2, 40);
    int opt_cam_frequency = 60; // 60 Hz

    // Read the command line options
    if (getOptions(argc, argv, opt_kalman, doAdaptativeGain, lambda) == false) {
      return (-1);
    }

    // Log file creation in /tmp/$USERNAME/log.dat
    // This file contains by line:
    // - the 6 computed cam velocities (m/s, rad/s) to achieve the task
    // - the 6 mesured joint velocities (m/s, rad/s)
    // - the 6 mesured joint positions (m, rad)
    // - the 2 values of s - s*
    std::string username;
    // Get the user login name
    vpIoTools::getUserName(username);

    // Create a log filename to save velocities...
    std::string logdirname;
    logdirname = "/tmp/" + username;

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

    vpImage<unsigned char> I;
    vp1394TwoGrabber g(false);
    g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
    switch (opt_cam_frequency) {
    case 15:
      g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_15);
      break;
    case 30:
      g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_30);
      break;
    case 60:
      g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_60);
      break;
    }
    g.open(I);

    for (int i = 0; i < 10; i++) //  10 acquisition to warm up the camera
      g.acquire(I);

#ifdef VISP_HAVE_X11
    vpDisplayX display(I, 100, 100, "Current image");
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV display(I, 100, 100, "Current image");
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK display(I, 100, 100, "Current image");
#endif

    vpDisplay::display(I);
    vpDisplay::flush(I);

    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << "Test program for target motion compensation using a Kalman "
                 "filter "
              << std::endl;
    std::cout << "Eye-in-hand task control, velocity computed in the camera frame" << std::endl;
    std::cout << "Task : servo a point \n" << std::endl;

    // Kalman filtering
    switch (opt_kalman) {
    case K_NONE:
      std::cout << "Servo with no target motion compensation (see -K option)\n";
      break;
    case K_VELOCITY:
      std::cout << "Servo with target motion compensation using a Kalman filter\n"
                << "with constant velocity modelization (see -K option)\n";
      break;
    case K_ACCELERATION:
      std::cout << "Servo with target motion compensation using a Kalman filter\n"
                << "with constant acceleration modelization (see -K option)\n";
      break;
    }
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << std::endl;

    vpDot2 dot;

    std::cout << "Click on the dot..." << std::endl;
    dot.setGraphics(true);
    dot.initTracking(I);
    vpImagePoint cog;
    cog = dot.getCog();
    vpDisplay::displayCross(I, cog, 10, vpColor::blue);
    vpDisplay::flush(I);

    vpRobotAfma4 robot;

    double px = 1000;
    double py = 1000;
    double u0 = I.getWidth() / 2.;
    double v0 = I.getHeight() / 2.;

    vpCameraParameters cam(px, py, u0, v0);

    // Sets the current position of the visual feature
    vpFeaturePoint p;
    vpFeatureBuilder::create(p, cam, dot);

    // Sets the desired position of the visual feature
    vpFeaturePoint pd;
    pd.buildFrom(0, 0, 1);

    // Define the task
    // - we want an eye-in-hand control law
    // - robot is controlled in the camera frame
    task.setServo(vpServo::EYEINHAND_CAMERA);

    // - we want to see a point on a point
    std::cout << std::endl;
    task.addFeature(p, pd);

    // - set the gain
    task.setLambda(lambda);

    // Display task information
    // task.print() ;

    //--------------------------------------------------------------------------
    //!---------------------------Init Kalman Filter--------------------------
    //--------------------------------------------------------------------------

    //! Initialize filter
    vpLinearKalmanFilterInstantiation kalman;

    // Initialize the kalman filter
    unsigned int nsignal = 2; // The two values of dedt
    double rho = 0.3;
    vpColVector sigma_state;
    vpColVector sigma_measure(nsignal);
    unsigned int state_size = 0; // Kalman state vector size

    switch (opt_kalman) {
    case K_VELOCITY: {
      // Set the constant velocity state model used for the filtering
      kalman.setStateModel(vpLinearKalmanFilterInstantiation::stateConstVelWithColoredNoise_MeasureVel);
      state_size = kalman.getStateSize();
      sigma_state.resize(state_size * nsignal);
      sigma_state = 0.00001; // Same state variance for all signals
      sigma_measure = 0.05;  // Same measure variance for all the signals
      double dummy = 0;      // non used parameter dt for the velocity state model
      kalman.initFilter(nsignal, sigma_state, sigma_measure, rho, dummy);

      break;
    }
    case K_ACCELERATION: {
      // Set the constant acceleration state model used for the filtering
      kalman.setStateModel(vpLinearKalmanFilterInstantiation::stateConstAccWithColoredNoise_MeasureVel);
      state_size = kalman.getStateSize();
      sigma_state.resize(state_size * nsignal);
      sigma_state = 0.00001; // Same variance for all the signals
      sigma_measure = 0.05;  // Same measure variance for all the signals
      double dt = 1. / opt_cam_frequency;
      kalman.initFilter(nsignal, sigma_state, sigma_measure, rho, dt);
      break;
    }
    default:
      break;
    }

    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    int iter = 0;

    double t_1, Tv_0;
    vpColVector vm(6), vm_0(6);
    vpColVector v(6), v1(6), v2(6); // robot velocities
    // task error
    vpColVector err(2), err_0(2), err_1(2);
    vpColVector dedt_filt(2), dedt_mes(2);

    t_1 = vpTime::measureTimeMs(); // t_1: time at previous iter

    Tv_0 = 0;

    //
    // Warning: In all varaible names,
    //   _0 means the value for the current iteration (t=0)
    //   _1 means the value for the previous iteration (t=-1)
    //   _2 means the value for the previous previous iteration (t=-2)
    //
    std::cout << "\nHit CTRL-C to stop the loop...\n" << std::flush;
    for (;;) {
      double t_0 = vpTime::measureTimeMs(); // t_0: current time
      // Temps de la boucle d'asservissement
      double Tv = (double)(t_0 - t_1) / 1000.0; // temps d'une iteration en s
                                                // !
      //     std::cout << "time iter : " << Tv << std::endl;

      // Update time for next iteration
      t_1 = t_0;

      vm = robot.getVelocity(vpRobot::CAMERA_FRAME);

      // Acquire a new image from the camera
      g.acquire(I);

      // Display this image
      vpDisplay::display(I);

      // Achieve the tracking of the dot in the image
      dot.track(I);
      vpImagePoint cog = dot.getCog();

      // Display a green cross at the center of gravity position in the image
      vpDisplay::displayCross(I, cog, 10, vpColor::green);

      // Update the point feature from the dot location
      vpFeatureBuilder::create(p, cam, dot);

      //----------------------------------------------------------------------
      //!-------------------- Update displacements and time ------------------
      //----------------------------------------------------------------------
      vm_0 = vm;

      // Update current loop time and previous one
      double Tv_1 = Tv_0;
      Tv_0 = Tv;

      // Compute the visual servoing skew vector
      v1 = task.computeControlLaw();

      err = task.error;

      //! terme correctif : de/dt = Delta s / Delta t - L*vc
      if (iter == 0) {
        err_0 = 0;
        err_1 = 0;
        dedt_mes = 0;
        dedt_filt = 0;
      } else {
        err_1 = err_0;
        err_0 = err;

        dedt_mes = (err_0 - err_1) / (Tv_1)-task.J1 * vm_0;
      }
      //! pour corriger pb de iter = 1
      if (iter <= 1) {
        dedt_mes = 0;
      }

      //----------------------------------------------------------------------
      //----------------------- Kalman Filter Equations ----------------------
      //----------------------------------------------------------------------
      // Kalman filtering
      switch (opt_kalman) {
      case K_NONE:
        dedt_filt = 0;
        break;
      case K_VELOCITY:
      case K_ACCELERATION:
        kalman.filter(dedt_mes);
        for (unsigned int i = 0; i < nsignal; i++) {
          dedt_filt[i] = kalman.Xest[i * state_size];
        }
        break;
      }

      //! Modified control law
      vpMatrix J1p = task.getTaskJacobianPseudoInverse();
      v2 = -J1p * dedt_filt;
      //   std::cout << "task J1p: " <<  J1p.t() << std::endl  ;
      //   std::cout << "dedt_filt: " <<  dedt_filt.t() << std::endl  ;

      v = v1 + v2;

      // Display the current and desired feature points in the image display
      vpServoDisplay::display(task, cam, I);

      //    std::cout << "v2 : " << v2.t() << std::endl  ;
      //   std::cout << "v1 : " << v1.t() << std::endl  ;

      //   std::cout << "v : " << v.t();

      // Apply the camera velocities to the robot
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

      // Save loop time
      flog << Tv_0 << " ";

      // Save velocities applied to the robot in the log file
      // v[0], v[1], v[2] correspond to camera translation velocities in m/s
      // v[3], v[4], v[5] correspond to camera rotation velocities in rad/s
      flog << v[0] << " " << v[1] << " " << v[2] << " " << v[3] << " " << v[4] << " " << v[5] << " ";

      // Save feature error (s-s*) for the feature point. For this feature
      // point, we have 2 errors (along x and y axis).  This error is
      // expressed in meters in the camera frame
      flog << task.error[0] << " " << task.error[1] << " ";

      // Save feature error (s-s*) in pixels in the image.
      flog << cog.get_u() - cam.get_u0() << " " << cog.get_v() - cam.get_v0() << " ";

      // Save de/dt
      flog << dedt_mes[0] << " " << dedt_mes[1] << " ";

      // Save de/dt filtered
      flog << dedt_filt[0] << " " << dedt_filt[1] << " ";

      flog << std::endl;

      // Flush the display
      vpDisplay::flush(I);

      iter++;
    }

    flog.close(); // Close the log file

    // Display task information
    task.print();

    // Kill the task
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
  std::cout << "You do not have an afma4 robot connected to your computer..." << std::endl;
  return EXIT_SUCCESS;
}
#endif
