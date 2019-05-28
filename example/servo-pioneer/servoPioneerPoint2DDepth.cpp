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
 * IBVS on Pioneer P3DX mobile platform
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
#include <iostream>

#include <visp3/core/vpConfig.h>

#include <visp3/robot/vpRobotPioneer.h> // Include first to avoid build issues with Status, None, isfinite
#include <visp3/blob/vpDot2.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vp1394CMUGrabber.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/sensor/vpOpenCVGrabber.h>
#include <visp3/sensor/vpV4l2Grabber.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>

#if defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_CMU1394) ||                              \
    (VISP_HAVE_OPENCV_VERSION >= 0x020100)
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)
#if defined(VISP_HAVE_PIONEER)
#define TEST_COULD_BE_ACHIEVED
#endif
#endif
#endif

#undef VISP_HAVE_OPENCV // To use a firewire camera
#undef VISP_HAVE_V4L2   // To use a firewire camera

/*!
  \example servoPioneerPoint2DDepth.cpp

  Example that shows how to control the Pioneer mobile robot by IBVS visual
  servoing with respect to a blob. The current visual features that are used
  are s = (x, log(Z/Z*)). The desired one are s* = (x*, 0), with:
  - x the abscisse of the point corresponding to the blob center of gravity
  measured at each iteration,
  - x* the desired abscisse position of the point (x* = 0)
  - Z the depth of the point measured at each iteration
  - Z* the desired depth of the point equal to the initial one.

  The degrees of freedom that are controlled are (vx, wz), where wz is the
  rotational velocity and vx the translational velocity of the mobile platform
  at point M located at the middle between the two wheels.

  The feature x allows to control wy, while log(Z/Z*) allows to control vz.
  The value of x is measured thanks to a blob tracker.
  The value of Z is estimated from the surface of the blob that is
  proportional to the depth Z.

  */
#ifdef TEST_COULD_BE_ACHIEVED
int main(int argc, char **argv)
{
  try {
    vpImage<unsigned char> I; // Create a gray level image container
    double depth = 1.;
    double lambda = 0.6;
    double coef = 1. / 6.77; // Scale parameter used to estimate the depth Z
                             // of the blob from its surface

    vpRobotPioneer robot;
    ArArgumentParser parser(&argc, argv);
    parser.loadDefaultArguments();

    // ArRobotConnector connects to the robot, get some initial data from it
    // such as type and name, and then loads parameter files for this robot.
    ArRobotConnector robotConnector(&parser, &robot);
    if (!robotConnector.connectRobot()) {
      ArLog::log(ArLog::Terse, "Could not connect to the robot.");
      if (parser.checkHelpAndWarnUnparsed()) {
        Aria::logOptions();
        Aria::exit(1);
      }
    }
    if (!Aria::parseArgs()) {
      Aria::logOptions();
      Aria::shutdown();
      return false;
    }

    // Wait 3 sec to be sure that the low level Aria thread used to control
    // the robot is started. Without this delay we experienced a delay
    // (arround 2.2 sec) between the velocity send to the robot and the
    // velocity that is really applied to the wheels.
    vpTime::sleepMs(3000);

    std::cout << "Robot connected" << std::endl;

    // Camera parameters. In this experiment we don't need a precise
    // calibration of the camera
    vpCameraParameters cam;

// Create the camera framegrabber
#if defined(VISP_HAVE_OPENCV)
    int device = 1;
    std::cout << "Use device: " << device << std::endl;
    cv::VideoCapture g(device); // open the default camera
    g.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    g.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    if (!g.isOpened()) // check if we succeeded
      return -1;
    cv::Mat frame;
    g >> frame; // get a new frame from camera
    vpImageConvert::convert(frame, I);

    // Logitec sphere parameters
    cam.initPersProjWithoutDistortion(558, 555, 312, 210);
#elif defined(VISP_HAVE_V4L2)
    // Create a grabber based on v4l2 third party lib (for usb cameras under
    // Linux)
    vpV4l2Grabber g;
    g.setScale(1);
    g.setInput(0);
    g.setDevice("/dev/video1");
    g.open(I);
    // Logitec sphere parameters
    cam.initPersProjWithoutDistortion(558, 555, 312, 210);
#elif defined(VISP_HAVE_DC1394)
    // Create a grabber based on libdc1394-2.x third party lib (for firewire
    // cameras under Linux)
    vp1394TwoGrabber g(false);
    g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
    g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_30);
    // AVT Pike 032C parameters
    cam.initPersProjWithoutDistortion(800, 795, 320, 216);
#elif defined(VISP_HAVE_CMU1394)
    // Create a grabber based on CMU 1394 third party lib (for firewire
    // cameras under windows)
    vp1394CMUGrabber g;
    g.setVideoMode(0, 5); // 640x480 MONO8
    g.setFramerate(4);    // 30 Hz
    g.open(I);
    // AVT Pike 032C parameters
    cam.initPersProjWithoutDistortion(800, 795, 320, 216);
#endif

// Acquire an image from the grabber
#if defined(VISP_HAVE_OPENCV)
    g >> frame; // get a new frame from camera
    vpImageConvert::convert(frame, I);
#else
    g.acquire(I);
#endif

// Create an image viewer
#if defined(VISP_HAVE_X11)
    vpDisplayX d(I, 10, 10, "Current frame");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I, 10, 10, "Current frame");
#endif
    vpDisplay::display(I);
    vpDisplay::flush(I);

    // Create a blob tracker
    vpDot2 dot;
    dot.setGraphics(true);
    dot.setComputeMoments(true);
    dot.setEllipsoidShapePrecision(0.);       // to track a blob without any constraint on the shape
    dot.setGrayLevelPrecision(0.9);           // to set the blob gray level bounds for binarisation
    dot.setEllipsoidBadPointsPercentage(0.5); // to be accept 50% of bad inner
                                              // and outside points with bad
                                              // gray level
    dot.initTracking(I);
    vpDisplay::flush(I);

    vpServo task;
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);
    task.setLambda(lambda);
    vpVelocityTwistMatrix cVe;
    cVe = robot.get_cVe();
    task.set_cVe(cVe);

    std::cout << "cVe: \n" << cVe << std::endl;

    vpMatrix eJe;
    robot.get_eJe(eJe);
    task.set_eJe(eJe);
    std::cout << "eJe: \n" << eJe << std::endl;

    // Current and desired visual feature associated to the x coordinate of
    // the point
    vpFeaturePoint s_x, s_xd;

    // Create the current x visual feature
    vpFeatureBuilder::create(s_x, cam, dot);

    // Create the desired x* visual feature
    s_xd.buildFrom(0, 0, depth);

    // Add the feature
    task.addFeature(s_x, s_xd);

    // Create the current log(Z/Z*) visual feature
    vpFeatureDepth s_Z, s_Zd;
    // Surface of the blob estimated from the image moment m00 and converted
    // in meters
    double surface = 1. / sqrt(dot.m00 / (cam.get_px() * cam.get_py()));
    double Z, Zd;
    // Initial depth of the blob in from of the camera
    Z = coef * surface;
    // Desired depth Z* of the blob. This depth is learned and equal to the
    // initial depth
    Zd = Z;

    std::cout << "Z " << Z << std::endl;
    s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z,
                  0); // log(Z/Z*) = 0 that's why the last parameter is 0
    s_Zd.buildFrom(s_x.get_x(), s_x.get_y(), Zd,
                   0); // log(Z/Z*) = 0 that's why the last parameter is 0

    // Add the feature
    task.addFeature(s_Z, s_Zd);

    vpColVector v; // vz, wx

    while (1) {
// Acquire a new image
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100)
      g >> frame; // get a new frame from camera
      vpImageConvert::convert(frame, I);
#else
      g.acquire(I);
#endif
      // Set the image as background of the viewer
      vpDisplay::display(I);

      // Does the blob tracking
      dot.track(I);
      // Update the current x feature
      vpFeatureBuilder::create(s_x, cam, dot);

      // Update log(Z/Z*) feature. Since the depth Z change, we need to update
      // the intection matrix
      surface = 1. / sqrt(dot.m00 / (cam.get_px() * cam.get_py()));
      Z = coef * surface;
      s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z, log(Z / Zd));

      robot.get_cVe(cVe);
      task.set_cVe(cVe);

      robot.get_eJe(eJe);
      task.set_eJe(eJe);

      // Compute the control law. Velocities are computed in the mobile robot
      // reference frame
      v = task.computeControlLaw();

      std::cout << "Send velocity to the pionner: " << v[0] << " m/s " << vpMath::deg(v[1]) << " deg/s" << std::endl;

      // Send the velocity to the robot
      robot.setVelocity(vpRobot::REFERENCE_FRAME, v);

      // Draw a vertical line which corresponds to the desired x coordinate of
      // the dot cog
      vpDisplay::displayLine(I, 0, 320, 479, 320, vpColor::red);
      vpDisplay::flush(I);

      // A click in the viewer to exit
      if (vpDisplay::getClick(I, false))
        break;
    }

    std::cout << "Ending robot thread..." << std::endl;
    robot.stopRunning();

    // wait for the thread to stop
    robot.waitForRunExit();

    // Kill the servo task
    task.print();
    task.kill();
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
#else
int main()
{
  std::cout << "You don't have the right 3rd party libraries to run this example..." << std::endl;
  return EXIT_SUCCESS;
}
#endif
