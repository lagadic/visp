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
 * Filip Novotny
 *
 *****************************************************************************/

/*!
  \example servoAfma6Segment2DCamVelocity.cpp

  Example of eye-in-hand control law. We control here a real robot, the Afma6
  robot (cartesian robot, with 6 degrees of freedom). The velocity is computed
  in camera frame. The visual feature is the segment between two points.

*/

#include <stdlib.h>
#include <vector>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h> // Debug trace
#if (defined(VISP_HAVE_AFMA6) && defined(VISP_HAVE_DC1394))

#include <visp3/blob/vpDot.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPoint.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/robot/vpRobotAfma6.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureSegment.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

int main()
{
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

  try {
    vpServo task;

    vpImage<unsigned char> I;

    vp1394TwoGrabber g;
    g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
    g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_60);
    g.open(I);

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

    std::vector<vpDot> dot_d(2), dot(2);
    vpFeatureSegment seg_d, seg;
    vpImagePoint cog;
    vpRobotAfma6 robot;
    vpCameraParameters cam;

    // Update camera parameters
    robot.getCameraParameters(cam, I);
    std::cout << "define the initial segment" << std::endl;

    for (std::vector<vpDot>::iterator i = dot.begin(); i != dot.end(); ++i) {
      std::cout << "Click on a dot..." << std::endl;
      i->initTracking(I);
      cog = i->getCog();
      vpDisplay::displayCross(I, cog, 10, vpColor::blue);
      vpDisplay::flush(I);
    }
    vpFeatureBuilder::create(seg, cam, dot[0], dot[1]);
    seg.display(cam, I, vpColor::red);
    vpDisplay::flush(I);
    std::cout << "define the destination segment" << std::endl;
    for (std::vector<vpDot>::iterator i = dot_d.begin(); i != dot_d.end(); ++i) {
      vpImagePoint ip;
      vpDisplay::getClick(I, ip);
      *i = vpDot(ip);
      vpDisplay::displayCross(I, ip, 10, vpColor::green);
    }
    vpFeatureBuilder::create(seg_d, cam, dot_d[0], dot_d[1]);
    seg_d.setZ1(1.);
    seg_d.setZ2(1.);

    seg_d.display(cam, I);
    vpDisplay::flush(I);
    // define the task
    // - we want an eye-in-hand control law
    // - robot is controlled in the camera frame
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::DESIRED);

    // - we want to see both segments
    task.addFeature(seg, seg_d);

    // - set the constant gain
    task.setLambda(0.8);

    // Display task information
    task.print();

    // Now the robot will be controlled in velocity
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    std::cout << "\nHit CTRL-C to stop the loop...\n" << std::flush;
    for (;;) {
      // Acquire a new image from the camera
      g.acquire(I);

      // Display this image
      vpDisplay::display(I);

      // Achieve the tracking of the dot in the image
      for (std::vector<vpDot>::iterator i = dot.begin(); i != dot.end(); ++i) {
        i->track(I);
      }

      // Update the segment feature from the dot locations
      vpFeatureBuilder::create(seg, cam, dot[0], dot[1]);

      vpColVector v;
      // Compute the visual servoing skew vector
      v = task.computeControlLaw();

      // Display the current and desired feature segments in the image display
      vpServoDisplay::display(task, cam, I);

      // Apply the computed joint velocities to the robot
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

      // Save feature error (s-s*) for the feature segment. For this feature
      // segments, we have 4 errors (Xc,Yc,l,alpha).
      flog << (task.getError()).t() << std::endl;

      // Flush the display
      vpDisplay::flush(I);
    }

    flog.close(); // Close the log file

    // Display task information
    task.print();

    // Kill the task
    task.kill();

    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    flog.close(); // Close the log file
    std::cout << "Test failed with exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}

#else
int main()
{
  std::cout << "You do not have an afma6 robot connected to your computer..." << std::endl;
  return EXIT_SUCCESS;
}
#endif
