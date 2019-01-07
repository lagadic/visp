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
 * Test for Afma 6 dof robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testRobotViper850Pose.cpp

  Example of robot pose usage.

  Show how to compute rMo = rMc * cMo with cMo obtained by pose computation
  and rMc from the robot position.

*/

#include <iostream>
#include <visp3/blob/vpDot.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPoint.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/robot/vpRobotViper850.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/vision/vpPose.h>
#if defined(VISP_HAVE_VIPER850) && defined(VISP_HAVE_DC1394)

int main()
{
  try {
    // Create an image B&W container
    vpImage<unsigned char> I;

    // Create a firewire grabber based on libdc1394-2.x
    bool reset = false;
    vp1394TwoGrabber g(reset);

    // Grab an image from the firewire camera
    g.acquire(I);

// Create an image viewer for the image
#ifdef VISP_HAVE_X11
    vpDisplayX display(I, 100, 100, "Current image");
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV display(I, 100, 100, "Current image");
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK display(I, 100, 100, "Current image");
#endif

    // Display the image
    vpDisplay::display(I);
    vpDisplay::flush(I);

    // Define a squared target
    // The target is made of 4 planar points (square dim = 0.077m)
    double sdim = 0.077; // square width and height
    vpPoint target[4];
    // Set the point world coordinates (x,y,z) in the object frame
    // o ----> x
    // |
    // |
    // \/
    // y
    target[0].setWorldCoordinates(-sdim / 2., -sdim / 2., 0);
    target[1].setWorldCoordinates(sdim / 2., -sdim / 2., 0);
    target[2].setWorldCoordinates(sdim / 2., sdim / 2., 0);
    target[3].setWorldCoordinates(-sdim / 2., sdim / 2., 0);

    // Image processing to extract the 2D coordinates in sub-pixels of the 4
    // points from the image acquired by the camera
    // Creation of 4 trackers
    vpDot dot[4];
    vpImagePoint cog;
    for (int i = 0; i < 4; i++) {
      dot[i].setGraphics(true); // to display the tracking results
      std::cout << "Click on dot " << i << std::endl;
      dot[i].initTracking(I);
      // The tracker computes the sub-pixels coordinates in the image
      // i ----> u
      // |
      // |
      // \/
      // v
      std::cout << "  Coordinates: " << dot[i].getCog() << std::endl;
      // Flush the tracking results in the viewer
      vpDisplay::flush(I);
    }

    // Create an intrinsic camera parameters structure
    vpCameraParameters cam;

    // Create a robot access
    vpRobotViper850 robot;

    // Load the end-effector to camera frame transformation obtained
    // using a camera intrinsic model with distortion
    robot.init(vpViper850::defaultTool, vpCameraParameters::perspectiveProjWithDistortion);

    // Get the intrinsic camera parameters associated to the image
    robot.getCameraParameters(cam, I);

    // Using the camera parameters, compute the perspective projection
    // (transform the dot sub-pixel coordinates into coordinates in the camera
    // frame in meter)
    for (int i = 0; i < 4; i++) {
      double x = 0, y = 0; // coordinates of the dots in the camera frame
      // c ----> x
      // |
      // |
      // \/
      // y
      // pixel to meter conversion
      cog = dot[i].getCog();
      vpPixelMeterConversion::convertPoint(cam, cog, x, y);
      target[i].set_x(x);
      target[i].set_y(y);
    }

    // From now, in target[i], we have the 3D coordinates of a point in the
    // object frame, and their correspondances in the camera frame. We can now
    // compute the pose cMo between the camera and the object.
    vpPose pose;
    // Add the 4 points to compute the pose
    for (int i = 0; i < 4; i++) {
      pose.addPoint(target[i]);
    }
    // Create an homogeneous matrix for the camera to object transformation
    // computed just bellow
    vpHomogeneousMatrix cMo;
    vpRotationMatrix R;
    vpRxyzVector r;
    // Compute the pose: initialisation is done by Lagrange method, and the
    // final pose is computed by the more accurate Virtual Visual Servoing
    // method.
    pose.computePose(vpPose::LAGRANGE_VIRTUAL_VS, cMo);

    std::cout << "Pose cMo: " << std::endl << cMo;
    cMo.extract(R);
    r.buildFrom(R);
    std::cout << "  rotation: " << vpMath::deg(r[0]) << " " << vpMath::deg(r[1]) << " " << vpMath::deg(r[2]) << " deg"
              << std::endl
              << std::endl;

    // Get the robot position in the reference frame
    vpHomogeneousMatrix rMc;
    vpColVector p; // position x,y,z,rx,ry,rz
    robot.getPosition(vpRobotViper850::REFERENCE_FRAME, p);
    std::cout << "Robot pose in reference frame: " << p << std::endl;
    vpTranslationVector t;
    t[0] = p[0];
    t[1] = p[1];
    t[2] = p[2];
    r[0] = p[3];
    r[1] = p[4];
    r[2] = p[5];
    R.buildFrom(r);
    rMc.buildFrom(t, R);
    std::cout << "Pose rMc: " << std::endl << rMc;
    rMc.extract(R);
    r.buildFrom(R);
    std::cout << "  rotation: " << vpMath::deg(r[0]) << " " << vpMath::deg(r[1]) << " " << vpMath::deg(r[2]) << " deg"
              << std::endl
              << std::endl;

    robot.getPosition(vpRobotViper850::ARTICULAR_FRAME, p);
    std::cout << "Robot pose in articular: " << p << std::endl;

    robot.get_fMc(p, rMc);
    std::cout << "Pose rMc from MGD: " << std::endl << rMc;
    rMc.extract(R);
    r.buildFrom(R);
    std::cout << "  rotation: " << vpMath::deg(r[0]) << " " << vpMath::deg(r[1]) << " " << vpMath::deg(r[2]) << " deg"
              << std::endl
              << std::endl;

    vpHomogeneousMatrix rMo;
    rMo = rMc * cMo;
    std::cout << "Pose rMo = rMc * cMo: " << std::endl << rMo;
    rMo.extract(R);
    r.buildFrom(R);
    std::cout << "  rotation: " << vpMath::deg(r[0]) << " " << vpMath::deg(r[1]) << " " << vpMath::deg(r[2]) << " deg"
              << std::endl
              << std::endl;

  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
  return 0;
}
#else
int main()
{
  std::cout << "Sorry, test not valid. You should have an Viper850 robot..." << std::endl;
  return 0;
}

#endif
