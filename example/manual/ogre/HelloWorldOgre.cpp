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
 * Ogre example.
 *
 * Authors:
 * Bertrand Delabarre
 *
 *****************************************************************************/
/*!
  \example HelloWorldOgre.cpp

  \brief Example that shows how to exploit the vpAROgre class.

*/

#include <iostream>

#include <visp3/ar/vpAROgre.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/sensor/vpOpenCVGrabber.h>
#include <visp3/sensor/vpV4l2Grabber.h>

int main()
{
  try {
#if defined(VISP_HAVE_OGRE)
#if defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394) || (VISP_HAVE_OPENCV_VERSION >= 0x020100)

    // Image to stock gathered data
    // Here we acquire a color image. The consequence will be that
    // the background texture used in Ogre renderer will be also in color.
    vpImage<vpRGBa> I;

// Now we try to find an available framegrabber
#if defined(VISP_HAVE_V4L2)
    // Video for linux 2 grabber
    vpV4l2Grabber grabber;
    grabber.open(I);
    grabber.acquire(I);
#elif defined(VISP_HAVE_DC1394)
    // libdc1394-2
    vp1394TwoGrabber grabber;
    grabber.open(I);
    grabber.acquire(I);
#elif defined(VISP_HAVE_OPENCV)
    // OpenCV to gather images
    cv::VideoCapture grabber(0); // open the default camera
    if (!grabber.isOpened()) {   // check if we succeeded
      std::cout << "Failed to open the camera" << std::endl;
      return -1;
    }
    cv::Mat frame;
    grabber >> frame; // get a new frame from camera
    vpImageConvert::convert(frame, I);
#endif

    // Parameters of our camera
    double px = 565;
    double py = 565;
    double u0 = I.getWidth() / 2;
    double v0 = I.getHeight() / 2;
    vpCameraParameters cam(px, py, u0, v0);
    // The matrix with our pose
    // Defines the pose of the object in the camera frame
    vpHomogeneousMatrix cMo;

    // Our object
    // A simulator with the camera parameters defined above,
    // a grey level background image and of the good size
    vpAROgre ogre(cam, I.getWidth(), I.getHeight());
    // Initialisation
    // Here we load the requested plugins specified in the "plugins.cfg" file
    // and the resources specified in the "resources.cfg" file
    // These two files can be found respectively in
    // ViSP_HAVE_OGRE_PLUGINS_PATH and ViSP_HAVE_OGRE_RESOURCES_PATH folders
    ogre.init(I);

    // Create a basic scene
    // -----------------------------------
    //  	      Loading things
    // -----------------------------------
    //  As you will see in section 5, our
    //  application knows locations where
    //  it can search for medias.
    //  Here we use a mesh included in
    //  the installation files : a robot.
    // -----------------------------------
    // Here we load the "robot.mesh" model that is found thanks to the
    // resources locations specified in the "resources.cfg" file
    ogre.load("Robot", "robot.mesh");
    // Modify robot scale and orientation
    // - downscale the size to have something completly visible in the image
    // - rotation of 180 deg along robot x axis to have head over feet
    // - rotation of -90 deg along y axis to have robot facing the camera
    ogre.setScale("Robot", 0.001f, 0.001f, 0.001f);
    ogre.setRotation("Robot", vpRotationMatrix(vpRxyzVector(M_PI, -M_PI / 2, 0)));

    // Update projection matrix
    cMo[2][3] = 0.5; // Z = 0.5 meter

    std::cout << "cMo:\n" << cMo << std::endl;

    // Rendering loop, ended with on escape
    while (ogre.continueRendering()) {
// Acquire a new image
#if defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394)
      grabber.acquire(I);
#elif defined(VISP_HAVE_OPENCV)
      grabber >> frame;
      vpImageConvert::convert(frame, I);
#endif
      // Pose computation
      // ...
      // cMo updated
      // Display the robot at the position specified by cMo with vpAROgre
      ogre.display(I, cMo);
    }
#else
    std::cout << "You need an available framegrabber to run this example" << std::endl;
#endif
#else
    std::cout << "You need Ogre3D to run this example" << std::endl;
#endif
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  } catch (...) {
    std::cout << "Catch an exception " << std::endl;
    return EXIT_FAILURE;
  }
}
