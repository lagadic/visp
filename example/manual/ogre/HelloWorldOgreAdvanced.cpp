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
  \example HelloWorldOgreAdvanced.cpp

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

#if defined(VISP_HAVE_OGRE)

#ifndef DOXYGEN_SHOULD_SKIP_THIS

class vpAROgreAdvanced : public vpAROgre
{
private:
  // Animation attribute
  Ogre::AnimationState *mAnimationState;

public:
  vpAROgreAdvanced(const vpCameraParameters &cam = vpCameraParameters(), unsigned int width = 640,
                   unsigned int height = 480)
    : vpAROgre(cam, width, height)
  {
    mAnimationState = NULL;
  }

protected:
  void createScene()
  {
    // Create the Entity
    Ogre::Entity *robot = mSceneMgr->createEntity("Robot", "robot.mesh");
    // Attach robot to scene graph
    Ogre::SceneNode *RobotNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("Robot");
    // RobotNode->setPosition((Ogre::Real)-0.3, (Ogre::Real)0.2,
    // (Ogre::Real)0);
    RobotNode->attachObject(robot);
    RobotNode->scale((Ogre::Real)0.001, (Ogre::Real)0.001, (Ogre::Real)0.001);
    RobotNode->pitch(Ogre::Degree(180));
    RobotNode->yaw(Ogre::Degree(-90));

    // The animation
    // Set the good animation
    mAnimationState = robot->getAnimationState("Idle");
    // Start over when finished
    mAnimationState->setLoop(true);
    // Animation enabled
    mAnimationState->setEnabled(true);
  }

  bool customframeEnded(const Ogre::FrameEvent &evt)
  {
    // Update animation
    // To move, we add it the time since last frame
    mAnimationState->addTime(evt.timeSinceLastFrame);
    return true;
  }
}; // End of vpAROgreAdvanced class definition
#endif

#endif

int main()
{
  try {
#if defined(VISP_HAVE_OGRE)
#if defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394) || (VISP_HAVE_OPENCV_VERSION >= 0x020100)

    // Image to store gathered data
    // Here we acquire a grey level image. The consequence will be that
    // the background texture used in Ogre renderer will be also in grey
    // level.
    vpImage<unsigned char> I;

// Now we try to find an available framegrabber
#if defined(VISP_HAVE_V4L2)
    // Video for linux 2 grabber
    vpV4l2Grabber grabber;
    // Open frame grabber
    // Here we acquire an image from an available framegrabber to update
    // the image size
    grabber.open(I);
    grabber.acquire(I);
#elif defined(VISP_HAVE_DC1394)
    // libdc1394-2
    vp1394TwoGrabber grabber;
    // Open frame grabber
    // Here we acquire an image from an available framegrabber to update
    // the image size
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
    vpHomogeneousMatrix cMo;
    cMo[2][3] = 0.5; // Z = 0.5 meter

    // Our object
    vpAROgreAdvanced ogre(cam, I.getWidth(), I.getHeight());
    // Initialisation
    ogre.init(I);

    // Rendering loop
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
      // Display with vpAROgre
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
