/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
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

#include <visp/vpConfig.h>
#include <visp/vpOpenCVGrabber.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpAROgre.h>

#if defined(VISP_HAVE_OGRE)

class vpAROgreAdvanced : public vpAROgre
{
private:
  // Animation attribute
  Ogre::AnimationState * mAnimationState;

public:
  vpAROgreAdvanced(const vpCameraParameters &cam = vpCameraParameters(), 
		   vpBackgroundType type = BACKGROUND_GREY,
		   unsigned int width = 640, unsigned int height = 480)
		: vpAROgre(cam, type, width, height)
  {
  }

protected:
  void createScene()
  {
    // Create the Entity
    Ogre::Entity* robot = mSceneMgr->createEntity("Robot", "robot.mesh");
    // Attach robot to scene graph
    Ogre::SceneNode* RobotNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("Robot");
    RobotNode->setPosition(0.05, 0.05, 0);
    RobotNode->attachObject(robot);
    RobotNode->scale(0.001,0.001,0.001);
    RobotNode->pitch(Ogre::Degree(90));
    RobotNode->yaw(Ogre::Degree(-90));

    // The animation
    // Set the good animation
    mAnimationState = robot->getAnimationState( "Idle" );
    // Start over when finished
    mAnimationState->setLoop( true );
    // Animation enabled
    mAnimationState->setEnabled( true );
  }

  bool customframeEnded( const Ogre::FrameEvent& evt)
  {
    // Update animation
    // To move, we add it the time since last frame
    mAnimationState->addTime( 2*evt.timeSinceLastFrame );
    return true;
  }
};// End of vpAROgreAdvanced class definition
#endif

int main()
{
#if defined(VISP_HAVE_OGRE) && defined(VISP_HAVE_OPENCV)
  // OpenCVGrabber to gather images
  vpOpenCVGrabber grabber;
  // Image to stock gathered data
  vpImage<unsigned char> I;
  // Parameters of our camera
  double px = 565;
  double py = 565;
  double u0 = 320;
  double v0 = 240;
  vpCameraParameters cam(px,py,u0,v0);
  // The matrix with our pose
  vpHomogeneousMatrix cMo;

  // Our object
  vpAROgreAdvanced ogre(cam, vpAROgre::BACKGROUND_COLOR,
                       (unsigned int)grabber.getWidth(), (unsigned int)grabber.getHeight());
  // Initialisation
  ogre.init();

  // Open frame grabber
  grabber.open(I);

  // Rendering loop
  while(ogre.continueRendering()){
      // Image Acquisition
      grabber.acquire(I);
      // Pose computation
      // ...
      // cMo updated
      // Display with vpAROgre
      ogre.display(I, cMo);
  }
  // Release video device
  grabber.close();
#endif
}
