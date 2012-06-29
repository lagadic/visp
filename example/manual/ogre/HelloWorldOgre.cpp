/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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
  \example HelloWorldOgre.cpp

  \brief Example that shows how to exploit the vpAROgre class.

*/

#include <visp/vpOpenCVGrabber.h>
#include <visp/vpV4l2Grabber.h>
#include <visp/vp1394TwoGrabber.h>
#include <visp/vpDirectShowGrabber.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpAROgre.h>

int main()
{
#if defined(VISP_HAVE_OGRE) 

  // Now we try to find an available framegrabber
#if defined(VISP_HAVE_V4L2)
  // Video for linux 2 grabber
  vpV4l2Grabber grabber;
#elif defined(VISP_HAVE_DC1394_2)
  // libdc1394-2
  vp1394TwoGrabber grabber;
#elif defined(VISP_HAVE_DIRECTSHOW)
  // OpenCV to gather images
  vpOpenCVGrabber grabber;
#elif defined(VISP_HAVE_OPENCV)
  // OpenCV to gather images
  vpOpenCVGrabber grabber;
#else
#  error "You need an available framegrabber to run this example"
#endif

  // Image to stock gathered data
  // Here we acquire a color image. The consequence will be that
  // the background texture used in Ogre renderer will be also in color.
  vpImage<vpRGBa> I;
  // Open frame grabber
  // Here we acquire an image from an available framegrabber to update
  // the image size
  grabber.open(I);

  // Parameters of our camera
  double px = 565;
  double py = 565;
  double u0 = grabber.getWidth() / 2;
  double v0 = grabber.getHeight() / 2;
  vpCameraParameters cam(px,py,u0,v0);
  // The matrix with our pose
  // Defines the pose of the object in the camera frame
  vpHomogeneousMatrix cMo;

  // Our object
  // A simulator with the camera parameters defined above, 
  // a grey level background image and of the good size
  vpAROgre ogre(cam, (unsigned int)grabber.getWidth(), (unsigned int)grabber.getHeight());
  // Initialisation
  // Here we load the requested plugins specified in the "plugins.cfg" file 
  // and the resources specified in the "resources.cfg" file
  // These two files can be found respectively in ViSP_HAVE_OGRE_PLUGINS_PATH 
  // and ViSP_HAVE_OGRE_RESOURCES_PATH folders  
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
  // Here we load the "robot.mesh" model that is found thanks to the ressources locations 
  // specified in the "resources.cfg" file
  ogre.load("Robot", "robot.mesh");
  ogre.setPosition("Robot", vpTranslationVector(0, 0.05, 0.5));
  ogre.setScale("Robot", 0.001f,0.001f,0.001f);
  ogre.setRotation("Robot", vpRotationMatrix(vpRxyzVector(M_PI, -M_PI/4, 0)));


  // Rendering loop, ended with on escape
  while(ogre.continueRendering()){
    // Image Acquisition
    // Acquire a new image
    grabber.acquire(I);
    //Pose computation
    // ...
    // cMo updated
    // Display the robot at the position specified by cMo with vpAROgre
    ogre.display(I,cMo);
  }
  // Release video device
  grabber.close();
#endif
}
