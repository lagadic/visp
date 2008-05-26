/****************************************************************************
 *
 * $Id: testRobotAfma6Pose.cpp,v 1.1 2008-05-26 15:14:12 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Test for Afma 6 dof robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testRobotAfma6Pose.cpp

  Example of robot pose usage.

  Show how to compute rMo = rMc * cMo with cMo obtained by pose computation and
  rMc from the robot position.

*/

#include <iostream>


#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpDisplayX.h>
#include <visp/vpRobotAfma6.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vp1394TwoGrabber.h>
#include <visp/vpPoint.h>
#include <visp/vpDot.h>
#include <visp/vpPose.h>
#include <visp/vpDebug.h>

#if defined(VISP_HAVE_AFMA6) && defined(VISP_HAVE_DC1394_2)

int main()
{
  // Create an image B&W container
  vpImage<unsigned char> I;

  // Create a firewire grabber based on libdc1394-2.x
  vp1394TwoGrabber g;

  // Grab an image from the firewire camera
  g.acquire(I);

  // Create an image viewer for the image
  vpDisplayX display(I);

  // Display the image
  vpDisplay::display(I) ;
  vpDisplay::flush(I) ;

  // Define a squared target
  // The target is made of 4 planar points (square dim = 0.077m)
  double sdim = 0.077; // square width and height
  vpPoint target[4] ;
  // Set the point world coordinates (x,y,z) in the object frame
  // o ----> x
  // |
  // |
  // \/
  // y
  target[0].setWorldCoordinates(-sdim/2., -sdim/2., 0) ;
  target[1].setWorldCoordinates( sdim/2., -sdim/2., 0) ;
  target[2].setWorldCoordinates( sdim/2.,  sdim/2., 0) ;
  target[3].setWorldCoordinates(-sdim/2.,  sdim/2., 0) ;

  // Image processing to extract the 2D coordinates in sub-pixels of the 4
  // points from the image acquired by the camera
  // Creation of 4 trackers
  vpDot dot[4];
  for (int i=0; i < 4; i ++) {
    dot[i].setGraphics(true); // to display the tracking results
    std::cout << "Click on dot " << i << std::endl;
    dot[i].initTracking( I );
    // The tracker computes the sub-pixels coordinates in the image
    // i ----> u
    // |
    // |
    // \/
    // v
    std::cout << "  Coordinates ("
	      << dot[i].get_u()
	      << ", "
	      << dot[i].get_v() << std::endl;
    // Flush the tracking results in the viewer
    vpDisplay::flush(I) ;
  }

  // Create an intrinsic camera parameters structure
  vpCameraParameters cam;

  // Create a robot access
  vpRobotAfma6 robot;

  // Load the end-effector to camera frame transformation obtained
  // using a camera intrinsic model with distortion
  robot.init(vpAfma6::CAMERA_DRAGONFLY2_8MM,
	     vpCameraParameters::perspectiveProjWithDistortion);

  // Get the intrinsic camera parameters associated to the image
  robot.getCameraParameters(cam, I);

  // Using the camera parameters, compute the perspective projection (transform
  // the dot sub-pixel coordinates into coordinates in the camera frame in
  // meter)
  for (int i=0; i < 4; i ++) {
    double x=0, y=0 ; // coordinates of the dots in the camera frame
    // c ----> x
    // |
    // |
    // \/
    // y
    //pixel to meter conversion
    vpPixelMeterConversion::convertPoint(cam,
					 dot[i].get_u(), dot[i].get_v(),
					 x, y);
    target[i].set_x(x) ;
    target[i].set_y(y) ;
  }

  // From now, in target[i], we have the 3D coordinates of a point in the
  // object frame, and their correspondances in the camera frame. We can now
  // compute the pose cMo between the camera and the object.
  vpPose pose;
  // Add the 4 points to compute the pose
  for (int i=0; i < 4; i ++) {
    pose.addPoint(target[i]) ;
  }
  // Create an homogeneous matrix for the camera to object transformation
  // computed just bellow
  vpHomogeneousMatrix cMo;
  vpRotationMatrix    R;
  vpRxyzVector        r;
  // Compute the pose: initialisation is done by Lagrange method, and the final
  // pose is computed by the more accurate Virtual Visual Servoing method.
  pose.computePose(vpPose::LAGRANGE_VIRTUAL_VS, cMo) ;


  std::cout << "Pose cMo: " << std::endl << cMo;
  cMo.extract(R);
  r.buildFrom(R);
  std::cout << "  rotation: "
	    << vpMath::deg(r[0]) << " "
	    << vpMath::deg(r[1]) << " "
	    << vpMath::deg(r[2]) << " deg" << std::endl << std::endl;

  // Get the robot position in the reference frame
  vpHomogeneousMatrix rMc;
  vpColVector p; // position x,y,z,rx,ry,rz
  robot.getPosition(vpRobotAfma6::REFERENCE_FRAME, p);
  std::cout << "Robot pose in reference frame: " << p << std::endl;
  vpTranslationVector t;
  t[0] = p[0]; t[1] = p[1]; t[2] = p[2];
  r[0] = p[3]; r[1] = p[4]; r[2] = p[5];
  R.buildFrom(r);
  rMc.buildFrom(t, R);
  std::cout << "Pose rMc: " << std::endl << rMc;
  rMc.extract(R);
  r.buildFrom(R);
  std::cout << "  rotation: "
	    << vpMath::deg(r[0]) << " "
	    << vpMath::deg(r[1]) << " "
	    << vpMath::deg(r[2]) << " deg" << std::endl << std::endl;

  robot.getPosition(vpRobotAfma6::ARTICULAR_FRAME, p);
  std::cout << "Robot pose in articular: " << p << std::endl;

  robot.computeMGD(p, rMc);
  std::cout << "Pose rMc from MGD: " << std::endl << rMc;
  rMc.extract(R);
  r.buildFrom(R);
  std::cout << "  rotation: "
	    << vpMath::deg(r[0]) << " "
	    << vpMath::deg(r[1]) << " "
	    << vpMath::deg(r[2]) << " deg" << std::endl << std::endl;

  vpHomogeneousMatrix rMo;
  rMo = rMc * cMo;
  std::cout << "Pose rMo = rMc * cMo: " << std::endl << rMo;
  rMo.extract(R);
  r.buildFrom(R);
  std::cout << "  rotation: "
	    << vpMath::deg(r[0]) << " "
	    << vpMath::deg(r[1]) << " "
	    << vpMath::deg(r[2]) << " deg" << std::endl << std::endl;

  return 0;
}
#else
int main()
{
  std::cout << "a test..." << std::endl;
  return 0;
}

#endif
