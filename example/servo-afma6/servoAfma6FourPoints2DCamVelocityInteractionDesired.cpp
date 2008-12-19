/****************************************************************************
 *
 * $Id: servoAfma6FourPoints2DCamVelocityInteractionDesired.cpp,v 1.7 2008-12-19 14:14:45 fspindle Exp $
 *
 * Copyright (C) 1998-2008 Inria. All rights reserved.
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
  \file servoAfma6FourPoints2DCamVelocityInteractionDesired.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the
  Afma6 robot (cartesian robot, with 6 degrees of freedom). The velocity is
  computed in the camera frame.  Visual features are the image coordinates of 4
  vpDot2 points. The interaction matrix is computed using the desired visual
  features.

*/


/*!
  \example servoAfma6FourPoints2DCamVelocityInteractionDesired.cpp

  Example of eye-in-hand control law. We control here a real robot, the Afma6
  robot (cartesian robot, with 6 degrees of freedom). The velocity is computed
  in the camera frame.  Visual features are the image coordinates of 4 vpDot2
  points. The interaction matrix is computed using the desired visual
  features.

*/

#include <stdlib.h>

#include <visp/vpConfig.h>
#include <visp/vpDebug.h> // Debug trace

#if (defined (VISP_HAVE_AFMA6) && defined (VISP_HAVE_DC1394_2))

#include <visp/vp1394TwoGrabber.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>

#include <visp/vpMath.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpPoint.h>
#include <visp/vpServo.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpDot.h>
#include <visp/vpRobotAfma6.h>
#include <visp/vpServoDisplay.h>
#include <visp/vpIoTools.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>

#define L 0.05 // to deal with a 10cm by 10cm square

int
main()
{
  // Log file creation in /tmp/$USERNAME/log.dat
  // This file contains by line:
  // - the 6 computed camera velocities (m/s, rad/s) to achieve the task
  // - the 6 mesured camera velocities (m/s, rad/s)
  // - the 6 mesured joint positions (m, rad)
  // - the 8 values of s - s*
  std::string username;
  // Get the user login name
  vpIoTools::getUserName(username);

  // Create a log filename to save velocities...
  std::string logdirname;
  logdirname ="/tmp/" + username;

  // Test if the output path exist. If no try to create it
  if (vpIoTools::checkDirectory(logdirname) == false) {
    try {
      // Create the dirname
      vpIoTools::makeDirectory(logdirname);
    }
    catch (...) {
      std::cerr << std::endl
	   << "ERROR:" << std::endl;
      std::cerr << "  Cannot create " << logdirname << std::endl;
      exit(-1);
    }
  }
  std::string logfilename;
  logfilename = logdirname + "/log.dat";

  // Open the log file name
  std::ofstream flog(logfilename.c_str());

  try
  {
    vpServo task ;

    vpImage<unsigned char> I ;
    int i ;

    vp1394TwoGrabber g;
    g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
    g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_60);
    g.open(I) ;

    vpDisplayX display(I,100,100,"Current image") ;

    g.acquire(I) ;

    vpDisplay::display(I) ;
    vpDisplay::flush(I) ;

    std::cout << std::endl ;
    std::cout << "-------------------------------------------------------" << std::endl ;
    std::cout << " Test program for vpServo "  <<std::endl ;
    std::cout << " Eye-in-hand task control, velocity computed in the camera frame" << std::endl ;
    std::cout << " Use of the Afma6 robot " << std::endl ;
    std::cout << " Interaction matrix computed with the desired features " << std::endl ;

    std::cout << " task : servo 4 points on a square with dimention " << L << " meters" << std::endl ;
    std::cout << "-------------------------------------------------------" << std::endl ;
    std::cout << std::endl ;


    vpDot2 dot[4] ;

    std::cout << "Click on the 4 dots clockwise starting from upper/left dot..."
	      << std::endl;
    for (i=0 ; i < 4 ; i++) {
      dot[i].initTracking(I) ;
      vpDisplay::displayCross(I,
			      (unsigned int)dot[i].get_v(),
			      (unsigned int)dot[i].get_u(),
			      10, vpColor::blue) ;
      vpDisplay::flush(I);
    }

    vpRobotAfma6 robot;

    vpCameraParameters::vpCameraParametersProjType
      projModel = vpCameraParameters::perspectiveProjWithDistortion;

    // Load the end-effector to camera frame transformation obtained
    // using a camera intrinsic model with distortion
    robot.init(vpAfma6::CAMERA_DRAGONFLY2_8MM, projModel);

    vpCameraParameters cam ;
    // Update camera parameters
    robot.getCameraParameters (cam, I);

    // Sets the current position of the visual feature
    vpFeaturePoint p[4] ;
    for (i=0 ; i < 4 ; i++)
      vpFeatureBuilder::create(p[i], cam, dot[i]);  //retrieve x,y  of the vpFeaturePoint structure

    // Set the position of the square target in a frame which origin is
    // centered in the middle of the square
    vpPoint point[4] ;
    point[0].setWorldCoordinates(-L, -L, 0) ;
    point[1].setWorldCoordinates( L, -L, 0) ;
    point[2].setWorldCoordinates( L,  L, 0) ;
    point[3].setWorldCoordinates(-L,  L, 0) ;

    // Initialise a desired pose to compute s*, the desired 2D point features
    vpHomogeneousMatrix cMo;
    vpTranslationVector cto(0, 0, 0.7); // tz = 0.7 meter
    vpRxyzVector cro(vpMath::rad(0), vpMath::rad(0), vpMath::rad(0)); // No rotations
    vpRotationMatrix cRo(cro); // Build the rotation matrix
    cMo.buildFrom(cto, cRo); // Build the homogeneous matrix

    // sets the desired position of the 2D visual feature
    vpFeaturePoint pd[4] ;
    // Compute the desired position of the features from the desired pose
    for (int i=0; i < 4; i ++) {
      vpColVector cP, p ;
      point[i].changeFrame(cMo, cP) ;
      point[i].projection(cP, p) ;

      pd[i].set_x(p[0]) ;
      pd[i].set_y(p[1]) ;
      pd[i].set_Z(cP[2]);
    }

    // Define the task
    // - we want an eye-in-hand control law
    // - robot is controlled in the camera frame
    // - Interaction matrix is computed with the desired visual features
    task.setServo(vpServo::EYEINHAND_CAMERA) ;
    task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);

    // We want to see a point on a point
    std::cout << std::endl ;
    for (i=0 ; i < 4 ; i++)
      task.addFeature(p[i],pd[i]) ;

    // Set the proportional gain
    task.setLambda(0.4) ;

    // Display task information
    task.print() ;

    // Initialise the velocity control of the robot
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

    std::cout << "\nHit CTRL-C to stop the loop...\n" << std::flush;

    while (1) {
      // Acquire a new image from the camera
      g.acquire(I) ;

      // Display this image
      vpDisplay::display(I) ;

      // For each point...
      for (i=0 ; i < 4 ; i++) {
	// Achieve the tracking of the dot in the image
	dot[i].track(I) ;
	// Display a green cross at the center of gravity position in the
	// image
	vpDisplay::displayCross(I,
				(unsigned int)dot[i].get_v(),
				(unsigned int)dot[i].get_u(),
				10, vpColor::green) ;
      }

      // Printing on stdout concerning task information
      // task.print() ;

      // Update the point feature from the dot location
      for (i=0 ; i < 4 ; i++)
	vpFeatureBuilder::create(p[i], cam, dot[i]);

      vpColVector v ;
      // Compute the visual servoing skew vector
      v = task.computeControlLaw() ;

      // Display the current and desired feature points in the image display
      vpServoDisplay::display(task, cam, I);

      // Apply the computed camera velocities to the robot
      robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

      // Save velocities applied to the robot in the log file
      // v[0], v[1], v[2] correspond to camera translation velocities in m/s
      // v[3], v[4], v[5] correspond to camera rotation velocities in rad/s
      flog << v[0] << " " << v[1] << " " << v[2] << " "
	   << v[3] << " " << v[4] << " " << v[5] << " ";

      // Get the measured joint velocities of the robot
      vpColVector qvel;
      robot.getVelocity(vpRobot::CAMERA_FRAME, qvel);
      // Save measured camera velocities of the robot in the log file:
      // - qvel[0], qvel[1], qvel[2] correspond to measured camera translation
      //   velocities in m/s
      // - qvel[3], qvel[4], qvel[5] correspond to measured camera rotation
      //   velocities in rad/s 
      flog << qvel[0] << " " << qvel[1] << " " << qvel[2] << " "
	   << qvel[3] << " " << qvel[4] << " " << qvel[5] << " ";

      // Get the measured joint positions of the robot
      vpColVector q;
      robot.getPosition(vpRobot::ARTICULAR_FRAME, q);
      // Save measured joint positions of the robot in the log file
      // - q[0], q[1], q[2] correspond to measured joint translation
      //   positions in m
      // - q[3], q[4], q[5] correspond to measured joint rotation
      //   positions in rad
      flog << q[0] << " " << q[1] << " " << q[2] << " "
	   << q[3] << " " << q[4] << " " << q[5] << " ";

      // Save feature error (s-s*) for the 4 feature points. For each feature
      // point, we have 2 errors (along x and y axis).  This error is expressed
      // in meters in the camera frame
      flog << task.error[0] << " " << task.error[1] << " " // s-s* for point 1
	   << task.error[2] << " " << task.error[3] << " " // s-s* for point 2
	   << task.error[4] << " " << task.error[5] << " " // s-s* for point 3
	   << task.error[6] << " " << task.error[7]        // s-s* for point 4
	   << std::endl;

      // Flush the display
      vpDisplay::flush(I) ;
    }

    flog.close() ; // Close the log file

    // Display task information
    task.print() ;

    // Kill the task
    task.kill();

    return 0;
  }
  catch (...) {
    flog.close() ; // Close the log file
    vpERROR_TRACE(" Test failed") ;
    return 0;
  }
}

#else
int
main()
{
  vpERROR_TRACE("You do not have an afma6 robot or a firewire framegrabber connected to your computer...");

}

#endif
