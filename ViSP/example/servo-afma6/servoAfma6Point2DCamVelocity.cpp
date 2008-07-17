/****************************************************************************
 *
 * $Id: servoAfma6Point2DCamVelocity.cpp,v 1.10 2008-07-17 20:11:57 fspindle Exp $
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
  \file servoAfma6Point2DCamVelocity.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the
  Afma6 robot (cartesian robot, with 6 degrees of freedom). The velocity is
  computed in camera frame. The visual feature is the center of gravity of a
  point.

*/



/*!
  \example servoAfma6Point2DCamVelocity.cpp

  Example of eye-in-hand control law. We control here a real robot, the Afma6
  robot (cartesian robot, with 6 degrees of freedom). The velocity is computed
  in camera frame. The visual feature is the center of gravity of a point.

*/


#include <visp/vpConfig.h>
#include <visp/vpDebug.h> // Debug trace

#if (defined (VISP_HAVE_AFMA6) && defined (VISP_HAVE_DC1394_2))

#include <visp/vp1394TwoGrabber.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpPoint.h>
#include <visp/vpServo.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpRobotAfma6.h>
#include <visp/vpIoTools.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpServoDisplay.h>

#include <visp/vpDot.h>

int
main()
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
    vpRobotAfma6 robot ;
    vpServo task ;


    vpImage<unsigned char> I ;

    vp1394TwoGrabber g;
    g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
    g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_60);
    g.open(I) ;

    g.acquire(I) ;

    vpDisplayX display(I,100,100,"testDisplayX.cpp ") ;
    vpTRACE(" ") ;

    vpDisplay::display(I) ;
    vpDisplay::flush(I) ;

    std::cout << std::endl ;
    std::cout << "-------------------------------------------------------" << std::endl ;
    std::cout << " Test program for vpServo "  <<std::endl ;
    std::cout << " Eye-in-hand task control, velocity computed in the camera frame" << std::endl ;
    std::cout << " Simulation " << std::endl ;
    std::cout << " task : servo a point " << std::endl ;
    std::cout << "-------------------------------------------------------" << std::endl ;
    std::cout << std::endl ;

    vpDot dot ;

    std::cout << "Click on a dot..." << std::endl;
    dot.initTracking(I) ;
    vpDisplay::displayCross(I,
			    (unsigned int)dot.get_v(),
			    (unsigned int)dot.get_u(),
			    10, vpColor::blue) ;
    vpDisplay::flush(I);


    vpCameraParameters cam ;
    // Update camera parameters
    robot.getCameraParameters (cam, I);

    vpTRACE("sets the current position of the visual feature ") ;
    vpFeaturePoint p ;
    vpFeatureBuilder::create(p,cam, dot)  ;  //retrieve x,y and Z of the vpPoint structure

    vpTRACE("sets the desired position of the visual feature ") ;
    vpFeaturePoint pd ;
    pd.buildFrom(0,0,1) ;

    vpTRACE("define the task") ;
    vpTRACE("\t we want an eye-in-hand control law") ;
    vpTRACE("\t robot is controlled in the camera frame") ;
    task.setServo(vpServo::EYEINHAND_CAMERA) ;

    vpTRACE("\t we want to see a point on a point..") ;
    std::cout << std::endl ;
    task.addFeature(p,pd) ;

    vpTRACE("\t set the gain") ;
    task.setLambda(0.8) ;


    vpTRACE("Display task information " ) ;
    task.print() ;


    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

    std::cout << "\nHit CTRL-C to stop the loop...\n" << std::flush;
    while(1) {
      // Acquire a new image from the camera
      g.acquire(I) ;

      // Display this image
      vpDisplay::display(I) ;

      // Achieve the tracking of the dot in the image
      dot.track(I) ;

      // Display a green cross at the center of gravity position in the image
      vpDisplay::displayCross(I,
			      (unsigned int)dot.get_v(),
			      (unsigned int)dot.get_u(),
			      10, vpColor::green) ;

      // Update the point feature from the dot location
      vpFeatureBuilder::create(p, cam, dot);

      vpColVector v ;
      // Compute the visual servoing skew vector
      v = task.computeControlLaw() ;

      // Display the current and desired feature points in the image display
      vpServoDisplay::display(task, cam, I) ;

      // Apply the computed joint velocities to the robot
      robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

      // Save velocities applied to the robot in the log file
      // v[0], v[1], v[2] correspond to camera translation velocities in m/s
      // v[3], v[4], v[5] correspond to camera rotation velocities in rad/s
      flog << v[0] << " " << v[1] << " " << v[2] << " "
	   << v[3] << " " << v[4] << " " << v[5] << " ";

      // Get the measured joint velocities of the robot
      vpColVector qvel;
      robot.getVelocity(vpRobot::ARTICULAR_FRAME, qvel);
      // Save measured joint velocities of the robot in the log file:
      // - qvel[0], qvel[1], qvel[2] correspond to measured joint translation
      //   velocities in m/s
      // - qvel[3], qvel[4], qvel[5] correspond to measured joint rotation
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

      // Save feature error (s-s*) for the feature point. For this feature
      // point, we have 2 errors (along x and y axis).  This error is expressed
      // in meters in the camera frame
      flog << task.error[0] << " " << task.error[1] << " " // s-s* for point
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
  catch (...)
  {
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
