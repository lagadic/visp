
/****************************************************************************
 *
 * $Id: servoViper850FourPoints2DCamVelocityKinect.cpp 3530 2012-01-03 10:52:12Z fspindle $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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
 *
 * Description:
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in the camera frame
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \example servoViper850FourPointsKinect.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the
  Viper850 robot (cartesian robot, with 6 degrees of freedom). A kinect is attached to the hand.
  The velocity is computed in the kinect camera frame.  Visual features are the image
  coordinates of 4 points.

*/


#include <visp/vpConfig.h>
#include <visp/vpDebug.h> // Debug trace

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>

#if (defined (VISP_HAVE_VIPER850) && defined (VISP_HAVE_LIBFREENECT_AND_DEPENDENCIES) )


#include <visp/vp1394TwoGrabber.h>
#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpPoint.h>
#include <visp/vpServo.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpIoTools.h>
#include <visp/vpRobotViper850.h>
#include <visp/vpPose.h>
#include <visp/vpKinect.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpServoDisplay.h>

#include <visp/vpDot2.h>
#define L 0.05 // to deal with a 10cm by 10cm square


/*!

  Compute the pose \e cMo from the 3D coordinates of the points \e point and
  their corresponding 2D coordinates \e dot. The pose is computed using a Lowe
  non linear method.

  \param point : 3D coordinates of the points.

  \param dot : 2D coordinates of the points.

  \param ndot : Number of points or dots used for the pose estimation.

  \param cam : Intrinsic camera parameters.

  \param cMo : Homogeneous matrix in output describing the transformation
  between the camera and object frame.

  \param cto : Translation in ouput extracted from \e cMo.

  \param cro : Rotation in ouput extracted from \e cMo.

  \param init : Indicates if the we have to estimate an initial pose with
  Lagrange or Dementhon methods.

*/
void compute_pose(vpPoint point[], vpDot2 dot[], int ndot,
                  vpCameraParameters cam,
                  vpHomogeneousMatrix &cMo,
                  vpTranslationVector &cto,
                  vpRxyzVector &cro, bool init)
{
  vpHomogeneousMatrix cMo_dementhon;  // computed pose with dementhon
  vpHomogeneousMatrix cMo_lagrange;  // computed pose with dementhon
  vpRotationMatrix cRo;
  vpPose pose;
  vpImagePoint cog;
  for (int i=0; i < ndot; i ++) {

    double x=0, y=0;
    cog = dot[i].getCog();
    vpPixelMeterConversion::convertPoint(cam, cog, x, y) ; //pixel to meter conversion
    point[i].set_x(x) ;//projection perspective          p
    point[i].set_y(y) ;
    pose.addPoint(point[i]) ;
  }

  if (init == true) {
    pose.computePose(vpPose::DEMENTHON, cMo_dementhon) ;
    // Compute and return the residual expressed in meter for the pose matrix
    // 'cMo'
    double residual_dementhon = pose.computeResidual(cMo_dementhon);
    pose.computePose(vpPose::LAGRANGE, cMo_lagrange) ;
    double residual_lagrange = pose.computeResidual(cMo_lagrange);

    // Select the best pose to initialize the lowe pose computation
    if (residual_lagrange < residual_dementhon)
      cMo = cMo_lagrange;
    else
      cMo = cMo_dementhon;

  }
  else { // init = false; use of the previous pose to initialise LOWE
    cRo.buildFrom(cro);
    cMo.buildFrom(cto, cRo);
  }
  pose.computePose(vpPose::LOWE, cMo) ;
  cMo.extract(cto);
  cMo.extract(cRo);
  cro.buildFrom(cRo);
}

int main()
{
  // Log file creation in /tmp/$USERNAME/log.dat
  // This file contains by line:
  // - the 6 computed joint velocities (m/s, rad/s) to achieve the task
  // - the 6 mesured joint velocities (m/s, rad/s)
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
      return(-1);
    }
  }
  std::string logfilename;
  logfilename = logdirname + "/log.dat";

  // Open the log file name
  std::ofstream flog(logfilename.c_str());

  try {
    vpRobotViper850 robot ;
    // Load the end-effector to camera frame transformation obtained
    // using a camera intrinsic model with distortion
    vpCameraParameters::vpCameraParametersProjType projModel =
        vpCameraParameters::perspectiveProjWithDistortion;
    robot.init(vpRobotViper850::TOOL_GENERIC_CAMERA, projModel);

    vpServo task ;

    vpImage<unsigned char> I ;
    vpImage<vpRGBa> Irgb;
    int i ;

#ifdef VISP_HAVE_LIBFREENECT_OLD
    // This is the way to initialize Freenect with an old version of libfreenect packages under ubuntu lucid 10.04
    Freenect::Freenect<vpKinect> freenect;
    vpKinect & kinect = freenect.createDevice(0);
#else
    Freenect::Freenect freenect;
    vpKinect & kinect = freenect.createDevice<vpKinect>(0);
#endif

    kinect.start(vpKinect::DMAP_LOW_RES);
    kinect.getRGB(Irgb);
    vpImageConvert::convert(Irgb, I);

#ifdef VISP_HAVE_X11
    vpDisplayX display(I,100,100,"Current image") ;
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV display(I,100,100,"Current image") ;
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK display(I,100,100,"Current image") ;
#endif

    vpDisplay::display(I) ;
    vpDisplay::flush(I) ;

    std::cout << std::endl ;
    std::cout << "-------------------------------------------------------" << std::endl ;
    std::cout << " Test program for vpServo "  <<std::endl ;
    std::cout << " Eye-in-hand task control, velocity computed in the camera space" << std::endl ;
    std::cout << " Use of the Viper850 robot " << std::endl ;
    std::cout << " task : servo 4 points on a square with dimention " << L << " meters" << std::endl ;
    std::cout << "-------------------------------------------------------" << std::endl ;
    std::cout << std::endl ;

    vpDot2 dot[4] ;
    vpImagePoint cog;

    std::cout << "Click on the 4 dots clockwise starting from upper/left dot..."
              << std::endl;

    for (i=0 ; i < 4 ; i++) {
      dot[i].initTracking(I) ;
      cog = dot[i].getCog();
      vpDisplay::displayCross(I, cog, 10, vpColor::blue) ;
      vpDisplay::flush(I);
    }

    // Get Kinect Camera Parameters
    vpCameraParameters cam ;
    //kinect.getRGBCamParameters(cam);

    robot.getCameraParameters(cam, I);

    cam.printParameters();

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
    vpTranslationVector cto(0, 0, 0.5); // tz = 0.5 meter
    vpRxyzVector cro(vpMath::rad(0), vpMath::rad(0), vpMath::rad(0));
    vpRotationMatrix cRo(cro); // Build the rotation matrix
    cMo.buildFrom(cto, cRo); // Build the homogeneous matrix

    // Sets the desired position of the 2D visual feature
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

    // We want to see a point on a point
    for (i=0 ; i < 4 ; i++)
      task.addFeature(p[i],pd[i]) ;

    // Set the proportional gain
    task.setLambda(0.5) ;

    // Display task information
    task.print() ;

    // Define the task
    // - we want an eye-in-hand control law
    // - articular velocity are computed
    task.setServo(vpServo::EYEINHAND_CAMERA) ;
    task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE) ;
    task.print() ;

    // Initialise the velocity control of the robot
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

    std::cout << "\nHit CTRL-C to stop the loop...\n" << std::flush;
    for ( ; ; ) {
      // Acquire a new image from the kinect
      kinect.getRGB(Irgb);
      vpImageConvert::convert(Irgb, I);

      // Display this image
      vpDisplay::display(I) ;

      try {
        // For each point...
        for (i=0 ; i < 4 ; i++) {
          // Achieve the tracking of the dot in the image
          dot[i].track(I) ;
          // Display a green cross at the center of gravity position in the
          // image
          cog = dot[i].getCog();
          vpDisplay::displayCross(I, cog, 10, vpColor::green) ;
        }
      }
      catch(...) {
        flog.close() ; // Close the log file
        vpTRACE("Error detected while tracking visual features") ;
        robot.stopMotion() ;
        kinect.stop();
        return(1) ;
      }

      // During the servo, we compute the pose using LOWE method. For the
      // initial pose used in the non linear minimisation we use the pose
      // computed at the previous iteration.
      compute_pose(point, dot, 4, cam, cMo, cto, cro, false);

      for (i=0 ; i < 4 ; i++) {
        // Update the point feature from the dot location
        vpFeatureBuilder::create(p[i], cam, dot[i]);
        // Set the feature Z coordinate from the pose
        vpColVector cP;
        point[i].changeFrame(cMo, cP) ;

        p[i].set_Z(cP[2]);
      }

      vpColVector v ;
      // Compute the visual servoing skew vector
      v = task.computeControlLaw() ;

      // Display the current and desired feature points in the image display
      vpServoDisplay::display(task,cam,I) ;

      // Apply the computed joint velocities to the robot
      robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

      // Save velocities applied to the robot in the log file
      // v[0], v[1], v[2] correspond to joint translation velocities in m/s
      // v[3], v[4], v[5] correspond to joint rotation velocities in rad/s
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

      // Save feature error (s-s*) for the 4 feature points. For each feature
      // point, we have 2 errors (along x and y axis).  This error is expressed
      // in meters in the camera frame
      flog << ( task.getError() ).t() << std::endl;

      // Flush the display
      vpDisplay::flush(I) ;

      // std::cout << "|| s - s* || = "  << ( task.getError() ).sumSquare() << std::endl;
    }

    kinect.stop();
    std::cout << "Display task information: " << std::endl;
    task.print() ;
    task.kill();
    flog.close() ; // Close the log file
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
int main()
{
  vpERROR_TRACE("You do not have a Viper robot or a kinect connected to your computer...");
}

#endif
//#endif
