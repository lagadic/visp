/****************************************************************************
 *
 * $Id: servoAfma6FourPoints2DCamVelocityInteractionDesired.cpp,v 1.2 2008-01-31 14:42:44 asaunier Exp $
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
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in the camera frame
 *
 * Authors:
 * Eric Marchand
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

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>


int
main()
{
  try
    {
      vpCameraParameters::vpCameraParametersProjType
          projModel = vpCameraParameters::perspectiveProjWithDistortion;
      vpRobotAfma6 robot;

	    // Load the end-effector to camera frame transformation obtained
	    // using a camera intrinsic model with distortion
	    robot.init(vpAfma6::CAMERA_DRAGONFLY2_8MM, projModel);

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
      std::cout << " Simulation " << std::endl ;
      std::cout << " task : servo a point " << std::endl ;
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

      vpCameraParameters cam ;
      // Update camera parameters
      robot.getCameraParameters (cam, I);

      vpTRACE("sets the current position of the visual feature ") ;
      vpFeaturePoint p[4] ;
      for (i=0 ; i < 4 ; i++)
        vpFeatureBuilder::create(p[i], cam, dot[i]);  //retrieve x,y  of the vpFeaturePoint structure

      // Set the position of the square target in a frame which origin is
      // centered in the middle of the square
      vpPoint point[4] ;
#define L 0.05 // to deal with a 10cm by 10cm square
      point[0].setWorldCoordinates(-L, -L, 0) ;
      point[1].setWorldCoordinates( L, -L, 0) ;
      point[2].setWorldCoordinates( L,  L, 0) ;
      point[3].setWorldCoordinates(-L,  L, 0) ;

      // Initialise a desired pose to compute s*, the desired 2D point features
      vpHomogeneousMatrix cMo;
      vpTranslationVector cto(0, 0, 0.7); // tz = 1 meter
      vpRxyzVector cro(vpMath::rad(0), vpMath::rad(0), vpMath::rad(0)); // No rotations
      vpRotationMatrix cRo(cro); // Build the rotation matrix
      cMo.buildFrom(cRo, cto); // Build the homogeneous matrix

      vpTRACE("sets the desired position of the 2D visual feature ");
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

      vpTRACE("define the task") ;
      vpTRACE("\t we want an eye-in-hand control law") ;
      vpTRACE("\t robot is controlled in the camera frame") ;
      task.setServo(vpServo::EYEINHAND_CAMERA) ;
      task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);

      vpTRACE("\t we want to see a point on a point..") ;
      std::cout << std::endl ;
      for (i=0 ; i < 4 ; i++)
        task.addFeature(p[i],pd[i]) ;

      vpTRACE("\t set the gain") ;
      task.setLambda(0.6) ;

      vpTRACE("Display task information " ) ;
      task.print() ;

      robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

      int iter=0 ;
      double error = 1;
      vpTRACE("\t loop") ;
      while(error > 0.000001 ) {
        std::cout << "-------------------------------" << iter <<std::endl ;

        g.acquire(I) ;
        vpDisplay::display(I) ;

        for (i=0 ; i < 4 ; i++) {
	        dot[i].track(I) ;
	        vpDisplay::displayCross(I,
				        (unsigned int)dot[i].get_v(),
				        (unsigned int)dot[i].get_u(),
				        10, vpColor::green) ;
        }


        task.print() ;

        for (i=0 ; i < 4 ; i++)
	        vpFeatureBuilder::create(p[i],cam, dot[i]);

        vpColVector v ;
        v = task.computeControlLaw() ;

        vpServoDisplay::display(task, cam, I);
        //	v = 0;
        std::cout << "Velocity: " <<v.t() ;

        robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

        vpDisplay::flush(I) ;

        error = task.error.sumSquare();
        vpTRACE("\t\t || s - s* || = %g ", error) ;

        iter ++;
      }

      vpTRACE("Display task information " ) ;
      task.print() ;
      task.kill();
    }
  catch (...) {
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
