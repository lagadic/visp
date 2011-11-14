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
  \file servoAfma6SquareLines2DCamVelocity.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the Afma6
  robot (cartesian robot, with 6 degrees of freedom). The velocity is computed
  in the camera frame. Visual features are the four lines of a square.

*/

/*!
  \example servoAfma6SquareLines2DCamVelocity.cpp

  Example of eye-in-hand control law. We control here a real robot, the Afma6
  robot (cartesian robot, with 6 degrees of freedom). The velocity is computed
  in the camera frame. Visual features are the four lines of a square.

*/



#include <visp/vpConfig.h>
#include <visp/vpDebug.h> // Debug trace
#include <stdlib.h>
#include <cmath>    // std::fabs
#include <limits>   // numeric_limits
#if (defined (VISP_HAVE_AFMA6) && defined (VISP_HAVE_DC1394_2))

#include <visp/vp1394TwoGrabber.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeatureLine.h>
#include <visp/vpLine.h>
#include <visp/vpMeLine.h>
#include <visp/vpServo.h>
#include <visp/vpFeatureBuilder.h>

#include <visp/vpRobotAfma6.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpServoDisplay.h>

#include <visp/vpDot.h>

int
main()
{
  try
    {
      vpImage<unsigned char> I ;

      vp1394TwoGrabber g;
      g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
      g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_60);
      g.open(I) ;

      g.acquire(I) ;


      vpDisplayX display(I,100,100,"test SquareLines.cpp ") ;
      vpTRACE(" ") ;

      vpDisplay::display(I) ;
      vpDisplay::flush(I);

      vpServo task ;


      std::cout << std::endl ;
      std::cout << "-------------------------------------------------------" << std::endl ;
      std::cout << " Test program for vpServo "  <<std::endl ;
      std::cout << " Eye-in-hand task control, velocity computed in the camera frame" << std::endl ;
      std::cout << " Simulation " << std::endl ;
      std::cout << " task : servo a line " << std::endl ;
      std::cout << "-------------------------------------------------------" << std::endl ;
      std::cout << std::endl ;

      int i ;
      int nbline =4 ;

      vpMeLine line[nbline] ;

      vpMe me ;
      me.setRange(10) ;
      me.setPointsToTrack(100) ;
      me.setThreshold(50000) ;
      me.setSampleStep(10);

      //Initialize the tracking. Define the four lines to track.
      for (i=0 ; i < nbline ; i++)
	{
	  line[i].setDisplay(vpMeSite::RANGE_RESULT) ;
	  line[i].setMe(&me) ;

	  line[i].initTracking(I) ;
	  line[i].track(I) ;
	}

      vpRobotAfma6 robot ;
      //robot.move("zero.pos") ;

      vpCameraParameters cam ;
      // Update camera parameters
      robot.getCameraParameters (cam, I);

      vpTRACE("sets the current position of the visual feature ") ;
      vpFeatureLine p[nbline] ;
      for (i=0 ; i < nbline ; i++)
	vpFeatureBuilder::create(p[i],cam, line[i])  ;

      vpTRACE("sets the desired position of the visual feature ") ;
      vpLine lined[nbline];
      lined[0].setWorldCoordinates(1,0,0,0.05,0,0,1,0);
      lined[1].setWorldCoordinates(0,1,0,0.05,0,0,1,0);
      lined[2].setWorldCoordinates(1,0,0,-0.05,0,0,1,0);
      lined[3].setWorldCoordinates(0,1,0,-0.05,0,0,1,0);

      vpHomogeneousMatrix cMo(0,0,0.5,0,0,vpMath::rad(0));

      lined[0].project(cMo);
      lined[1].project(cMo);
      lined[2].project(cMo);
      lined[3].project(cMo);

      //Those lines are needed to keep the conventions define in vpMeLine (Those in vpLine are less restrictive)
      //Another way to have the coordinates of the desired features is to learn them before executing the program.
      lined[0].setRho(-fabs(lined[0].getRho()));
      lined[0].setTheta(0);
      lined[1].setRho(-fabs(lined[1].getRho()));
      lined[1].setTheta(M_PI/2);
      lined[2].setRho(-fabs(lined[2].getRho()));
      lined[2].setTheta(M_PI);
      lined[3].setRho(-fabs(lined[3].getRho()));
      lined[3].setTheta(-M_PI/2);

      vpFeatureLine pd[nbline] ;

      vpFeatureBuilder::create(pd[0],lined[0]);
      vpFeatureBuilder::create(pd[1],lined[1]);
      vpFeatureBuilder::create(pd[2],lined[2]);
      vpFeatureBuilder::create(pd[3],lined[3]);

      vpTRACE("define the task") ;
      vpTRACE("\t we want an eye-in-hand control law") ;
      vpTRACE("\t robot is controlled in the camera frame") ;
      task.setServo(vpServo::EYEINHAND_CAMERA) ;
      task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);

      vpTRACE("\t we want to see a point on a point..") ;
      std::cout << std::endl ;
      for (i=0 ; i < nbline ; i++)
	task.addFeature(p[i],pd[i]) ;

      vpTRACE("\t set the gain") ;
      task.setLambda(0.2) ;


      vpTRACE("Display task information " ) ;
      task.print() ;

      robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

      int iter=0 ;
      vpTRACE("\t loop") ;
      vpColVector v ;
      vpImage<vpRGBa> Ic ;
      double lambda_av =0.05;
      double alpha = 0.05 ;
      double beta =3 ;

      while(1)
	{
	  std::cout << "---------------------------------------------" << iter <<std::endl ;

	  try {
	    g.acquire(I) ;
	    vpDisplay::display(I) ;

	    //Track the lines and update the features
	    for (i=0 ; i < nbline ; i++)
	      {
		      line[i].track(I) ;
		      line[i].display(I, vpColor::red) ;

		      vpFeatureBuilder::create(p[i],cam,line[i]);

		      p[i].display(cam, I,  vpColor::red) ;
		      pd[i].display(cam, I,  vpColor::green) ;
	      }

	    double gain ;
	    {
	      if (std::fabs(alpha) <= std::numeric_limits<double>::epsilon()) 
		gain = lambda_av ;
	      else
		    {
		      gain = alpha * exp (-beta * task.error.sumSquare() ) +  lambda_av ;
		    }
	    }

	    task.setLambda(gain) ;

	    v = task.computeControlLaw() ;

            vpDisplay::flush(I) ;

	    if (iter==0)  vpDisplay::getClick(I) ;
	    if (v.sumSquare() > 0.5)
	      {
		      v =0 ;
		      robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;
		      robot.stopMotion() ;
		      vpDisplay::getClick(I) ;
	      }

	    robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

	  }
	  catch(...)
	    {
	      v =0 ;
	      robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;
	      robot.stopMotion() ;
	      exit(1) ;
	    }

	  vpTRACE("\t\t || s - s* || = %f ", task.error.sumSquare()) ;
	  iter++;
	}

      vpTRACE("Display task information " ) ;
      task.print() ;
      task.kill();
    }
  catch (...)
    {
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
