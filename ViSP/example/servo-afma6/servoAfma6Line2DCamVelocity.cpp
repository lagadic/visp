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
  \file servoAfma6Line2DCamVelocity.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the
  Afma6 robot (cartesian robot, with 6 degrees of freedom). The velocity is
  computed in the camera frame.  The visual feature is a line.
*/


/*!
  \example servoAfma6Line2DCamVelocity.cpp

  Example of eye-in-hand control law. We control here a real robot, the Afma6
  robot (cartesian robot, with 6 degrees of freedom). The velocity is computed
  in the camera frame.  The visual feature is a line.
*/



#include <visp/vpConfig.h>
#include <visp/vpDebug.h> // Debug trace
#include <stdlib.h>
#if (defined (VISP_HAVE_AFMA6) && defined (VISP_HAVE_DC1394_2))

#include <visp/vp1394TwoGrabber.h>
#include <visp/vpImage.h>
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


      vpDisplayX display(I,100,100,"Example using one line.cpp ") ;
      vpTRACE(" ") ;

      vpDisplay::display(I) ;
      vpDisplay::flush(I) ;


      vpServo task ;


      std::cout << std::endl ;
      std::cout << "-------------------------------------------------------" << std::endl ;
      std::cout << " Test program for vpServo "  <<std::endl ;
      std::cout << " Eye-in-hand task control, velocity computed in the camera frame" << std::endl ;
      std::cout << " Simulation " << std::endl ;
      std::cout << " task : servo a line " << std::endl ;
      std::cout << "-------------------------------------------------------" << std::endl ;
      std::cout << std::endl ;


      vpMeLine line ;

      vpMe me ;
      me.setRange(10) ;
      me.setPointsToTrack(100) ;
      me.setThreshold(100000) ;
      me.setSampleStep(10);
      line.setDisplay(vpMeSite::RANGE_RESULT) ;


      line.setMe(&me) ;

      //Initialize the tracking. Define the line to track.
      line.initTracking(I) ;
      line.track(I) ;

      vpRobotAfma6 robot ;
      //  robot.move("pos-init.pos") ;

      vpCameraParameters cam ;
      // Update camera parameters
      robot.getCameraParameters (cam, I);

      vpTRACE("sets the current position of the visual feature ") ;
      vpFeatureLine p ;
      vpFeatureBuilder::create(p,cam, line)  ;

      vpTRACE("sets the desired position of the visual feature ") ;
      vpLine lined;
      lined.setWorldCoordinates(1,0,0,0,0,0,1,0);
      vpHomogeneousMatrix cMo(0,0,0.3,0,0,vpMath::rad(0));
      lined.project(cMo);
      lined.setRho(-fabs(lined.getRho()));
      lined.setTheta(0);

      vpFeatureLine pd ;
      vpFeatureBuilder::create(pd,lined);

      vpTRACE("define the task") ;
      vpTRACE("\t we want an eye-in-hand control law") ;
      vpTRACE("\t robot is controlled in the camera frame") ;
      task.setServo(vpServo::EYEINHAND_CAMERA) ;

      vpTRACE("\t we want to see a point on a point..") ;
      std::cout << std::endl ;
      task.addFeature(p,pd) ;

      vpTRACE("\t set the gain") ;
      task.setLambda(0.2) ;


      vpTRACE("Display task information " ) ;
      task.print() ;


      robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

      int iter=0 ;
      vpTRACE("\t loop") ;
      vpColVector v ;
      while(1)
	{
	  std::cout << "---------------------------------------------" << iter <<std::endl ;

	  try {
	    g.acquire(I) ;
	    vpDisplay::display(I) ;

	    //Track the line
	    line.track(I) ;
	    line.display(I, vpColor::red) ;

	    //Update the current line feature
	    vpFeatureBuilder::create(p,cam,line);

	    //displqy the current and the desired features
	    p.display(cam, I,  vpColor::red) ;
	    pd.display(cam, I,  vpColor::green) ;

	    v = task.computeControlLaw() ;

	    vpDisplay::flush(I) ;
	    if (iter==0)  vpDisplay::getClick(I) ;
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
