/****************************************************************************
 *
 * $Id: servoAfma6Point2DCamVelocity.cpp,v 1.5 2007-04-20 14:22:15 asaunier Exp $
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

#if (defined (VISP_HAVE_AFMA6) && defined (VISP_HAVE_ITIFG8))

#include <visp/vpItifg8Grabber.h>
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
      vpRobotAfma6 robot ;
      vpServo task ;


      vpImage<unsigned char> I ;

      vpItifg8Grabber g(2) ;
      g.open(I) ;

      g.acquire(I) ;

      vpDisplayX display(I,100,100,"testDisplayX.cpp ") ;
      vpTRACE(" ") ;

      vpDisplay::display(I) ;

      std::cout << std::endl ;
      std::cout << "-------------------------------------------------------" << std::endl ;
      std::cout << " Test program for vpServo "  <<std::endl ;
      std::cout << " Eye-in-hand task control, velocity computed in the camera frame" << std::endl ;
      std::cout << " Simulation " << std::endl ;
      std::cout << " task : servo a point " << std::endl ;
      std::cout << "-------------------------------------------------------" << std::endl ;
      std::cout << std::endl ;

      vpDot dot ;

      std::cout << "Click on a dot..." << endl;
      dot.initTracking(I) ;
      vpDisplay::displayCross(I,
			      (unsigned int)dot.get_v(),
			      (unsigned int)dot.get_u(),
			      10, vpColor::blue) ;
      vpDisplay::flush(I);


      vpCameraParameters cam ;

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
      task.setLambda(0.2) ;


      vpTRACE("Display task information " ) ;
      task.print() ;


      robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

      int iter=0 ;
      vpTRACE("\t loop") ;
      while(1)
	{
	  std::cout << "---------------------------------------------" << iter <<std::endl ;

	  g.acquire(I) ;
	  vpDisplay::display(I) ;

	  dot.track(I) ;
	  vpDisplay::displayCross(I,
				  (unsigned int)dot.get_v(),
				  (unsigned int)dot.get_u(),
				  10, vpColor::green) ;

	  vpFeatureBuilder::create(p,cam, dot);


	  vpColVector v ;
	  v = task.computeControlLaw() ;

	  vpServoDisplay::display(task,cam,I) ;
	  std::cout << v.t() ;
	  robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

	  vpDisplay::flush(I) ;

	  vpTRACE("\t\t || s - s* || = %f ", task.error.sumSquare()) ;
	}

      vpTRACE("Display task information " ) ;
      task.print() ;
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
  vpERROR_TRACE("You do not have an afma6 robot or an Itifg8 framegrabber connected to your computer...");
}
#endif
