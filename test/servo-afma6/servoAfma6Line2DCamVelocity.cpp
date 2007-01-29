/****************************************************************************
 *
 * $Id: servoAfma6Line2DCamVelocity.cpp,v 1.1 2007-01-29 09:40:22 asaunier Exp $
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
  \example servoAfma6Line2DCamVelocity.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the Afma6
  robot (cartesian robot, with 6 degrees of freedom). The velocity is computed
  in the camera frame.  The visual feature is a line.
*/

#include <visp/vpConfig.h>
#include <visp/vpDebug.h> // Debug trace

#if (defined (VISP_HAVE_AFMA6) && defined (VISP_HAVE_ICCOMP))

#include <visp/vpIcCompGrabber.h>
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
      //  robot.move("pos-init.pos") ;

      vpImage<unsigned char> I ;


      vpIcCompGrabber g(2) ;
      g.open(I) ;

      g.acquire(I) ;


      vpDisplayX display(I,100,100,"testDisplayX.cpp ") ;
      vpTRACE(" ") ;

      vpDisplay::display(I) ;


      vpServo task ;


      cout << endl ;
      cout << "-------------------------------------------------------" << endl ;
      cout << " Test program for vpServo "  <<endl ;
      cout << " Eye-in-hand task control, velocity computed in the camera frame" << endl ;
      cout << " Simulation " << endl ;
      cout << " task : servo a line " << endl ;
      cout << "-------------------------------------------------------" << endl ;
      cout << endl ;


      vpMeLine line ;

      vpMe me ;
      me.setRange(10) ;
      me.setPointsToTrack(60) ;
      me.setThreshold(15000) ;
      line.setDisplay(vpMeTracker::RANGE_RESULT) ;


      line.setMe(&me) ;

      line.initTracking(I) ;
      line.track(I) ;

      vpCameraParameters cam ;

      vpTRACE("sets the current position of the visual feature ") ;
      vpFeatureLine p ;
      vpFeatureBuilder::create(p,cam, line)  ;

      vpTRACE("sets the desired position of the visual feature ") ;
      vpFeatureLine pd ;
      pd.setRhoTheta(0,0) ;
      pd.setABCD(0,0,1,-1) ; //z = 1

      vpTRACE("define the task") ;
      vpTRACE("\t we want an eye-in-hand control law") ;
      vpTRACE("\t robot is controlled in the camera frame") ;
      task.setServo(vpServo::EYEINHAND_CAMERA) ;

      vpTRACE("\t we want to see a point on a point..") ;
      cout << endl ;
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
	  cout << "---------------------------------------------" << iter <<endl ;

	  try {
	    g.acquire(I) ;
	    vpDisplay::display(I) ;

	    line.track(I) ;
	    line.display(I, vpColor::red) ;

	    //    vpDisplay::displayCross(I,(int)line.I(), (int)line.J(),
	    //			   10,vpColor::green) ;

	    vpFeatureBuilder::create(p,cam,line);
	    vpTRACE("%f %f ",line.getRho(), line.getTheta()) ;

	    p.display(cam, I,  vpColor::red) ;
	    v = task.computeControlLaw() ;

	    vpServoDisplay::display(task,cam,I) ;
	    //  cout << v.t() ;
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
  vpERROR_TRACE("You do not have an afma6 robot or an ICcomp framegrabber connected to your computer...");
}

#endif
