/****************************************************************************
 *
 * $Id: servoAfma6FourPoints2DArtVelocity.cpp,v 1.1 2007-01-29 09:36:09 asaunier Exp $
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
 *   velocity computed in the articular frame
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \example servoAfma6FourPoints2DArtVelocity.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the Afma6
  robot (cartesian robot, with 6 degrees of freedom). The velocity is computed
  in articular.  Visual features are the image coordinates of 4 vdot points.

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
      vpImage<unsigned char> I ;
      int i ;

      vpIcCompGrabber g(2) ;
      g.open(I) ;

      g.acquire(I) ;

      vpDisplayX display(I,100,100,"testDisplayX.cpp ") ;
      vpTRACE(" ") ;

      vpDisplay::display(I) ;


      vpServo task ;

      vpRobotAfma6 robot ;


      cout << endl ;
      cout << "-------------------------------------------------------" << endl ;
      cout << " Test program for vpServo "  <<endl ;
      cout << " Eye-in-hand task control, velocity computed in the camera frame" << endl ;
      cout << " Simulation " << endl ;
      cout << " task : servo a point " << endl ;
      cout << "-------------------------------------------------------" << endl ;
      cout << endl ;


      vpDot dot[4] ;

      for (i=0 ; i < 4 ; i++)
	dot[i].initTracking(I) ;

      vpCameraParameters cam ;

      vpTRACE("sets the current position of the visual feature ") ;
      vpFeaturePoint p[4] ;
      for (i=0 ; i < 4 ; i++)
	vpFeatureBuilder::create(p[i],cam, dot[i])  ;  //retrieve x,y and Z of the vpPoint structure

      vpTRACE("sets the desired position of the visual feature ") ;
      vpFeaturePoint pd[4] ;

#define L 0.075
#define D 0.5

      pd[0].buildFrom(-L,-L,D) ;
      pd[1].buildFrom(L,-L,D) ;
      pd[2].buildFrom(L,L,D) ;
      pd[3].buildFrom(-L,L,D) ;

      vpTRACE("\t we want to see a point on a point..") ;
      cout << endl ;
      for (i=0 ; i < 4 ; i++)
	task.addFeature(p[i],pd[i]) ;

      vpTRACE("\t set the gain") ;
      task.setLambda(0.2) ;


      vpTRACE("Display task information " ) ;
      task.print() ;

      vpTRACE("define the task") ;
      vpTRACE("\t we want an eye-in-hand control law") ;
      vpTRACE("\t articular velocity are computed") ;
      task.setServo(vpServo::EYEINHAND_L_cVe_eJe) ;
      task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE) ;
      task.print() ;

      vpTwistMatrix cVe ;
      robot.get_cVe(cVe) ;
      task.set_cVe(cVe) ;
      task.print() ;

      vpTRACE("Set the Jacobian (expressed in the end-effector frame)") ;
      vpMatrix eJe ;
      robot.get_eJe(eJe) ;
      task.set_eJe(eJe) ;
      task.print() ;

      robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

      int iter=0 ;
      vpTRACE("\t loop") ;
      while(1)
	{
	  cout << "---------------------------------------------" << iter <<endl ;

	  g.acquire(I) ;
	  vpDisplay::display(I) ;

	  try
	    {
	      for (i=0 ; i < 4 ; i++)
		dot[i].track(I) ;
	    }
	  catch(...)
	    {
	      vpTRACE("Error detected while tracking visual features") ;
	      robot.stopMotion() ;
	      exit(1) ;
	    }
	  //    vpDisplay::displayCross(I,(int)dot.I(), (int)dot.J(),
	  //			   10,vpColor::green) ;


	  for (i=0 ; i < 4 ; i++)
	    vpFeatureBuilder::create(p[i],cam, dot[i]);


	  // get the jacobian
	  robot.get_eJe(eJe) ;
	  task.set_eJe(eJe) ;

	  vpColVector v ;
	  v = task.computeControlLaw() ;

	  vpServoDisplay::display(task,cam,I) ;
	  cout << v.t() ;
	  robot.setVelocity(vpRobot::ARTICULAR_FRAME, v) ;

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
  vpERROR_TRACE("You do not have an afma6 robot or an ICcomp framegrabber connected to your computer...");

}

#endif
