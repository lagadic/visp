/****************************************************************************
 *
 * $Id: servoAfma6TwoLines2DCamVelocity.cpp,v 1.6 2007-09-28 14:46:32 asaunier Exp $
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

\file servoAfma6TwoLines2DCamVelocity.cpp

\brief Example of eye-in-hand control law. We control here a real robot, the Afma6
robot (cartesian robot, with 6 degrees of freedom). The velocity is computed
in the camera frame. Visual features are the two lines.

*/


/*!

\example servoAfma6TwoLines2DCamVelocity.cpp

Example of eye-in-hand control law. We control here a real robot, the Afma6
robot (cartesian robot, with 6 degrees of freedom). The velocity is computed
in the camera frame. Visual features are the two lines.

*/

#include <visp/vpConfig.h>
#include <visp/vpDebug.h> // Debug trace

#if (defined (VISP_HAVE_AFMA6) && defined (VISP_HAVE_ITIFG8))

#include <visp/vpItifg8Grabber.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
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
      //robot.move("zero.pos") ;

      vpImage<unsigned char> I ;


      vpItifg8Grabber g(2) ;
      g.open(I) ;

      g.acquire(I) ;


      vpDisplayX display(I,100,100,"testDisplayX.cpp ") ;
      vpTRACE(" ") ;

      vpDisplay::display(I) ;
      vpDisplay::flush(I) ;


      vpServo task ;


      std::cout << std::endl ;
      std::cout << "-------------------------------------------------------" << std::endl ;
      std::cout << " Test program for vpServo "  <<std::endl ;
      std::cout << " Eye-in-hand task control, velocity computed in the camera frame" << std::endl ;
      std::cout << " Simulation " << std::endl ;
      std::cout << " task : servo a point " << std::endl ;
      std::cout << "-------------------------------------------------------" << std::endl ;
      std::cout << std::endl ;


      int nbline =2 ;
      vpMeLine line[nbline] ;
      int i ;
      vpMe me ;
      me.setRange(10) ;
      me.setPointsToTrack(60) ;
      me.setThreshold(15000) ;

      for (i=0 ; i < nbline ; i++)
	    {
	      line[i].setDisplay(vpMeTracker::RANGE_RESULT) ;
	      line[i].setMe(&me) ;

	      line[i].initTracking(I) ;
	      line[i].track(I) ;
	    }

      vpCameraParameters cam ;

      vpTRACE("sets the current position of the visual feature ") ;
      vpFeatureLine p[nbline] ;
      for (i=0 ; i < nbline ; i++)
      	vpFeatureBuilder::create(p[i],cam, line[i])  ;

      vpTRACE("sets the desired position of the visual feature ") ;
      vpFeatureLine pd[nbline] ;
      pd[0].setRhoTheta(-0.15,0) ;
      pd[0].setABCD(0,0,1,-1) ; //z = 1
      pd[1].setRhoTheta(0.15,0) ;
      pd[1].setABCD(0.0,0,1,-1) ; //z = 1

      vpTRACE("define the task") ;
      vpTRACE("\t we want an eye-in-hand control law") ;
      vpTRACE("\t robot is controlled in the camera frame") ;
      task.setServo(vpServo::EYEINHAND_CAMERA) ;

      vpTRACE("\t we want to see a point on a point..") ;
      std::cout << std::endl ;
      for (i=0 ; i < nbline ; i++)
      	task.addFeature(p[i],pd[i]) ;

      vpTRACE("\t set the gain") ;
      task.setLambda(0.032) ;


      vpTRACE("Display task information " ) ;
      task.print() ;


      robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

      int iter=0 ;
      vpTRACE("\t loop") ;
      vpColVector v ;
      //  char s[FILENAME_MAX] ;
      vpImage<vpRGBa> Ic ;
      double lambda_av =0.05;
      double alpha = 0.2 ; //1 ;
      double beta =3 ; //3 ;

      while(1)
	{
	  std::cout << "---------------------------------------------" << iter <<std::endl ;

	  try {
	    g.acquire(I) ;
	    vpDisplay::display(I) ;

	    for (i=0 ; i < nbline ; i++)
	      {
		      line[i].track(I) ;
		      line[i].display(I, vpColor::red) ;

		      //    vpDisplay::displayCross(I,(int)line.I(), (int)line.J(),
		      //			   10,vpColor::green) ;

		      vpFeatureBuilder::create(p[i],cam,line[i]);
		      vpTRACE("%f %f ",line[i].getRho(), line[i].getTheta()) ;

		      p[i].display(cam, I,  vpColor::red) ;
	      }
      vpDisplay::flush(I) ;
	    double gain ;
	    {
	      if (alpha == 0) gain = lambda_av ;
	      else
		    {
		      gain = alpha * exp (-beta * task.error.sumSquare() ) +  lambda_av ;
		    }
	    }  
	    task.setLambda(gain) ;
	    v = task.computeControlLaw() ;

	    vpServoDisplay::display(task,cam,I) ;
	    //  std::cout << v.t() ;
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
	  //   vpDisplay::getImage(I,Ic) ;
	  // sprintf(s,"/tmp/image.%04d.ppm",iter) ;
	  //   vpImageIo::writePPM(Ic,s) ;

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
  vpERROR_TRACE("You do not have an afma6 robot or an Itifg8 framegrabber connected to your computer...");
}

#endif
