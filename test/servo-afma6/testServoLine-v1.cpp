

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      test1ServoPoint.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: testServoLine-v1.cpp,v 1.3 2005-06-29 15:57:35 fspindle Exp $
 *
 * Description
 * ============
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in the camera frame
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \example test1ServoPoint.cpp
  \brief tests the control law, eye-in-hand control, velocity computed in the camera frame
*/

#include <visp/vpConfig.h>

#ifdef HAVE_ROBOT_AFMA6

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
#include <visp/afma_main.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpServoDisplay.h>

// Debug trace
#include <visp/vpDebug.h>

#include <visp/vpDot.h>

int
main()
{
  vpRobotAfma6 robot ;
  robot.move("pos-init.pos") ;

  vpImage<unsigned char> I ;


  vpIcCompGrabber g(2) ;
  g.open(I) ;

  try{
    g.acquire(I) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }


  vpDisplayX display(I,100,100,"testDisplayX.cpp ") ;
  TRACE(" ") ;

  try{
    vpDisplay::display(I) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }


  vpServo task ;


  cout << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << " Test program for vpServo "  <<endl ;
  cout << " Eye-in-hand task control, velocity computed in the camera frame" << endl ;
  cout << " Simulation " << endl ;
  cout << " task : servo a point " << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << endl ;


  vpMeLine line ;

  vpMe me ;
  me.setRange(10) ;
  me.setPointsToTrack(60) ;
  me.setThreshold(15000) ;
  line.setDisplay(vpMeTracker::RANGE_RESULT) ;


  line.setMe(&me) ;

  try{
    line.initTracking(I) ;
    line.track(I) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

  vpCameraParameters cam ;

  TRACE("sets the current position of the visual feature ") ;
  vpFeatureLine p ;
  vpFeatureBuilder::create(p,cam, line)  ;

  TRACE("sets the desired position of the visual feature ") ;
  vpFeatureLine pd ;
  pd.setRhoTheta(0,0) ;
  pd.setABCD(0,0,1,-1) ; //z = 1

  TRACE("define the task") ;
  TRACE("\t we want an eye-in-hand control law") ;
  TRACE("\t robot is controlled in the camera frame") ;
  task.setServo(vpServo::EYEINHAND_CAMERA) ;

  TRACE("\t we want to see a point on a point..") ;
  cout << endl ;
  task.addFeature(p,pd) ;

  TRACE("\t set the gain") ;
  task.setLambda(0.2) ;


  TRACE("Display task information " ) ;
  task.print() ;


  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

  int iter=0 ;
  TRACE("\t loop") ;
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
      TRACE("%f %f ",line.getRho(), line.getTheta()) ;

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

    TRACE("\t\t || s - s* || = %f ", task.error.sumSquare()) ;
    iter++;
  }

  TRACE("Display task information " ) ;
  task.print() ;
}

#else
int
main()
{
  ERROR_TRACE("You do not have an afma6 robot connected to your computer...");
}

#endif
