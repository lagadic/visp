

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      $RCSfile: servoAfma6PointCameraVelocity.cpp,v $
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: servoAfma6PointCameraVelocity.cpp,v 1.3 2006-06-23 14:45:07 brenier Exp $
 *
 * Description
 * ============
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in the camera frame
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \example servoAfma6PointCameraVelocity.cpp

  Example of eye-in-hand control law. We control here a real robot, the Afma6
  robot (cartesian robot, with 6 degrees of freedom). The velocity is computed
  in camera frame. The visual feature is the center of gravity of a point.

*/


#include <visp/vpConfig.h>
#include <visp/vpDebug.h> // Debug trace

#ifdef VISP_HAVE_AFMA6

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
  vpImage<unsigned char> I ;


  vpIcCompGrabber g(2) ;
  g.open(I) ;

  try{
    g.acquire(I) ;
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }


  vpDisplayX display(I,100,100,"testDisplayX.cpp ") ;
  vpTRACE(" ") ;

  try{
    vpDisplay::display(I) ;
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }


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


  vpDot dot ;

  try{
    dot.initTracking(I) ;
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }

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
  cout << endl ;
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
    cout << "---------------------------------------------" << iter <<endl ;

    g.acquire(I) ;
    vpDisplay::display(I) ;

    dot.track(I) ;

    //    vpDisplay::displayCross(I,(int)dot.I(), (int)dot.J(),
    //			   10,vpColor::green) ;


    vpFeatureBuilder::create(p,cam, dot);


    vpColVector v ;
    v = task.computeControlLaw() ;

    vpServoDisplay::display(task,cam,I) ;
    cout << v.t() ;
    robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

    vpTRACE("\t\t || s - s* || = %f ", task.error.sumSquare()) ;
  }

  vpTRACE("Display task information " ) ;
  task.print() ;
}


#else
int
main()
{
  vpERROR_TRACE("You do not have an afma6 robot connected to your computer...");
}
#endif
