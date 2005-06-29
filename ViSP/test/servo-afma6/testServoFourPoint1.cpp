

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
 *  $Id: testServoFourPoint1.cpp,v 1.4 2005-06-29 15:57:35 fspindle Exp $
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
  vpImage<unsigned char> I ;
  int i ;

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

  try{
    for (i=0 ; i < 4 ; i++)
      dot[i].initTracking(I) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

  vpCameraParameters cam ;

  TRACE("sets the current position of the visual feature ") ;
  vpFeaturePoint p[4] ;
  for (i=0 ; i < 4 ; i++)
    vpFeatureBuilder::create(p[i],cam, dot[i])  ;  //retrieve x,y  of the vpFeaturePoint structure

  TRACE("sets the desired position of the visual feature ") ;
  vpFeaturePoint pd[4] ;
#define L 0.075
#define D 0.5
  pd[0].buildFrom(-L,-L,D) ;
  pd[1].buildFrom(L,-L,D) ;
  pd[2].buildFrom(L,L,D) ;
  pd[3].buildFrom(-L,L,D) ;

  TRACE("define the task") ;
  TRACE("\t we want an eye-in-hand control law") ;
  TRACE("\t robot is controlled in the camera frame") ;
  task.setServo(vpServo::EYEINHAND_CAMERA) ;
  task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE) ;

  TRACE("\t we want to see a point on a point..") ;
  cout << endl ;
  for (i=0 ; i < 4 ; i++)
    task.addFeature(p[i],pd[i]) ;

  TRACE("\t set the gain") ;
  task.setLambda(0.2) ;


  TRACE("Display task information " ) ;
  task.print() ;


  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

  int iter=0 ;
  TRACE("\t loop") ;
  while(1)
  {
    cout << "---------------------------------------------" << iter <<endl ;

    g.acquire(I) ;
    vpDisplay::display(I) ;

    for (i=0 ; i < 4 ; i++)
      dot[i].track(I) ;

    //    vpDisplay::displayCross(I,(int)dot.I(), (int)dot.J(),
    //			   10,vpColor::green) ;

    task.print() ;

    for (i=0 ; i < 4 ; i++)
      vpFeatureBuilder::create(p[i],cam, dot[i]);


    vpColVector v ;
    v = task.computeControlLaw() ;

    vpServoDisplay::display(task,cam,I) ;
    cout << v.t() ;
    robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

    TRACE("\t\t || s - s* || = %f ", task.error.sumSquare()) ;
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
