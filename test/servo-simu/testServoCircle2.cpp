

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      testServoSphere1.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: testServoCircle2.cpp,v 1.4 2006-04-19 09:01:24 fspindle Exp $
 *
 * Description
 * ============
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in the camera frame
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \example testServoCircle1.cpp
  \brief servo a circle
*/

#include <visp/vpDebug.h>
#include <visp/vpConfig.h>

#ifdef VISP_HAVE_X11

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeatureEllipse.h>
#include <visp/vpCircle.h>
#include <visp/vpServo.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpFeatureBuilder.h>


// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>

// Debug trace
#include <visp/vpDebug.h>

#include <visp/vpServoDisplay.h>

#include <visp/vpImage.h>
#include <visp/vpDisplayX.h>
#include <visp/vpCameraParameters.h>


int
main()
{


  vpImage<unsigned char> I(512,512,0) ;
  vpDisplayX display(I,100,100, "Camera view") ;

  vpCameraParameters cam ;
  double px, py ; px = py = 600 ;
  double u0, v0 ; u0 = v0 = 256 ;

  cam.init(px,py,u0,v0);

  vpServo task ;
  vpRobotCamera robot ;

  cout << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << " Test program for vpServo "  <<endl ;
  cout << " Simulation " << endl ;
  cout << " task : servo a circle " << endl ;
  cout << " display the task " << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << endl ;


  TRACE("sets the initial camera location " ) ;
  vpHomogeneousMatrix cMo(0,0,1,
			  vpMath::rad(0),  vpMath::rad(80),  vpMath::rad(30))   ;
  robot.setPosition(cMo) ;

  vpHomogeneousMatrix cMod(-0.1,-0.1,0.7,
			  vpMath::rad(40),  vpMath::rad(10),  vpMath::rad(30))   ;



  TRACE("sets the circle coordinates in the world frame "  ) ;
  vpCircle circle ;
  circle.setWorldCoordinates(0,0,1,
			     0,0,0,
			     0.1) ;

  TRACE("sets the desired position of the visual feature ") ;
  vpFeatureEllipse pd ;
  circle.track(cMod) ;
  vpFeatureBuilder::create(pd,circle)  ;

  TRACE("project : computes  the circle coordinates in the camera frame and its 2D coordinates"  ) ;

  TRACE("sets the current position of the visual feature ") ;
  vpFeatureEllipse p ;
  circle.track(cMo) ;
  vpFeatureBuilder::create(p,circle)  ;

  TRACE("define the task") ;
  TRACE("\t we want an eye-in-hand control law") ;
  TRACE("\t robot is controlled in the camera frame") ;
  task.setServo(vpServo::EYEINHAND_CAMERA) ;
  task.setInteractionMatrixType(vpServo::DESIRED) ;
  TRACE("\t we want to see a circle on a circle..") ;
  cout << endl ;
  task.addFeature(p,pd) ;

  TRACE("\t set the gain") ;
  task.setLambda(0.1) ;


  TRACE("Display task information " ) ;
  task.print() ;

  int iter=0 ;
  TRACE("\t loop") ;
  while(iter++<200)
  {
    cout << "---------------------------------------------" << iter <<endl ;
    vpColVector v ;

    if (iter==1) TRACE("\t\t get the robot position ") ;
    robot.getPosition(cMo) ;
    if (iter==1) TRACE("\t\t new circle position ") ;
    //retrieve x,y and Z of the vpCircle structure
    circle.track(cMo) ;
    vpFeatureBuilder::create(p,circle);
    circle.print() ;
    p.print() ;

    vpDisplay::display(I) ;
    vpServoDisplay::display(task,cam,I) ;

    if (iter==1) TRACE("\t\t compute the control law ") ;
    v = task.computeControlLaw() ;
    //  TRACE("computeControlLaw" ) ;
    cout << task.rankJ1 <<endl ;
    if (iter==1) TRACE("\t\t send the camera velocity to the controller ") ;
    robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;
  }

  TRACE("Display task information " ) ;
  task.print() ;
}
#else
int
main()
{
  ERROR_TRACE("You do not have X11 functionalities to display images...");
}

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
