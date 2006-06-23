

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      testServoSphereSecondaryTask.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: testServoSphereSecondaryTask.cpp,v 1.3 2006-06-23 14:45:07 brenier Exp $
 *
 * Description
 * ============
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in the camera frame
 *   a secondary is the added
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \example testServoSphereSecondaryTask.cpp
  \brief servo a sphere, add a secondary task, allows to test the
  secondary task
*/


#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeatureEllipse.h>
#include <visp/vpSphere.h>
#include <visp/vpServo.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpFeatureBuilder.h>


// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>

// Debug trace
#include <visp/vpDebug.h>


int
main()
{
  vpServo task ;
  vpRobotCamera robot ;

  cout << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << " Test program for vpServo "  <<endl ;
  cout << " Simulation " << endl ;
  cout << " task : servo a sphere with a secondary task" << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << endl ;


  vpTRACE("sets the initial camera location " ) ;
  vpHomogeneousMatrix cMo ;
  cMo[0][3] = 0.1 ;
  cMo[1][3] = 0.2 ;
  cMo[2][3] = 2 ;
  robot.setPosition(cMo) ;

  vpHomogeneousMatrix cMod ;
  cMod[0][3] = 0 ;
  cMod[1][3] = 0 ;
  cMod[2][3] = 1 ;



  vpTRACE("sets the sphere coordinates in the world frame "  ) ;
  vpSphere sphere ;
  sphere.setWorldCoordinates(0,0,0,0.1) ;

  vpTRACE("sets the desired position of the visual feature ") ;
  vpFeatureEllipse pd ;
  sphere.track(cMod) ;
  vpFeatureBuilder::create(pd,sphere)  ;

  vpTRACE("project : computes  the sphere coordinates in the camera frame and its 2D coordinates"  ) ;

  vpTRACE("sets the current position of the visual feature ") ;
  vpFeatureEllipse p ;
  sphere.track(cMo) ;
  vpFeatureBuilder::create(p,sphere)  ;

  vpTRACE("define the task") ;
  vpTRACE("\t we want an eye-in-hand control law") ;
  vpTRACE("\t robot is controlled in the camera frame") ;
  task.setServo(vpServo::EYEINHAND_CAMERA) ;

  vpTRACE("\t we want to see a sphere on a sphere..") ;
  cout << endl ;
  task.addFeature(p,pd) ;

  vpTRACE("\t set the gain") ;
  task.setLambda(0.1) ;


  vpTRACE("Display task information " ) ;
  task.print() ;
  // exit(1) ;
  int iter=0 ;
  vpTRACE("\t loop") ;
  while(iter++<500)
  {
    cout << "---------------------------------------------" << iter <<endl ;
    vpColVector v ;

    if (iter==1) vpTRACE("\t\t get the robot position ") ;
    robot.getPosition(cMo) ;
    if (iter==1) vpTRACE("\t\t new sphere position ") ;
    //retrieve x,y and Z of the vpSphere structure

    sphere.track(cMo) ;
    vpFeatureBuilder::create(p,sphere);

    vpColVector de2dt(6) ;
    de2dt[2] = 1 ;    // should be zero in (I-WpW)de2dt
    de2dt[5] = 0.01 ; // should be ok
    de2dt[0] = 0.01 ;  // should generate a motion on (I-WpW)de2dt[4]

    if (iter==1) vpTRACE("\t\t compute the control law ") ;

    v = task.computeControlLaw() ;

    cout << "de2dt :"<< de2dt.t() ;
    vpColVector sec ;
    sec = task.secondaryTask(de2dt) ;
    cout << " (I-WpW)de2dt :"<< sec.t() ;

    if (iter>20)  v += sec ;

    if (iter==1) vpTRACE("\t\t send the camera velocity to the controller ") ;
    robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

    cout << "\t || s - s* || " ;
    cout << task.error.sumSquare() <<endl ;


  }

  vpTRACE("Display task information " ) ;
  task.print() ;
}

