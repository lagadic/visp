

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      testServoLine1.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: testServoLine1.cpp,v 1.2 2005-06-30 08:22:37 fspindle Exp $
 *
 * Description
 * ============
 *   Servo a Line
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \example testServoLine1.cpp
  \brief Servo a line
*/


#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeatureLine.h>
#include <visp/vpLine.h>
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

  // open a display for the visualization

  vpImage<unsigned char> I(512,512,255) ;
  vpDisplayX display(I,100,100,"Camera view") ;

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
  cout << " task : servo a line " << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << endl ;


  TRACE("sets the initial camera location " ) ;
  vpHomogeneousMatrix cMo(-0.2,0.1,1,
			  vpMath::rad(5),  vpMath::rad(5),  vpMath::rad(90));

  robot.setPosition(cMo) ;

  TRACE("sets the final camera location (for simulation purpose)" ) ;
  vpHomogeneousMatrix cMod(0,0,1,
			  vpMath::rad(0),  vpMath::rad(0),  vpMath::rad(0));



  TRACE("sets the line coordinates (2 planes) in the world frame "  ) ;

  vpColVector plane1(4) ;
  vpColVector plane2(4) ;
  plane1[0] = 0;  // z = 0
  plane1[1] = 0;
  plane1[2] = 1;
  plane1[3] = 0;
  plane2[0] = 0; // y  =0
  plane2[1] = 1;
  plane2[2] = 0;
  plane2[3] = 0;


  vpLine line ;
  line.setWorldCoordinates(plane1, plane2) ;

  TRACE("sets the desired position of the visual feature ") ;
  line.track(cMod) ;
  line.print() ;

  vpFeatureLine ld ;
  vpFeatureBuilder::create(ld,line)  ;


  TRACE("project : computes  the line coordinates in the camera frame and its 2D coordinates"  ) ;
  TRACE("sets the current position of the visual feature ") ;
  line.track(cMo) ;
  line.print() ;

  vpFeatureLine l ;
  vpFeatureBuilder::create(l,line)  ;
  l.print() ;

  TRACE("define the task") ;
  TRACE("\t we want an eye-in-hand control law") ;
  TRACE("\t robot is controlled in the camera frame") ;
  task.setServo(vpServo::EYEINHAND_CAMERA) ;

  TRACE("\t we want to see a line on a line..\n") ;

  task.addFeature(l,ld) ;
  vpServoDisplay::display(task,cam,I) ;

  TRACE("\t set the gain") ;
  task.setLambda(1.) ;


  TRACE("Display task information " ) ;
  task.print() ;

  int iter=0 ;
  TRACE("\t loop") ;
  while(iter++<100)
  {
    cout << "---------------------------------------------" << iter <<endl ;
    vpColVector v ;

    if (iter==1) TRACE("\t\t get the robot position ") ;
    robot.getPosition(cMo) ;
    if (iter==1) TRACE("\t\t new line position ") ;
    //retrieve x,y and Z of the vpLine structure

    line.track(cMo) ;
    vpFeatureBuilder::create(l,line);

    vpDisplay::display(I) ;
    vpServoDisplay::display(task,cam,I) ;

    if (iter==1) TRACE("\t\t compute the control law ") ;
    v = task.computeControlLaw() ;

    if (iter==1) TRACE("\t\t send the camera velocity to the controller ") ;
    robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

    TRACE("\t\t || s - s* || ") ;
    cout << task.error.sumSquare() <<endl ; ;

  }

  TRACE("Display task information " ) ;
  task.print() ;
}

