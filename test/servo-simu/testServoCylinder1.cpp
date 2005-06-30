

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      test1ServoCylinder.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand 20-3-2005
 *
 * Version control
 * ===============
 *
 *  $Id: testServoCylinder1.cpp,v 1.2 2005-06-30 08:22:37 fspindle Exp $
 *
 * Description
 * ============
 *   Servo a cylinder
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \example testServoCylinder1.cpp
  \brief Servo a cylinder
*/


#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeatureLine.h>
#include <visp/vpCylinder.h>
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
  cout << " task : servo a cylinder " << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << endl ;


  TRACE("sets the initial camera location " ) ;
  vpHomogeneousMatrix cMo(-0.2,0.1,2,
			  vpMath::rad(5),  vpMath::rad(5),  vpMath::rad(20));

  robot.setPosition(cMo) ;

  TRACE("sets the final camera location (for simulation purpose)" ) ;
  vpHomogeneousMatrix cMod(0,0,1,
			  vpMath::rad(-60),  vpMath::rad(0),  vpMath::rad(0));



  TRACE("sets the cylinder coordinates in the world frame "  ) ;
  vpCylinder cylinder(0,1,0,  // direction
		      0,0,0,  // point of the axis
		      0.1) ;  // radius

  TRACE("sets the desired position of the visual feature ") ;
  cylinder.track(cMod) ;
  cylinder.print() ;

  vpFeatureLine ld[2] ;
  int i ;
  for(i=0 ; i < 2 ; i++)
    vpFeatureBuilder::create(ld[i],cylinder,i)  ;


  TRACE("project : computes  the cylinder coordinates in the camera frame and its 2D coordinates"  ) ;
  TRACE("sets the current position of the visual feature ") ;
  cylinder.track(cMo) ;
  cylinder.print() ;

  vpFeatureLine l[2] ;
  for(i=0 ; i < 2 ; i++)
  {
    vpFeatureBuilder::create(l[i],cylinder,i)  ;
    l[i].print() ;
  }

  TRACE("define the task") ;
  TRACE("\t we want an eye-in-hand control law") ;
  TRACE("\t robot is controlled in the camera frame") ;
  task.setServo(vpServo::EYEINHAND_CAMERA) ;
  // task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE) ;
 //  it can also be interesting to test these possibilities
  //    task.setInteractionMatrixType(vpServo::MEAN, vpServo::PSEUDO_INVERSE) ;
   task.setInteractionMatrixType(vpServo::DESIRED,vpServo::PSEUDO_INVERSE) ;
  //task.setInteractionMatrixType(vpServo::DESIRED,  vpServo::TRANSPOSE) ;
  // task.setInteractionMatrixType(vpServo::CURRENT,  vpServo::TRANSPOSE) ;

  TRACE("\t we want to see  2 lines on 2 lines.") ;

  task.addFeature(l[0],ld[0]) ;
  task.addFeature(l[1],ld[1]) ;

  vpServoDisplay::display(task,cam,I) ;


  TRACE("Display task information " ) ;
  task.print() ;


  TRACE("\n\nClick in the camera view window to start...");
  vpDisplay::getClick(I) ;
  TRACE("\t set the gain") ;
  task.setLambda(0.1) ;


  TRACE("Display task information " ) ;
  task.print() ;

  int iter=0 ;
  TRACE("\t loop") ;
  do
  {
    cout << "---------------------------------------------" << iter++ <<endl ;
    vpColVector v ;

    if (iter==1) TRACE("\t\t get the robot position ") ;
    robot.getPosition(cMo) ;
    if (iter==1) TRACE("\t\t new line position ") ;
    //retrieve x,y and Z of the vpLine structure

    cylinder.track(cMo) ;
    //    cylinder.print() ;
    for(i=0 ; i < 2 ; i++)
    {
      vpFeatureBuilder::create(l[i],cylinder,i)  ;
      //   l[i].print() ;
    }


    vpDisplay::display(I) ;
    vpServoDisplay::display(task,cam,I) ;


    if (iter==1) TRACE("\t\t compute the control law ") ;
    v = task.computeControlLaw() ;

    if (iter==1) TRACE("\t\t send the camera velocity to the controller ") ;
    robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

    TRACE("\t\t || s - s* || ") ;
    cout << task.error.sumSquare() <<endl ; ;

    //   vpDisplay::getClick(I) ;
  }
  while(task.error.sumSquare() >  1e-9) ;

  TRACE("\n\nClick in the camera view window to end...");
  vpDisplay::getClick(I) ;
  TRACE("Display task information " ) ;
  task.print() ;
}

