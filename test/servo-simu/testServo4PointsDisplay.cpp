

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      testServo4Points.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: testServo4PointsDisplay.cpp,v 1.4 2006-04-19 09:01:24 fspindle Exp $
 *
 * Description
 * ============
 *   tests the control law
 *   eye-in-hand control
 *   articular velocity are computed
 *   servo on 4 points
 *   interaction computed as the mean of the current and desired interaction
 *   matrix
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \example testServo4Points.cpp
  \brief tests the control law, eye-in-hand control,  articular velocity are computed, servo on 4 points

  interaction computed as the mean of the current and desired interaction
  matrix

*/

#include <visp/vpDebug.h>
#include <visp/vpConfig.h>

#ifdef VISP_HAVE_X11

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpServo.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpDebug.h>
#include <visp/vpFeatureBuilder.h>

#include <visp/vpServoDisplay.h>
#include <visp/vpProjectionDisplay.h>

#include <visp/vpImage.h>
#include <visp/vpDisplayX.h>
#include <visp/vpCameraParameters.h>

int
main()
{

  // open a display for the visualization

  vpImage<unsigned char> I(300,300,0) ;
  vpDisplayX display(I,0,0, "Internal view") ;
  vpImage<unsigned char> Iext(300,300,0) ;
  vpDisplayX displayExt(Iext,330,000, "External view") ;

  vpProjectionDisplay externalview ;

  vpCameraParameters cam ;
  double px, py ; px = py = 600 ;
  double u0, v0 ; u0 = v0 = 150 ;

  cam.init(px,py,u0,v0);

  int i ;
  vpServo task ;
  vpRobotCamera robot ;


  cout << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << " Test program for vpServo "  <<endl ;
  cout << " Eye-in-hand task control,  articular velocity are computed" << endl ;
  cout << " Simulation " << endl ;
  cout << " task : servo 4 points " << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << endl ;


  TRACE("sets the initial camera location " ) ;
  vpHomogeneousMatrix cMo(-0.1,-0.1,1,
			  vpMath::rad(40),  vpMath::rad(10),  vpMath::rad(60))   ;

  robot.setPosition(cMo) ;

  vpHomogeneousMatrix cextMo(0,0,2,
			     0,0,0) ;//vpMath::rad(40),  vpMath::rad(10),  vpMath::rad(60))   ;


  TRACE("sets the point coordinates in the world frame "  ) ;
  vpPoint point[4] ;
  point[0].setWorldCoordinates(-0.1,-0.1,0) ;
  point[1].setWorldCoordinates(0.1,-0.1,0) ;
  point[2].setWorldCoordinates(0.1,0.1,0) ;
  point[3].setWorldCoordinates(-0.1,0.1,0) ;


  for (i = 0 ; i < 4 ; i++)
    externalview.insert(point[i]) ;

  TRACE("project : computes  the point coordinates in the camera frame and its 2D coordinates"  ) ;
  for (i = 0 ; i < 4 ; i++)
    point[i].track(cMo) ;

  TRACE("sets the desired position of the point ") ;
  vpFeaturePoint p[4] ;
  for (i = 0 ; i < 4 ; i++)
    vpFeatureBuilder::create(p[i],point[i])  ;  //retrieve x,y and Z of the vpPoint structure


  TRACE("sets the desired position of the point ") ;
  vpFeaturePoint pd[4] ;

  pd[0].buildFrom(-0.1,-0.1,1) ;
  pd[1].buildFrom(0.1,-0.1,1) ;
  pd[2].buildFrom(0.1,0.1,1) ;
  pd[3].buildFrom(-0.1,0.1,1) ;

  TRACE("define the task") ;
  TRACE("\t we want an eye-in-hand control law") ;
  TRACE("\t articular velocity are computed") ;
  task.setServo(vpServo::EYEINHAND_L_cVe_eJe) ;
  task.setInteractionMatrixType(vpServo::MEAN) ;


  TRACE("Set the position of the camera in the end-effector frame ") ;
  vpHomogeneousMatrix cMe ;
  vpTwistMatrix cVe(cMe) ;
  task.set_cVe(cVe) ;

  TRACE("Set the Jacobian (expressed in the end-effector frame)") ;
  vpMatrix eJe ;
  robot.get_eJe(eJe) ;
  task.set_eJe(eJe) ;

  TRACE("\t we want to see a point on a point..") ;
  for (i = 0 ; i < 4 ; i++)
    task.addFeature(p[i],pd[i]) ;

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


     if (iter==1)
     {
       TRACE("Set the Jacobian (expressed in the end-effector frame)") ;
       TRACE("since q is modified eJe is modified") ;
     }
    robot.get_eJe(eJe) ;
    task.set_eJe(eJe) ;


    if (iter==1) TRACE("\t\t get the robot position ") ;
    robot.getPosition(cMo) ;
    if (iter==1) TRACE("\t\t new point position ") ;
    for (i = 0 ; i < 4 ; i++)
    {
      point[i].track(cMo) ;
      //retrieve x,y and Z of the vpPoint structure
      vpFeatureBuilder::create(p[i],point[i])  ;

    }

    vpServoDisplay::display(task,cam,I) ;
    externalview.display(Iext,cextMo, cMo, cam, vpColor::green) ;

    if (iter==1) TRACE("\t\t compute the control law ") ;
    v = task.computeControlLaw() ;

    if (iter==1)
    {
      TRACE("Display task information " ) ;
      task.print() ;
    }

    if (iter==1) TRACE("\t\t send the camera velocity to the controller ") ;
    robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

    TRACE("\t\t || s - s* || ") ;
    cout << task.error.sumSquare() <<endl ; ;
  }

  TRACE("Display task information " ) ;
  task.print() ;

#if 0
  // suppressed for automate test
  TRACE("\n\nClick in the internal view window to end...");
  vpDisplay::getClick(I) ;
#endif
}
#else
int
main()
{
  ERROR_TRACE("You do not have X11 functionalities to display images...");
}

#endif
