

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      $RCSfile: servoPtu46PointArticularVelocity.cpp,v $
 * Author:    Fabien Spindler
 *
 * Version control
 * ===============
 *
 *  $Id: servoPtu46PointArticularVelocity.cpp,v 1.1 2006-01-16 09:57:08 fspindle Exp $
 *
 * Description
 * ============
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in articular
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \example servoPtu46PointArticularVelocity.cpp

  Example of eye-in-hand control law. We control here a real robot, the ptu-46
  robot (pan-tilt turret provided by Directed Perception). The velocity is
  computed in articular. The visual feature is the center of gravity of a
  point.

*/


#include <visp/vpConfig.h>
#include <visp/vpDebug.h> // Debug trace

#ifdef HAVE_ROBOT_PTUEVI
#include <visp/vp1394Grabber.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpPoint.h>
#include <visp/vpServo.h>
#include <visp/vpFeatureBuilder.h>

#include <visp/vpRobotPtu46.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpServoDisplay.h>

#include <visp/vpDot.h>

int
main()
{
  vpImage<unsigned char> I ;


  vp1394Grabber g;

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

  vpRobotPtu46 robot ;

  // exit(1) ;

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
    ERROR_TRACE(" ") ;
    throw ;
  }

  vpCameraParameters cam ;

  TRACE("sets the current position of the visual feature ") ;
  vpFeaturePoint p ;
  vpFeatureBuilder::create(p,cam, dot)  ;  //retrieve x,y and Z of the vpPoint structure

  p.set_Z(1) ;
  TRACE("sets the desired position of the visual feature ") ;
  vpFeaturePoint pd ;
  pd.buildFrom(0,0,1) ;

  TRACE("define the task") ;
   TRACE("\t we want an eye-in-hand control law") ;
  TRACE("\t articular velocity are computed") ;
  task.setServo(vpServo::EYEINHAND_L_cVe_eJe) ;
  task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE) ;


  TRACE("Set the position of the camera in the end-effector frame ") ;
  vpHomogeneousMatrix cMe ;
  //  robot.get_cMe(cMe) ;

  vpTwistMatrix cVe ;
  robot.get_cVe(cVe) ;
  cout << cVe <<endl ;
  task.set_cVe(cVe) ;

  vpDisplay::getClick(I) ;
  TRACE("Set the Jacobian (expressed in the end-effector frame)") ;
  vpMatrix eJe ;
  robot.get_eJe(eJe) ;
  task.set_eJe(eJe) ;


  TRACE("\t we want to see a point on a point..") ;
  cout << endl ;
  task.addFeature(p,pd) ;

  TRACE("\t set the gain") ;
  task.setLambda(0.8) ;


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

    dot.track(I) ;

    //    vpDisplay::displayCross(I,(int)dot.I(), (int)dot.J(),
    //			   10,vpColor::green) ;


    vpFeatureBuilder::create(p,cam, dot);

    // get the jacobian
    robot.get_eJe(eJe) ;
    task.set_eJe(eJe) ;

    //  cout << (vpMatrix)cVe*eJe << endl ;

    vpColVector v ;
    v = task.computeControlLaw() ;

    vpServoDisplay::display(task,cam,I) ;
    cout << v.t() ;
    robot.setVelocity(vpRobot::ARTICULAR_FRAME, v) ;

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
