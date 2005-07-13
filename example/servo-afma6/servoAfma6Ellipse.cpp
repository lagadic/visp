

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      $RCSfile: servoAfma6Ellipse.cpp,v $
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: servoAfma6Ellipse.cpp,v 1.1 2005-07-13 09:02:24 fspindle Exp $
 *
 * Description
 * ============
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in the camera frame
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \example servoAfma6Ellipse.cpp

  Example of eye-in-hand control law. We control here a real robot, the Afma6
  robot (cartesian robot, with 6 degrees of freedom). The velocity is computed
  in the camera frame. The used visual feature is a circle.

*/

#include <visp/vpConfig.h>
#include <visp/vpDebug.h> // Debug trace

#ifdef HAVE_ROBOT_AFMA6

#include <visp/vpIcCompGrabber.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeatureEllipse.h>
#include <visp/vpServo.h>
#include <visp/vpFeatureBuilder.h>

#include <visp/vpRobotAfma6.h>
#include <visp/afma_main.h>

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
    ERROR_TRACE(" ") ;
    throw ;
  }


  vpDisplayX display(I,0,0,"testServoEllipse.cpp ") ;
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


  vpDot dot ;

  try{
     dot.setNbMaxPoint(1e6) ;
     // dot.setGraphics(true) ;
    dot.setComputeMoments(true) ;
    dot.initTracking(I) ;
    dot.track(I) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

  vpCameraParameters cam ;

  TRACE("sets the current position of the visual feature ") ;
  vpFeatureEllipse c ;
  vpFeatureBuilder::create(c,cam, dot)  ;


  cout << " Learning 0/1 " <<endl ;
  int learning ;
  cin >> learning ;
  char name[FILENAME_MAX] ;
  sprintf(name,"dat/ellipse.dat") ;
  if (learning ==1)
    {
      // save the object position
      TRACE("Save the location of the object in a file dat/ellipse.dat") ;
      ofstream f(name) ;
      f << c.get_s().t() ;
      f.close() ;
      exit(1) ;
    }


  TRACE("sets the desired position of the visual feature ") ;
  vpFeatureEllipse cd ;
  ifstream f("dat/ellipse.dat") ;
  double x,y,mu20,mu11,mu02 ;
  f >> x ;   f >> y ;  f >> mu20 ;  f >> mu11 ;  f >> mu02 ;
  f.close() ;
  cd.buildFrom(x,y,mu20,mu11,mu02) ;
  cd.setABC(0,0,10) ;

  task.setServo(vpServo::EYEINHAND_CAMERA) ;
  task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE) ;

  task.addFeature(c,cd) ;

  task.setLambda(0.01) ;

  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;
  int iter=0 ;
  double lambda_av =0.01;
  double alpha = 0.1 ; //1 ;
  double beta =3 ; //3 ;

  cout << "alpha 0.7" << endl;
  cin >> alpha ;
  cout << "beta 5" << endl;
  cin >> beta ;
  while(1)
  {
    cout << "---------------------------------------------" << iter++ <<endl ;

    g.acquire(I) ;
    vpDisplay::display(I) ;

    dot.track(I) ;

    vpFeatureBuilder::create(c,cam, dot);
    // Compute the adaptative gain (speed up the convergence)
    double gain ;
    if (iter>2)
      {
	if (alpha == 0) gain = lambda_av ;
	else
	  {
	    gain = alpha * exp (-beta * task.error.sumSquare() ) +  lambda_av;
	  }
      }
    else gain = lambda_av ;


    TRACE("%f %f", task.error.sumSquare(),  gain) ;
    task.setLambda(gain) ;
    vpColVector v ;
    v = task.computeControlLaw() ;
    cout <<"rank " << task.rankJ1 << endl ;
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
