

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      $RCSfile: servoAfma6FourPointCameraVelocity.cpp,v $
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: servoAfma6FourPointCameraVelocity.cpp,v 1.5 2006-10-18 13:42:54 mtallur Exp $
 *
 * Description
 * ============
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in the camera frame
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*!
  \example servoAfma6FourPointCameraVelocity.cpp

  Example of eye-in-hand control law. We control here a real robot, the Afma6
  robot (cartesian robot, with 6 degrees of freedom). The velocity is computed
  in the camera frame.  Visual features are the image coordinates of 4 vdot points.

*/


#include <visp/vpConfig.h>
#include <visp/vpDebug.h> // Debug trace

#if (defined (VISP_HAVE_AFMA6) && defined (VISP_HAVE_ICCOMP))

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
  try 
  {
    vpImage<unsigned char> I ;
    int i ;

    vpIcCompGrabber g(2) ;
    g.open(I) ;

    g.acquire(I) ;


    vpDisplayX display(I,100,100,"testDisplayX.cpp ") ;
    vpTRACE(" ") ;

    vpDisplay::display(I) ;


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

    for (i=0 ; i < 4 ; i++)
      dot[i].initTracking(I) ;

    vpCameraParameters cam ;

    vpTRACE("sets the current position of the visual feature ") ;
    vpFeaturePoint p[4] ;
    for (i=0 ; i < 4 ; i++)
      vpFeatureBuilder::create(p[i],cam, dot[i])  ;  //retrieve x,y  of the vpFeaturePoint structure

    vpTRACE("sets the desired position of the visual feature ") ;
    vpFeaturePoint pd[4] ;
#define L 0.075
#define D 0.5
    pd[0].buildFrom(-L,-L,D) ;
    pd[1].buildFrom(L,-L,D) ;
    pd[2].buildFrom(L,L,D) ;
    pd[3].buildFrom(-L,L,D) ;

    vpTRACE("define the task") ;
    vpTRACE("\t we want an eye-in-hand control law") ;
    vpTRACE("\t robot is controlled in the camera frame") ;
    task.setServo(vpServo::EYEINHAND_CAMERA) ;
    task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE) ;

    vpTRACE("\t we want to see a point on a point..") ;
    cout << endl ;
    for (i=0 ; i < 4 ; i++)
      task.addFeature(p[i],pd[i]) ;

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

      vpTRACE("\t\t || s - s* || = %f ", task.error.sumSquare()) ;
    }

    vpTRACE("Display task information " ) ;
    task.print() ;
  }
  catch (...)
  {
    vpERROR_TRACE(" Test failed") ;
    return 0;
  }
}

#else
int
main()
{
  vpERROR_TRACE("You do not have an afma6 robot or an ICcomp framegrabber connected to your computer...");

}

#endif
