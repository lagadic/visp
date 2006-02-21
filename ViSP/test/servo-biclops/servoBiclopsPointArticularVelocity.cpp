

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      $RCSfile: servoBiclopsPointArticularVelocity.cpp,v $
 * Author:    Fabien Spindler
 *
 * Version control
 * ===============
 *
 *  $Id: servoBiclopsPointArticularVelocity.cpp,v 1.1 2006-02-21 11:17:04 fspindle Exp $
 *
 * Description
 * ============
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in articular
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \example servoBiclopsArticularVelocity.cpp

  Example of eye-in-hand control law. We control here a real robot, the biclops
  robot (pan-tilt head provided by Traclabs). The velocity is
  computed in articular. The visual feature is the center of gravity of a
  point.

*/

#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <error.h>

#include <visp/vpConfig.h>
#include <visp/vpDebug.h> // Debug trace

#if ( defined (HAVE_ROBOT_BICLOPS_PT) & defined (HAVE_LIBDC1394_CONTROL) & defined(HAVE_LIBRAW1394) )
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

#include <visp/vpRobotBiclops.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpServoDisplay.h>

#include <visp/vpDot2.h>


pthread_mutex_t mutexEndLoop = PTHREAD_MUTEX_INITIALIZER;

void signalCtrC( int signumber )
{
  pthread_mutex_unlock( &mutexEndLoop );
  usleep( 1000*10 );
  TRACE("Ctrl-C pressed...");
}

int
main()
{
  cout << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << " Test program for vpServo "  <<endl ;
  cout << " Eye-in-hand task control, velocity computed in the camera frame" << endl ;
  cout << " Simulation " << endl ;
  cout << " task : servo a point " << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << endl ;

  try{

  pthread_mutex_lock( &mutexEndLoop );
  signal( SIGINT,&signalCtrC );

  vpRobotBiclops robot ;
  {
    vpColVector q(2); q=0;
    robot.setRobotState(vpRobot::STATE_POSITION_CONTROL) ;
    robot.setPosition( vpRobot::ARTICULAR_FRAME,q );
  }

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

  vpDot2 dot ;

  try{
    ERROR_TRACE("start dot.initTracking(I) ") ;
    int x,y;
    vpDisplay::getClick( I,y,x );
    dot.set_u( x ) ;
    dot.set_v( y ) ;
    DEBUG_TRACE(25,"Click!");
    //dot.initTracking(I) ;
    dot.track(I);
    ERROR_TRACE("after dot.initTracking(I) ") ;
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
  task.setLambda(0.1) ;


  TRACE("Display task information " ) ;
  task.print() ;


  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

  int iter=0 ;
  TRACE("\t loop") ;
  while( 0 != pthread_mutex_trylock( &mutexEndLoop ) )
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

  } catch (...) { ERROR_TRACE("Trow uncatched..."); }
}


#else
int
main()
{
  ERROR_TRACE("You don't have a biclops head connected to your computer or 1394 framegrabbing capabilities...");
}
#endif
