

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
 *  $Id: servoBiclopsPointArticularVelocity.cpp,v 1.6 2006-06-23 14:45:07 brenier Exp $
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

#ifdef UNIX
#  include <unistd.h>
#endif
#include <signal.h>

#include <visp/vpConfig.h>
#include <visp/vpDebug.h> // Debug trace

#if ( defined (VISP_HAVE_BICLOPS) & defined (VISP_HAVE_DC1394) )

#ifdef VISP_HAVE_PTHREAD
#  include <pthread.h>
#endif

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
#include <visp/vpIoTools.h>
#include <visp/vpServoDisplay.h>
#include <visp/vpDot2.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>


#ifdef VISP_HAVE_PTHREAD
pthread_mutex_t mutexEndLoop = PTHREAD_MUTEX_INITIALIZER;
#endif

void signalCtrC( int signumber )
{
#ifdef VISP_HAVE_PTHREAD
  pthread_mutex_unlock( &mutexEndLoop );
#endif
  usleep( 1000*10 );
  vpTRACE("Ctrl-C pressed...");
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

#ifdef VISP_HAVE_PTHREAD
  pthread_mutex_lock( &mutexEndLoop );
#endif
  signal( SIGINT,&signalCtrC );

  // Get the user login name
  char *user;
  user = getlogin();

  // Set debug directory to /tmp/$user
  char *debugdir = new char[FILENAME_MAX];
  sprintf(debugdir, "/tmp/%s", user);

  // If the debug dir don't exist, create it
  try {
    vpIoTools::checkDirectory(debugdir);
  }
  catch (...) {
    vpIoTools::makeDirectory(debugdir);
  }

  // Create the debub file: /tmp/$user/biclops.txt
  char *filename = new char[FILENAME_MAX];
  sprintf(filename, "%s/biclops.txt", debugdir);
  FILE *fd = fopen(filename, "w");

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
    vpERROR_TRACE(" ") ;
    throw ;
  }


  vpDisplayX display(I,100,100,"testDisplayX.cpp ") ;
  vpTRACE(" ") ;

  try{
    vpDisplay::display(I) ;
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }


  vpServo task ;

  vpDot2 dot ;

  try{
    cout << "Click on a dot to initialize the tracking..." << endl;
    dot.setGraphics(true);
    dot.initTracking(I) ;
    dot.track(I);
    vpERROR_TRACE("after dot.initTracking(I) ") ;
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }

  vpCameraParameters cam ;

  vpTRACE("sets the current position of the visual feature ") ;
  vpFeaturePoint p ;
  vpFeatureBuilder::create(p,cam, dot)  ;  //retrieve x,y and Z of the vpPoint structure

  p.set_Z(1) ;
  vpTRACE("sets the desired position of the visual feature ") ;
  vpFeaturePoint pd ;
  pd.buildFrom(0,0,1) ;

  vpTRACE("define the task") ;
   vpTRACE("\t we want an eye-in-hand control law") ;
  vpTRACE("\t articular velocity are computed") ;
  task.setServo(vpServo::EYEINHAND_L_cVe_eJe) ;
  task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE) ;


  vpTRACE("Set the position of the camera in the end-effector frame ") ;
  vpHomogeneousMatrix cMe ;
  //  robot.get_cMe(cMe) ;

  vpTwistMatrix cVe ;
  robot.get_cVe(cVe) ;
  cout << cVe <<endl ;
  task.set_cVe(cVe) ;

  cout << "Click in the image to start the servoing..." << endl;
  vpDisplay::getClick(I) ;
  vpTRACE("Set the Jacobian (expressed in the end-effector frame)") ;
  vpMatrix eJe ;
  robot.get_eJe(eJe) ;
  task.set_eJe(eJe) ;


  vpTRACE("\t we want to see a point on a point..") ;
  cout << endl ;
  task.addFeature(p,pd) ;

  vpTRACE("\t set the gain") ;
  task.setLambda(0.1) ;


  vpTRACE("Display task information " ) ;
  task.print() ;


  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

  int iter=0 ;
  vpTRACE("\t loop") ;
#ifdef VISP_HAVE_PTHREAD
  while( 0 != pthread_mutex_trylock( &mutexEndLoop ) )
#else
  while (1)
#endif
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

    vpTRACE("\t\t || s - s* || = %f ", task.error.sumSquare()) ;
    {
      vpColVector s_minus_sStar(2);
      s_minus_sStar = task.s - task.sStar;
      fprintf(fd, "%f %f %f %f %f\n",
	      v[0], v[1],
	      s_minus_sStar[0], s_minus_sStar[1],
	      task.error.sumSquare());
    }
  }

  vpTRACE("Display task information " ) ;
  task.print() ;

  delete [] debugdir;
  delete [] filename;

  fclose(fd);

  } catch (...) { vpERROR_TRACE("Trow uncatched..."); }

}


#else
int
main()
{
  vpERROR_TRACE("You don't have a biclops head connected to your computer or 1394 framegrabbing capabilities...");
}
#endif
