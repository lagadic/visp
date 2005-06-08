

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      test1ServoPoint.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: testServoSquareLine.cpp,v 1.1.1.1 2005-06-08 07:08:15 fspindle Exp $
 *
 * Description
 * ============
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in the camera frame
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \example test1ServoPoint.cpp
  \brief tests the control law, eye-in-hand control, velocity computed in the camera frame
*/

#include <visp/vpConfig.h>

#ifdef HAVE_ROBOT_AFMA6

#include <visp/vpIcComp.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpPoint.h>
#include <visp/vpServo.h>
#include <visp/vpFeatureBuilder.h>

#include <visp/vpRobotAfma6.h>
#include <visp/afma_main.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpServoDisplay.h>

// Debug trace
#include <visp/vpDebug.h>

#include <visp/vpDot.h>

int
main()
{
  vpRobotAfma6 robot ;
  robot.move("zero.pos") ;

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


  cout << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << " Test program for vpServo "  <<endl ;
  cout << " Eye-in-hand task control, velocity computed in the camera frame" << endl ;
  cout << " Simulation " << endl ;
  cout << " task : servo a point " << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << endl ;


  int nbline =4 ;
  vpMeLine line[nbline] ;
  int i ;
  vpMe me ;
  me.setRange(10) ;
  me.setPointsToTrack(60) ;
  me.setThreshold(15000) ;

  for (i=0 ; i < nbline ; i++)
    {
      line[i].setDisplay(vpMeTracker::RANGE_RESULT) ;
      line[i].setMe(&me) ;

      try{
	line[i].initTracking(I) ;
	line[i].track(I) ;
      }
      catch(...)
	{
	  ERROR_TRACE(" ") ;
	  throw ;
	}
    }

  vpCameraParameters cam ;

  TRACE("sets the current position of the visual feature ") ;
  vpFeatureLine p[nbline] ;
  for (i=0 ; i < nbline ; i++)
    vpFeatureBuilder::create(p[i],cam, line[i])  ;

  TRACE("sets the desired position of the visual feature ") ;
  vpFeatureLine pd[nbline] ;
   // vertical BW
  pd[0].setRhoTheta(0.15,M_PI) ;
  pd[0].setABCD(0,0,1,-1) ; //z = 1

  //horizontal BW
  pd[1].setRhoTheta(-0.15,M_PI/2) ;
  pd[1].setABCD(0.0,0,1,-1) ; //z = 1

  // vertical WB
  pd[2].setRhoTheta(0.15,0) ;
  pd[2].setABCD(0,0,1,-1) ; //z = 1

  // horizontal
  pd[3].setRhoTheta(-0.15,-M_PI/2) ;
  pd[3].setABCD(0.0,0,1,-1) ; //z = 1

  TRACE("define the task") ;
  TRACE("\t we want an eye-in-hand control law") ;
  TRACE("\t robot is controlled in the camera frame") ;
  task.setServo(vpServo::EYEINHAND_CAMERA) ;

  TRACE("\t we want to see a point on a point..") ;
  cout << endl ;
  for (i=0 ; i < nbline ; i++)
    task.addFeature(p[i],pd[i]) ;

  TRACE("\t set the gain") ;
  task.setLambda(0.032) ;


  TRACE("Display task information " ) ;
  task.print() ;

  vpDisplay::getClick(I) ;
  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

  int iter=0 ;
  TRACE("\t loop") ;
  vpColVector v ;
  char s[FILENAME_MAX] ;
  vpImage<vpRGBa> Ic ;
  double lambda_av =0.05;
  double alpha = 0.05 ; //1 ;
  double beta =3 ; //3 ;

  while(1)
  {
    cout << "---------------------------------------------" << iter <<endl ;

    try {
      g.acquire(I) ;
      vpDisplay::display(I) ;
      sprintf(s,"/tmp/marchand/image.%04d.ppm",iter) ;
      vpImageIo::writePGM(I,s) ;
      for (i=0 ; i < nbline ; i++)
	{
	  line[i].track(I) ;
	  line[i].display(I, vpColor::red) ;
	  
	  //    vpDisplay::displayCross(I,(int)line.I(), (int)line.J(),
	  //			   10,vpColor::green) ;
	  
	  vpFeatureBuilder::create(p[i],cam,line[i]);
	  //  TRACE("%f %f ",p[i].getRho(), p[i].getTheta()) ;
	  
	  p[i].display(cam, I,  vpColor::red) ;
	}
      double gain ;
      {
	if (alpha == 0) gain = lambda_av ;
	else
	  {
	    gain = alpha * exp (-beta * task.error.sumSquare() ) +  lambda_av ;
	  }
      }
      
      task.setLambda(gain) ;
      v = task.computeControlLaw() ;
      
      //  cout << task.error.t() << endl ;

      vpServoDisplay::display(task,cam,I) ;
         cout << v.sumSquare() <<endl  ;
      if (iter==0)  vpDisplay::getClick(I) ;
      if (v.sumSquare() > 0.5)
	{
	  v =0 ;
	  robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;
	  robot.stopMotion() ; 
	  vpDisplay::getClick(I) ;
	}

     robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

     
    }
    catch(...)
      {
	v =0 ;
	robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;
	robot.stopMotion() ;
	exit(1) ;
      }
    //   vpDisplay::getImage(I,Ic) ;
    //   vpImageIo::writePPM(Ic,s) ;

    TRACE("\t\t || s - s* || = %f ", task.error.sumSquare()) ;
    iter++;
  }

  TRACE("Display task information " ) ;
  task.print() ;
}

#else
int
main()
{
}

#endif
