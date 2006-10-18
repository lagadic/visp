

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      $RCSfile: servoAfma6TwoLine.cpp,v $
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: servoAfma6TwoLine.cpp,v 1.5 2006-10-18 13:42:54 mtallur Exp $
 *
 * Description
 * ============
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in the camera frame
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!

  \example servoAfma6TwoLine.cpp

  Example of eye-in-hand control law. We control here a real robot, the Afma6
  robot (cartesian robot, with 6 degrees of freedom). The velocity is computed
  in the camera frame. Visual features are the two lines.

*/

#include <visp/vpConfig.h>
#include <visp/vpDebug.h> // Debug trace

#if (defined (VISP_HAVE_AFMA6) && defined (VISP_HAVE_ICCOMP))

#include <visp/vpIcCompGrabber.h>
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
    vpRobotAfma6 robot ;
    //robot.move("zero.pos") ;

    vpImage<unsigned char> I ;


    vpIcCompGrabber g(2) ;
    g.open(I) ;

    g.acquire(I) ;


    vpDisplayX display(I,100,100,"testDisplayX.cpp ") ;
    vpTRACE(" ") ;

    vpDisplay::display(I) ;


    vpServo task ;


    cout << endl ;
    cout << "-------------------------------------------------------" << endl ;
    cout << " Test program for vpServo "  <<endl ;
    cout << " Eye-in-hand task control, velocity computed in the camera frame" << endl ;
    cout << " Simulation " << endl ;
    cout << " task : servo a point " << endl ;
    cout << "-------------------------------------------------------" << endl ;
    cout << endl ;


    int nbline =2 ;
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

      line[i].initTracking(I) ;
      line[i].track(I) ;
    }

    vpCameraParameters cam ;

    vpTRACE("sets the current position of the visual feature ") ;
    vpFeatureLine p[nbline] ;
    for (i=0 ; i < nbline ; i++)
      vpFeatureBuilder::create(p[i],cam, line[i])  ;

    vpTRACE("sets the desired position of the visual feature ") ;
    vpFeatureLine pd[nbline] ;
    pd[0].setRhoTheta(-0.15,0) ;
    pd[0].setABCD(0,0,1,-1) ; //z = 1
    pd[1].setRhoTheta(0.15,0) ;
    pd[1].setABCD(0.0,0,1,-1) ; //z = 1

    vpTRACE("define the task") ;
    vpTRACE("\t we want an eye-in-hand control law") ;
    vpTRACE("\t robot is controlled in the camera frame") ;
    task.setServo(vpServo::EYEINHAND_CAMERA) ;

    vpTRACE("\t we want to see a point on a point..") ;
    cout << endl ;
    for (i=0 ; i < nbline ; i++)
      task.addFeature(p[i],pd[i]) ;

    vpTRACE("\t set the gain") ;
   task.setLambda(0.032) ;


    vpTRACE("Display task information " ) ;
    task.print() ;


    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

    int iter=0 ;
    vpTRACE("\t loop") ;
    vpColVector v ;
    //  char s[FILENAME_MAX] ;
    vpImage<vpRGBa> Ic ;
    double lambda_av =0.05;
    double alpha = 0.2 ; //1 ;
    double beta =3 ; //3 ;

    while(1)
    {
      cout << "---------------------------------------------" << iter <<endl ;

      try {
        g.acquire(I) ;
        vpDisplay::display(I) ;

        for (i=0 ; i < nbline ; i++)
        {
          line[i].track(I) ;
          line[i].display(I, vpColor::red) ;

          //    vpDisplay::displayCross(I,(int)line.I(), (int)line.J(),
          //			   10,vpColor::green) ;

          vpFeatureBuilder::create(p[i],cam,line[i]);
          vpTRACE("%f %f ",line[i].getRho(), line[i].getTheta()) ;

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

        vpServoDisplay::display(task,cam,I) ;
        //  cout << v.t() ;
        if (iter==0)  vpDisplay::getClick(I) ;
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
      // sprintf(s,"/tmp/image.%04d.ppm",iter) ;
      //   vpImageIo::writePPM(Ic,s) ;

      vpTRACE("\t\t || s - s* || = %f ", task.error.sumSquare()) ;
      iter++;
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
