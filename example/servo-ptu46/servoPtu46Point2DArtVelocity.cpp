/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in articular
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file servoPtu46Point2DArtVelocity.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the ptu-46
  robot (pan-tilt head provided by Directed Perception). The velocity is
  computed in articular. The visual feature is the center of gravity of a
  point.

*/


/*!
  \example servoPtu46Point2DArtVelocity.cpp

  Example of eye-in-hand control law. We control here a real robot, the ptu-46
  robot (pan-tilt head provided by Directed Perception). The velocity is
  computed in articular. The visual feature is the center of gravity of a
  point.

*/
#include <visp/vpConfig.h>
#include <visp/vpDebug.h> // Debug trace
#ifdef UNIX
#  include <unistd.h>
#endif
#include <signal.h>



#if (defined(VISP_HAVE_PTU46) & defined (VISP_HAVE_DC1394) )

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

#include <visp/vpRobotPtu46.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpServoDisplay.h>

#include <visp/vpDot2.h>


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
  std::cout << std::endl ;
  std::cout << "-------------------------------------------------------" << std::endl ;
  std::cout << " Test program for vpServo "  <<std::endl ;
  std::cout << " Eye-in-hand task control, velocity computed in the camera frame" << std::endl ;
  std::cout << " Simulation " << std::endl ;
  std::cout << " task : servo a point " << std::endl ;
  std::cout << "-------------------------------------------------------" << std::endl ;
  std::cout << std::endl ;

  try{

#ifdef VISP_HAVE_PTHREAD
    pthread_mutex_lock( &mutexEndLoop );
#endif
    signal( SIGINT,&signalCtrC );

    vpRobotPtu46 robot ;
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
      vpERROR_TRACE(" Error caught") ;
      return(-1) ;
    }


    vpDisplayX display(I,100,100,"testDisplayX.cpp ") ;
    vpTRACE(" ") ;

    try{
      vpDisplay::display(I) ;
      vpDisplay::flush(I) ;
    }
    catch(...)
    {
      vpERROR_TRACE(" Error caught") ;
      return(-1) ;
    }


    vpServo task ;

    vpDot2 dot ;

    try{
      vpERROR_TRACE("start dot.initTracking(I) ") ;
      int x,y;
      vpDisplay::getClick( I,y,x );
      dot.set_u( x ) ;
      dot.set_v( y ) ;
      vpDEBUG_TRACE(25,"Click!");
      //dot.initTracking(I) ;
      dot.track(I);
      vpERROR_TRACE("after dot.initTracking(I) ") ;
    }
    catch(...)
    {
      vpERROR_TRACE(" Error caught ") ;
      return(-1) ;
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

    vpVelocityTwistMatrix cVe ;
    robot.get_cVe(cVe) ;
    std::cout << cVe <<std::endl ;
    task.set_cVe(cVe) ;

    vpDisplay::getClick(I) ;
    vpTRACE("Set the Jacobian (expressed in the end-effector frame)") ;
    vpMatrix eJe ;
    robot.get_eJe(eJe) ;
    task.set_eJe(eJe) ;


    vpTRACE("\t we want to see a point on a point..") ;
    std::cout << std::endl ;
    task.addFeature(p,pd) ;

    vpTRACE("\t set the gain") ;
    task.setLambda(0.1) ;


    vpTRACE("Display task information " ) ;
    task.print() ;


    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

    unsigned int iter=0 ;
    vpTRACE("\t loop") ;
#ifdef VISP_HAVE_PTHREAD
    while( 0 != pthread_mutex_trylock( &mutexEndLoop ) )
#else
    for ( ; ; )
#endif
    {
      std::cout << "---------------------------------------------" << iter <<std::endl ;

      g.acquire(I) ;
      vpDisplay::display(I) ;

      dot.track(I) ;

      //    vpDisplay::displayCross(I,(int)dot.I(), (int)dot.J(),
      //			   10,vpColor::green) ;


      vpFeatureBuilder::create(p,cam, dot);

      // get the jacobian
      robot.get_eJe(eJe) ;
      task.set_eJe(eJe) ;

      //  std::cout << (vpMatrix)cVe*eJe << std::endl ;

      vpColVector v ;
      v = task.computeControlLaw() ;

      vpServoDisplay::display(task,cam,I) ;
      std::cout << v.t() ;
      robot.setVelocity(vpRobot::ARTICULAR_FRAME, v) ;
      vpDisplay::flush(I) ;

      vpTRACE("\t\t || s - s* || = %f ", ( task.getError() ).sumSquare()) ;
    }

    vpTRACE("Display task information " ) ;
    task.print() ;
    task.kill();

  } catch (...) { vpERROR_TRACE("Trow uncatched..."); }
}


#else
int
main()
{
  vpERROR_TRACE("You don't have a ptu-46 head connected to your computer ",
                "or 1394 framegrabbing capabilities...");
}
#endif
