/****************************************************************************
 *
 * $Id: servoAfma6Ellipse2DCamVelocity.cpp,v 1.7 2007-06-06 08:25:56 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in the camera frame
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \file servoAfma6Ellipse2DCamVelocity.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the Afma6
  robot (cartesian robot, with 6 degrees of freedom). The velocity is computed
  in the camera frame. The used visual feature is a circle.

*/


/*!
  \example servoAfma6Ellipse2DCamVelocity.cpp

  Example of eye-in-hand control law. We control here a real robot, the Afma6
  robot (cartesian robot, with 6 degrees of freedom). The velocity is computed
  in the camera frame. The used visual feature is a circle.

*/

#include <visp/vpConfig.h>
#include <visp/vpDebug.h> // Debug trace

#if (defined (VISP_HAVE_AFMA6) && defined (VISP_HAVE_ITIFG8))

#include <visp/vpItifg8Grabber.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeatureEllipse.h>
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

    vpServo task ;

    vpImage<unsigned char> I ;
    vpItifg8Grabber g(2) ;
    g.open(I) ;
    g.acquire(I) ;

    vpDisplayX display(I,0,0,"testServoEllipse.cpp ") ;
    vpTRACE(" ") ;
    vpDisplay::display(I) ;

    std::cout << std::endl ;
    std::cout << "-------------------------------------------------------" << std::endl ;
    std::cout << " Test program for vpServo "  <<std::endl ;
    std::cout << " Eye-in-hand task control, velocity computed in the camera frame" << std::endl ;
    std::cout << " Simulation " << std::endl ;
    std::cout << " task : servo a point " << std::endl ;
    std::cout << "-------------------------------------------------------" << std::endl ;
    std::cout << std::endl ;


    vpDot dot ;

    dot.setMaxDotSize(0.30) ; // Max dot size is 30 % of the image size
    // dot.setGraphics(true) ;
    dot.setComputeMoments(true) ;
    std::cout << "Click on an ellipse..." << std::endl;
    dot.initTracking(I) ;
    vpDisplay::displayCross(I,
			    (unsigned int)dot.get_v(),
			    (unsigned int)dot.get_u(),
			    10, vpColor::blue) ;
    vpDisplay::flush(I);

    dot.track(I) ;

    vpCameraParameters cam ;

    vpTRACE("sets the current position of the visual feature ") ;
    vpFeatureEllipse c ;
    vpFeatureBuilder::create(c,cam, dot)  ;


    std::cout << " Learning 0/1 " <<std::endl ;
    int learning ;
    std::cin >> learning ;
    char name[FILENAME_MAX] ;
    sprintf(name,"dat/ellipse.dat") ;
    if (learning ==1)
    {
      // save the object position
      vpTRACE("Save the location of the object in a file dat/ellipse.dat") ;
      std::ofstream f(name) ;
      f << c.get_s().t() ;
      f.close() ;
      exit(1) ;
    }


    vpTRACE("sets the desired position of the visual feature ") ;
    vpFeatureEllipse cd ;
    std::ifstream f("dat/ellipse.dat") ;
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

    std::cout << "alpha 0.7" << std::endl;
    std::cin >> alpha ;
    std::cout << "beta 5" << std::endl;
    std::cin >> beta ;
    while(1)
    {
      std::cout << "---------------------------------------------" << iter++ <<std::endl ;

      g.acquire(I) ;
      vpDisplay::display(I) ;

      dot.track(I) ;
      vpDisplay::displayCross(I,
			      (unsigned int)dot.get_v(),
			      (unsigned int)dot.get_u(),
			      10, vpColor::green) ;

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


      vpTRACE("%f %f", task.error.sumSquare(),  gain) ;
      task.setLambda(gain) ;
      vpColVector v ;
      v = task.computeControlLaw() ;
      std::cout <<"rank " << task.rankJ1 << std::endl ;
      vpServoDisplay::display(task,cam,I) ;
      std::cout << v.t() ;
      robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

      vpDisplay::flush(I) ;
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
  vpERROR_TRACE("You do not have an afma6 robot or an Itifg8 framegrabber connected to your computer...");
}
#endif
