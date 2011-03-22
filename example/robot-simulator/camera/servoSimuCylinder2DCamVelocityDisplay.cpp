/****************************************************************************
 *
 * $Id: servoSimuCylinder2DCamVelocityDisplay.cpp 2457 2010-01-07 10:41:18Z nmelchio $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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
 * Simulation of a 2D visual servoing on a cylinder.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \file servoSimuCylinder2DCamVelocityDisplay.cpp
  \brief Servo a cylinder:
  - eye-in-hand control law,
  - velocity computed in the camera frame,
  - display the camera view.

*/

/*!
  \example servoSimuCylinder2DCamVelocityDisplay.cpp
  Servo a cylinder:
  - eye-in-hand control law,
  - velocity computed in the camera frame,
  - display the camera view.

*/

#include <visp/vpDebug.h>
#include <visp/vpConfig.h>

#if (defined (VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI))

#include <stdlib.h>
#include <stdio.h>

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeatureLine.h>
#include <visp/vpCylinder.h>
#include <visp/vpServo.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpFeatureBuilder.h>


// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>

// Debug trace
#include <visp/vpDebug.h>


#include <visp/vpServoDisplay.h>

#include <visp/vpImage.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS	"cdh"

/*!

Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Simulation of a 2D visual servoing on a cylinder:\n\
- eye-in-hand control law,\n\
- velocity computed in the camera frame,\n\
- display the camera view.\n\
\n\
SYNOPSIS\n\
  %s [-c] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
\n\
  -c\n\
     Disable the mouse click. Useful to automaze the \n\
     execution of this program without humain intervention.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -h\n\
     Print the help.\n");

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!

Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param click_allowed : false if mouse click is not allowed.
  \param display : false if the display is to turn off.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, bool &click_allowed, bool &display)
{
  const char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'c': click_allowed = false; break;
    case 'd': display = false; break;
    case 'h': usage(argv[0], NULL); return false; break;

    default:
      usage(argv[0], optarg);
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}


int
main(int argc, const char ** argv)
{
  bool opt_display = true;
  bool opt_click_allowed = true;

  // Read the command line options
  if (getOptions(argc, argv, opt_click_allowed, opt_display) == false) {
    exit (-1);
  }

  vpImage<unsigned char> I(512,512,255) ;

  // We open a window using either X11, GTK or GDI.
#if defined VISP_HAVE_X11
  vpDisplayX display;
#elif defined VISP_HAVE_GTK
  vpDisplayGTK display;
#elif defined VISP_HAVE_GDI
  vpDisplayGDI display;
#endif

  if (opt_display) {
    try{
      // Display size is automatically defined by the image (I) size
      display.init(I, 100, 100,"Camera view...") ;
      // Display the image
      // The image class has a member that specify a pointer toward
      // the display that has been initialized in the display declaration
      // therefore is is no longuer necessary to make a reference to the
      // display variable.
      vpDisplay::display(I) ;
      vpDisplay::flush(I) ;
    }
    catch(...)
      {
	vpERROR_TRACE("Error while displaying the image") ;
	exit(-1);
      }
  }

  double px, py ; px = py = 600 ;
  double u0, v0 ; u0 = v0 = 256 ;

  vpCameraParameters cam(px,py,u0,v0);

  vpServo task ;
  vpRobotCamera robot ;

  vpTRACE("sets the initial camera location " ) ;
  vpHomogeneousMatrix cMo(-0.2,0.1,2,
			  vpMath::rad(5),  vpMath::rad(5),  vpMath::rad(20));

  robot.setPosition(cMo) ;

  vpTRACE("sets the final camera location (for simulation purpose)" ) ;
  vpHomogeneousMatrix cMod(0,0,1,
			   vpMath::rad(-60),  vpMath::rad(0),  vpMath::rad(0));



  vpTRACE("sets the cylinder coordinates in the world frame "  ) ;
  vpCylinder cylinder(0,1,0,  // direction
		      0,0,0,  // point of the axis
		      0.1) ;  // radius

  vpTRACE("sets the desired position of the visual feature ") ;
  cylinder.track(cMod) ;
  cylinder.print() ;

  vpFeatureLine ld[2] ;
  int i ;
  for(i=0 ; i < 2 ; i++)
    vpFeatureBuilder::create(ld[i],cylinder,i)  ;


  vpTRACE("project : computes  the cylinder coordinates in the camera frame and its 2D coordinates"  ) ;
  vpTRACE("sets the current position of the visual feature ") ;
  cylinder.track(cMo) ;
  cylinder.print() ;

  vpFeatureLine l[2] ;
  for(i=0 ; i < 2 ; i++)
    {
      vpFeatureBuilder::create(l[i],cylinder,i)  ;
      l[i].print() ;
    }

  vpTRACE("define the task") ;
  vpTRACE("\t we want an eye-in-hand control law") ;
  vpTRACE("\t robot is controlled in the camera frame") ;
  task.setServo(vpServo::EYEINHAND_CAMERA) ;
  // task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE) ;
  //  it can also be interesting to test these possibilities
  //    task.setInteractionMatrixType(vpServo::MEAN, vpServo::PSEUDO_INVERSE) ;
  task.setInteractionMatrixType(vpServo::DESIRED,vpServo::PSEUDO_INVERSE) ;
  //task.setInteractionMatrixType(vpServo::DESIRED,  vpServo::TRANSPOSE) ;
  // task.setInteractionMatrixType(vpServo::CURRENT,  vpServo::TRANSPOSE) ;

  vpTRACE("\t we want to see  2 lines on 2 lines.") ;

  task.addFeature(l[0],ld[0]) ;
  task.addFeature(l[1],ld[1]) ;

  vpServoDisplay::display(task,cam,I) ;
  vpDisplay::flush(I) ;

  vpTRACE("Display task information " ) ;
  task.print() ;

  if (opt_display && opt_click_allowed) {
    std::cout << "\n\nClick in the camera view window to start..." << std::endl;
    vpDisplay::getClick(I) ;
  }

  vpTRACE("\t set the gain") ;
  task.setLambda(1) ;


  vpTRACE("Display task information " ) ;
  task.print() ;

  int iter=0 ;
  vpTRACE("\t loop") ;
  do
    {
      std::cout << "---------------------------------------------" << iter++ <<std::endl ;
      vpColVector v ;

      if (iter==1) vpTRACE("\t\t get the robot position ") ;
      robot.getPosition(cMo) ;
      if (iter==1) vpTRACE("\t\t new line position ") ;
      //retrieve x,y and Z of the vpLine structure

      cylinder.track(cMo) ;
      //    cylinder.print() ;
      for(i=0 ; i < 2 ; i++)
	{
	  vpFeatureBuilder::create(l[i],cylinder,i)  ;
	  //   l[i].print() ;
	}

      if (opt_display) {
	      vpDisplay::display(I) ;
	      vpServoDisplay::display(task,cam,I) ;
        vpDisplay::flush(I) ; 
      }

      if (iter==1) vpTRACE("\t\t compute the control law ") ;
      v = task.computeControlLaw() ;

      if (iter==1) vpTRACE("\t\t send the camera velocity to the controller ") ;
      robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

      vpTRACE("\t\t || s - s* || ") ;
      std::cout << task.error.sumSquare() <<std::endl ; ;

      //   vpDisplay::getClick(I) ;
    }
  while(task.error.sumSquare() >  1e-9) ;

  if (opt_display && opt_click_allowed) {
    std::cout << "\nClick in the camera view window to end..." << std::endl;
    vpDisplay::getClick(I) ;
  }

  vpTRACE("Display task information " ) ;
  task.print() ;
  task.kill();
}

#else
int
main()
{
  vpERROR_TRACE("You do not have X11, GTK or GDI display functionalities...");
}

#endif
