
/****************************************************************************
 *
 * $Id: servoSimuCircle2DCamVelocityDisplay.cpp 2457 2010-01-07 10:41:18Z nmelchio $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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
 * Simulation of a 2D visual servoing on a circle.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example servoSimuCircle2DCamVelocityDisplay.cpp
  \brief Servo a circle:

  Servo a circle:
  - eye-in-hand control law,
  - velocity computed in the camera frame,
  - display the camera view.
*/

#include <visp/vpDebug.h>
#include <visp/vpConfig.h>

#if (defined (VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI))

#include <stdlib.h>
#include <stdio.h>

#include <visp/vpCameraParameters.h>
#include <visp/vpCircle.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureLine.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpMath.h>
#include <visp/vpParseArgv.h>
#include <visp/vpProjectionDisplay.h>
#include <visp/vpServo.h>
#include <visp/vpSimulatorCamera.h>
#include <visp/vpServoDisplay.h>

// List of allowed command line options
#define GETOPTARGS	"cdh"

void usage(const char *name, const char *badparam);
bool getOptions(int argc, const char **argv, bool &click_allowed, bool &display);

/*!

Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Simulation of a 2D visual servoing on a circle:\n\
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

  if (badparam) {
    fprintf(stderr, "ERROR: \n" );
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);
  }
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
  const char *optarg_;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'c': click_allowed = false; break;
    case 'd': display = false; break;
    case 'h': usage(argv[0], NULL); return false; break;

    default:
      usage(argv[0], optarg_);
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}


int
main(int argc, const char ** argv)
{
  try {
    bool opt_display = true;
    bool opt_click_allowed = true;

    // Read the command line options
    if (getOptions(argc, argv, opt_click_allowed, opt_display) == false) {
      exit (-1);
    }

    vpImage<unsigned char> I(512,512,0) ;

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
    vpSimulatorCamera robot ;

    // sets the initial camera location
    vpHomogeneousMatrix cMo(0,0,1,
                            vpMath::rad(0),  vpMath::rad(80),  vpMath::rad(30))   ;
    vpHomogeneousMatrix wMc, wMo;
    robot.getPosition(wMc) ;
    wMo = wMc * cMo; // Compute the position of the object in the world frame

    vpHomogeneousMatrix cMod(-0.1,-0.1,0.7,
                             vpMath::rad(40),  vpMath::rad(10),  vpMath::rad(30))   ;

    // sets the circle coordinates in the world frame
    vpCircle circle ;
    circle.setWorldCoordinates(0,0,1,
                               0,0,0,
                               0.1) ;

    // sets the desired position of the visual feature
    vpFeatureEllipse pd ;
    circle.track(cMod) ;
    vpFeatureBuilder::create(pd,circle)  ;

    // project : computes the circle coordinates in the camera frame and its 2D coordinates
    // sets the current position of the visual feature
    vpFeatureEllipse p ;
    circle.track(cMo) ;
    vpFeatureBuilder::create(p,circle)  ;

    // define the task
    // - we want an eye-in-hand control law
    // - robot is controlled in the camera frame
    task.setServo(vpServo::EYEINHAND_CAMERA) ;
    task.setInteractionMatrixType(vpServo::DESIRED) ;
    // - we want to see a circle on a circle
    task.addFeature(p,pd) ;
    // - set the gain
    task.setLambda(1) ;

    // Display task information
    task.print() ;

    unsigned int iter=0 ;
    // loop
    while(iter++ < 200)
    {
      std::cout << "---------------------------------------------" << iter <<std::endl ;
      vpColVector v ;

      // get the robot position
      robot.getPosition(wMc) ;
      // Compute the position of the camera wrt the object frame
      cMo = wMc.inverse() * wMo;

      // new circle position
      // retrieve x,y and Z of the vpCircle structure
      circle.track(cMo) ;
      vpFeatureBuilder::create(p,circle);
      circle.print() ;
      p.print() ;

      if (opt_display) {
        vpDisplay::display(I) ;
        vpServoDisplay::display(task,cam,I) ;
        vpDisplay::flush(I) ;
      }

      // compute the control law
      v = task.computeControlLaw() ;
      std::cout << "task rank: " << task.getTaskRank() <<std::endl ;
      // send the camera velocity to the controller
      robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

      std::cout << "|| s - s* || = " << ( task.getError() ).sumSquare() <<std::endl ;
    }

    // Display task information
    task.print() ;
    task.kill();

    if (opt_display && opt_click_allowed) {
      std::cout << "Click in the camera view window to end..." << std::endl;
      vpDisplay::getClick(I) ;
    }
    return 0;
  }
  catch(vpException e) {
    std::cout << "Catch a ViSP exception: " << e << std::endl;
    return 1;
  }
}
#else
int
main()
{
  vpERROR_TRACE("You do not have X11, GTK or GDI display functionalities...");
}

#endif
