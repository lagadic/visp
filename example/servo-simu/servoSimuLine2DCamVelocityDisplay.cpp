
/****************************************************************************
 *
 * $Id: servoSimuLine2DCamVelocityDisplay.cpp 2457 2010-01-07 10:41:18Z nmelchio $
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
 * Simulation of a 2D visual servoing on a line.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file servoSimuLine2DCamVelocityDisplay.cpp
  \brief Servo a line:
  - eye-in-hand control law,
  - velocity computed in the camera frame,
  - display the camera view.
*/


/*!
  \example servoSimuLine2DCamVelocityDisplay.cpp
  Servo a line:
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
#include <visp/vpLine.h>
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
Simulation of 2D a visual servoing on a line:\n\
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
     Disable the mouse click. Usefull to automaze the \n\
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
  vpRobotCamera robot ;

  vpTRACE("sets the initial camera location " ) ;
  vpHomogeneousMatrix cMo(-0.2,0.1,1,
			  vpMath::rad(5),  vpMath::rad(5),  vpMath::rad(90));

  robot.setPosition(cMo) ;

  vpTRACE("sets the final camera location (for simulation purpose)" ) ;
  vpHomogeneousMatrix cMod(0,0,1,
			   vpMath::rad(0),  vpMath::rad(0),  vpMath::rad(0));



  vpTRACE("sets the line coordinates (2 planes) in the world frame "  ) ;

  vpColVector plane1(4) ;
  vpColVector plane2(4) ;
  plane1[0] = 0;  // z = 0
  plane1[1] = 0;
  plane1[2] = 1;
  plane1[3] = 0;
  plane2[0] = 0; // y  =0
  plane2[1] = 1;
  plane2[2] = 0;
  plane2[3] = 0;


  vpLine line ;
  line.setWorldCoordinates(plane1, plane2) ;

  vpTRACE("sets the desired position of the visual feature ") ;
  line.track(cMod) ;
  line.print() ;

  vpFeatureLine ld ;
  vpFeatureBuilder::create(ld,line)  ;


  vpTRACE("project : computes  the line coordinates in the camera frame and its 2D coordinates"  ) ;
  vpTRACE("sets the current position of the visual feature ") ;
  line.track(cMo) ;
  line.print() ;

  vpFeatureLine l ;
  vpFeatureBuilder::create(l,line)  ;
  l.print() ;

  vpTRACE("define the task") ;
  vpTRACE("\t we want an eye-in-hand control law") ;
  vpTRACE("\t robot is controlled in the camera frame") ;
  task.setServo(vpServo::EYEINHAND_CAMERA) ;

  vpTRACE("\t we want to see a line on a line..\n") ;

  task.addFeature(l,ld) ;
  vpDisplay::display(I) ;
  vpServoDisplay::display(task,cam,I) ;
  vpDisplay::flush(I) ; 

  vpTRACE("\t set the gain") ;
  task.setLambda(1) ;


  vpTRACE("Display task information " ) ;
  task.print() ;

  if (opt_display && opt_click_allowed) {
    std::cout << "\n\nClick in the camera view window to start..." << std::endl;
    vpDisplay::getClick(I) ;
  }

  int iter=0 ;
  vpTRACE("\t loop") ;
  while(iter++<200)
    {
      std::cout << "---------------------------------------------" << iter <<std::endl ;
      vpColVector v ;

      if (iter==1) vpTRACE("\t\t get the robot position ") ;
      robot.getPosition(cMo) ;
      if (iter==1) vpTRACE("\t\t new line position ") ;
      //retrieve x,y and Z of the vpLine structure

      line.track(cMo) ;
      vpFeatureBuilder::create(l,line);

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

    }

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

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
