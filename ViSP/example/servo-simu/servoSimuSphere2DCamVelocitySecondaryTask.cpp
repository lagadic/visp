
/****************************************************************************
 *
 * $Id: servoSimuSphere2DCamVelocitySecondaryTask.cpp,v 1.3 2007-04-20 14:22:15 asaunier Exp $
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
 * Simulation of a 2D visual servoing on a sphere.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file servoSimuSphere2DCamVelocitySecondaryTask.cpp
  \brief Servo a sphere:
  - eye-in-hand control law,
  - velocity computed in the camera frame,
  - without display,
  - a secondary task is the added.

*/

/*!
  \example servoSimuSphere2DCamVelocitySecondaryTask.cpp
  Servo a sphere:
  - eye-in-hand control law,
  - velocity computed in the camera frame,
  - without display,
  - a secondary task is the added.

*/

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeatureEllipse.h>
#include <visp/vpSphere.h>
#include <visp/vpServo.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpFeatureBuilder.h>


// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>

// Debug trace
#include <visp/vpDebug.h>
#include <visp/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS	"h"

/*!

Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

*/
void usage(char *name, char *badparam)
{
  fprintf(stdout, "\n\
Simulation of a 2D visual servoing on a sphere:\n\
- eye-in-hand control law,\n\
- velocity computed in the camera frame,\n\
- without display,\n\
- a secondary task is the added.\n\
\n\
SYNOPSIS\n\
  %s [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
\n\
  -h\n\
     Print the help.\n");

}

/*!

Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, char **argv)
{
  char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
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
main(int argc, char ** argv)
{
  // Read the command line options
  if (getOptions(argc, argv) == false) {
    exit (-1);
  }

  vpServo task ;
  vpRobotCamera robot ;

  std::cout << std::endl ;
  std::cout << "-------------------------------------------------------" << std::endl ;
  std::cout << " Test program for vpServo "  <<std::endl ;
  std::cout << " Simulation " << std::endl ;
  std::cout << " task : servo a sphere with a secondary task" << std::endl ;
  std::cout << "-------------------------------------------------------" << std::endl ;
  std::cout << std::endl ;


  vpTRACE("sets the initial camera location " ) ;
  vpHomogeneousMatrix cMo ;
  cMo[0][3] = 0.1 ;
  cMo[1][3] = 0.2 ;
  cMo[2][3] = 2 ;
  robot.setPosition(cMo) ;

  vpHomogeneousMatrix cMod ;
  cMod[0][3] = 0 ;
  cMod[1][3] = 0 ;
  cMod[2][3] = 1 ;



  vpTRACE("sets the sphere coordinates in the world frame "  ) ;
  vpSphere sphere ;
  sphere.setWorldCoordinates(0,0,0,0.1) ;

  vpTRACE("sets the desired position of the visual feature ") ;
  vpFeatureEllipse pd ;
  sphere.track(cMod) ;
  vpFeatureBuilder::create(pd,sphere)  ;

  vpTRACE("project : computes  the sphere coordinates in the camera frame and its 2D coordinates"  ) ;

  vpTRACE("sets the current position of the visual feature ") ;
  vpFeatureEllipse p ;
  sphere.track(cMo) ;
  vpFeatureBuilder::create(p,sphere)  ;

  vpTRACE("define the task") ;
  vpTRACE("\t we want an eye-in-hand control law") ;
  vpTRACE("\t robot is controlled in the camera frame") ;
  task.setServo(vpServo::EYEINHAND_CAMERA) ;

  vpTRACE("\t we want to see a sphere on a sphere..") ;
  std::cout << std::endl ;
  task.addFeature(p,pd) ;

  vpTRACE("\t set the gain") ;
  task.setLambda(0.1) ;


  vpTRACE("Display task information " ) ;
  task.print() ;
  // exit(1) ;
  int iter=0 ;
  vpTRACE("\t loop") ;
  while(iter++<500)
    {
      std::cout << "---------------------------------------------" << iter <<std::endl ;
      vpColVector v ;

      if (iter==1) vpTRACE("\t\t get the robot position ") ;
      robot.getPosition(cMo) ;
      if (iter==1) vpTRACE("\t\t new sphere position ") ;
      //retrieve x,y and Z of the vpSphere structure

      sphere.track(cMo) ;
      vpFeatureBuilder::create(p,sphere);

      vpColVector de2dt(6) ;
      de2dt[2] = 1 ;    // should be zero in (I-WpW)de2dt
      de2dt[5] = 0.01 ; // should be ok
      de2dt[0] = 0.01 ;  // should generate a motion on (I-WpW)de2dt[4]

      if (iter==1) vpTRACE("\t\t compute the control law ") ;

      v = task.computeControlLaw() ;

      std::cout << "de2dt :"<< de2dt.t() ;
      vpColVector sec ;
      sec = task.secondaryTask(de2dt) ;
      std::cout << " (I-WpW)de2dt :"<< sec.t() ;

      if (iter>20)  v += sec ;

      if (iter==1) vpTRACE("\t\t send the camera velocity to the controller ") ;
      robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

      std::cout << "\t || s - s* || " ;
      std::cout << task.error.sumSquare() <<std::endl ;


    }

  vpTRACE("Display task information " ) ;
  task.print() ;
}

