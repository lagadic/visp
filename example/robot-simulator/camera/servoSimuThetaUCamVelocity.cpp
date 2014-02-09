/****************************************************************************
 *
 * $Id: servoSimuThetaUCamVelocity.cpp 2457 2010-01-07 10:41:18Z nmelchio $
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
 * Simulation of a visual servoing using theta U visual features.
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in the camera frame
 *   using theta U visual feature
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/


/*!
  \example servoSimuThetaUCamVelocity.cpp
  Simulation of a visual servoing using theta U visual features:
  - eye-in-hand control law,
  - velocity computed in the camera frame,
  - no display.
*/

#include <stdlib.h>
#include <stdio.h>

#include <visp/vpFeatureThetaU.h>
#include <visp/vpFeatureTranslation.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpMath.h>
#include <visp/vpParseArgv.h>
#include <visp/vpServo.h>
#include <visp/vpSimulatorCamera.h>

// List of allowed command line options
#define GETOPTARGS	"h"
void usage(const char *name, const char *badparam);
bool getOptions(int argc, const char **argv);
/*!

Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Simulation of avisual servoing using theta U visual feature:\n\
- eye-in-hand control law,\n\
- velocity computed in the camera frame,\n\
- without display.\n\
          \n\
SYNOPSIS\n\
  %s [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
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

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv)
{
  const char *optarg_;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
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
    // Read the command line options
    if (getOptions(argc, argv) == false) {
      exit (-1);
    }

    vpServo task ;
    vpSimulatorCamera robot ;

    std::cout << std::endl ;
    std::cout << "-------------------------------------------------------" << std::endl ;
    std::cout << " Test program for vpServo "  <<std::endl ;
    std::cout << " Eye-in-hand task control, velocity computed in the camera frame" << std::endl ;
    std::cout << " Simulation " << std::endl ;
    std::cout << " task :  servo using theta U visual feature " << std::endl ;
    std::cout << "-------------------------------------------------------" << std::endl ;
    std::cout << std::endl ;


    // sets the initial camera location
    vpPoseVector c_r_o(0.1,0.2,2,
                       vpMath::rad(20), vpMath::rad(10),  vpMath::rad(50)
                       ) ;

    vpHomogeneousMatrix cMo(c_r_o) ;
    // Compute the position of the object in the world frame
    vpHomogeneousMatrix wMc, wMo;
    robot.getPosition(wMc) ;
    wMo = wMc * cMo;

    // sets the desired camera location
    vpPoseVector cd_r_o(0,0,1,
                        vpMath::rad(0),vpMath::rad(0),vpMath::rad(0)) ;
    vpHomogeneousMatrix cdMo(cd_r_o) ;


    // compute the rotation that the camera has to realize
    vpHomogeneousMatrix cdMc ;
    cdMc = cdMo*cMo.inverse() ;
    vpFeatureThetaU tu(vpFeatureThetaU::cdRc) ;
    tu.buildFrom(cdMc) ;

    // define the task
    // - we want an eye-in-hand control law
    // - robot is controlled in the camera frame
    task.setServo(vpServo::EYEINHAND_CAMERA) ;
    task.setInteractionMatrixType(vpServo::DESIRED) ;

    task.addFeature(tu) ;

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

      // new rotation to achieve
      cdMc = cdMo*cMo.inverse() ;
      tu.buildFrom(cdMc) ;

      // compute the control law
      v = task.computeControlLaw() ;

      // send the camera velocity to the controller
      robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

      std::cout << "|| s - s* || = " << ( task.getError() ).sumSquare() <<std::endl ; ;
    }

    // Display task information
    task.print() ;
    task.kill();
    return 0;
  }
  catch(vpException e) {
    std::cout << "Catch a ViSP exception: " << e << std::endl;
    return 1;
  }
}

