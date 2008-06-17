/****************************************************************************
 *
 * $Id: servoSimuPoint2DhalfCamVelocity1.cpp,v 1.9 2008-06-17 08:08:26 asaunier Exp $
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
 * Simulation of a 2 1/2 D visual servoing.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file servoSimuPoint2DhalfCamVelocity1.cpp
  \brief Simulation of a 2 1/2 D visual servoing (x,y,Z,theta U):
  - (x,y,Z,theta U) features,
  - eye-in-hand control law,
  - velocity computed in the camera frame,
  - no display.

*/

/*!
  \example servoSimuPoint2DhalfCamVelocity1.cpp
  Simulation of a 2 1/2 D visual servoing (theta U):
  - (x,y,Z,theta U) features,
  - eye-in-hand control law,
  - velocity computed in the camera frame,
  - no display.

*/


#include <stdlib.h>
#include <stdio.h>

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpPoint.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpFeaturePoint3D.h>
#include <visp/vpServo.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpDebug.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS	"h"

/*!

Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Simulation of a 2 1/2 D visual servoing (x,y,Z,theta U):\n\
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

  if (badparam) {
    fprintf(stderr, "ERROR: \n" );
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);
  }
}

/*!

Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv)
{
  const char *optarg;
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
main(int argc, const char ** argv)
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
  std::cout << " task :  2 1/2 D visual servoing " << std::endl ;
  std::cout << "-------------------------------------------------------" << std::endl ;
  std::cout << std::endl ;


  vpTRACE("sets the initial camera location " ) ;
  vpPoseVector c_r_o(0.1,0.2,2,
		     vpMath::rad(20), vpMath::rad(10),  vpMath::rad(50)
		     ) ;

  vpCTRACE ; std::cout << std::endl ;
  vpHomogeneousMatrix cMo(c_r_o) ;
  vpCTRACE ; std::cout << std::endl ;
  robot.setPosition(cMo) ;
  vpCTRACE ; std::cout << std::endl ;

  vpTRACE("sets the desired camera location " ) ;
  vpPoseVector cd_r_o(0,0,1,
		      vpMath::rad(0),vpMath::rad(0),vpMath::rad(0)) ;
  vpHomogeneousMatrix cdMo(cd_r_o) ;


  vpTRACE("\tsets the point coordinates in the world frame "  ) ;
  vpPoint point ;
  point.setWorldCoordinates(0,0,0) ;
  vpTRACE("\tproject : computes  the point coordinates in the camera frame and its 2D coordinates"  ) ;
  point.track(cMo) ;

  vpPoint pointd ;
  pointd.setWorldCoordinates(0,0,0) ;
  pointd.track(cdMo) ;
  //------------------------------------------------------------------
  vpTRACE("1st feature (x,y)");


  vpTRACE("\t want to it at (0,0)") ;
  vpFeaturePoint p ;
  vpFeatureBuilder::create(p,point)  ;


  vpFeaturePoint pd ;
  vpFeatureBuilder::create(pd,pointd)  ;


  //------------------------------------------------------------------
  vpTRACE("2nd feature (Z)") ;
  vpTRACE("\tnot necessary to project twice (reuse p)") ;
  vpFeaturePoint3D Z ;
  vpFeatureBuilder::create(Z,point)  ;  //retrieve x,y and Z of the vpPoint structure

  vpTRACE("\twant to see it one meter away (here again use pd)") ;
  vpFeaturePoint3D Zd ;
  vpFeatureBuilder::create(Zd,pointd)  ;  //retrieve x,y and Z of the vpPoint structure


  //------------------------------------------------------------------
  vpTRACE("3rd feature ThetaU") ;
  vpTRACE("\tcompute the rotation that the camera has to realize "  ) ;
  vpHomogeneousMatrix cdMc ;
  cdMc = cdMo*cMo.inverse() ;

  vpFeatureThetaU tu ;
  tu.buildFrom(cdMc) ;

  vpTRACE("\tsets the desired rotation (always zero !) ") ;
  vpTRACE("\tsince s is the rotation that the camera has to realize ") ;


  //------------------------------------------------------------------

  vpTRACE("define the task") ;
  vpTRACE("\t we want an eye-in-hand control law") ;
  vpTRACE("\t robot is controlled in the camera frame") ;
  task.setServo(vpServo::EYEINHAND_CAMERA) ;

  task.addFeature(p,pd) ;
  task.addFeature(Z,Zd,vpFeaturePoint3D::selectZ()) ;
  task.addFeature(tu) ;

  vpTRACE("\t set the gain") ;
  task.setLambda(1) ;


  vpTRACE("Display task information " ) ;
  task.print() ;

  int iter=0 ;
  vpTRACE("\t loop") ;
  while(iter++<200)
    {
      std::cout << "---------------------------------------------" << iter <<std::endl ;
      vpColVector v ;

      if (iter==1) vpTRACE("\t\t get the robot position ") ;
      robot.getPosition(cMo) ;

      if (iter==1) vpTRACE("\t\t update the feature ") ;
      point.track(cMo) ;
      vpFeatureBuilder::create(p,point)  ;
      vpFeatureBuilder::create(Z,point)  ;

      cdMc = cdMo*cMo.inverse() ;
      tu.buildFrom(cdMc) ;


      if (iter==1) vpTRACE("\t\t compute the control law ") ;
      v = task.computeControlLaw() ;
      if (iter==1) task.print() ;

      if (iter==1) vpTRACE("\t\t send the camera velocity to the controller ") ;
      robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;


      std::cout << task.error.sumSquare() <<std::endl ; ;
    }

  vpTRACE("Display task information " ) ;
  task.print() ;
  task.kill();
  vpTRACE("Final camera location " ) ;
  std::cout << cMo << std::endl ;
}

