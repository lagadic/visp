/****************************************************************************
 *
 * $Id: testServoThetaU.cpp,v 1.4 2006-09-11 16:14:58 fspindle Exp $
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
 * Simulation of a visual servoing using theta U visual features.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      testServoThetaU.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: testServoThetaU.cpp,v 1.4 2006-09-11 16:14:58 fspindle Exp $
 *
 * Description
 * ============
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in the camera frame
 *   using theta U visual feature
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \example testServoThetaU.cpp
  \brief Simulation of a visual servoing using theta U visual features:
  - eye-in-hand control law,
  - velocity computed in the camera frame,
  - no display.
*/


#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpFeatureTranslation.h>
#include <visp/vpServo.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpDebug.h>
#include <visp/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS	"h"

/*!

  Print the program options.

  \param ipath: Input image path.

*/
void usage(char *name, char *badparam)
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

}

/*!

  Set the program options.

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
    cerr << "ERROR: " << endl;
    cerr << "  Bad argument " << optarg << endl << endl;
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

  cout << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << " Test program for vpServo "  <<endl ;
  cout << " Eye-in-hand task control, velocity computed in the camera frame" << endl ;
  cout << " Simulation " << endl ;
  cout << " task :  servo using theta U visual feature " << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << endl ;


  vpTRACE("sets the initial camera location " ) ;
  vpPoseVector c_r_o(0.1,0.2,2,
		     vpMath::rad(20), vpMath::rad(10),  vpMath::rad(50)
		     ) ;

  vpCTRACE ; cout << endl ;
  vpHomogeneousMatrix cMo(c_r_o) ;
  vpCTRACE ; cout << endl ;
  robot.setPosition(cMo) ;
  vpCTRACE ; cout << endl ;

  vpTRACE("sets the desired camera location " ) ;
  vpPoseVector cd_r_o(0,0,1,
		      vpMath::rad(0),vpMath::rad(0),vpMath::rad(0)) ;
  vpHomogeneousMatrix cdMo(cd_r_o) ;


  vpTRACE("compute the rotation that the camera has to realize "  ) ;
  vpHomogeneousMatrix cdMc ;
  cdMc = cdMo*cMo.inverse() ;
  vpFeatureThetaU tu ;
  tu.buildFrom(cdMc) ;


  vpTRACE("define the task") ;
  vpTRACE("\t we want an eye-in-hand control law") ;
  vpTRACE("\t robot is controlled in the camera frame") ;
  task.setServo(vpServo::EYEINHAND_CAMERA) ;
  task.setInteractionMatrixType(vpServo::DESIRED) ;

  task.addFeature(tu) ;

  vpTRACE("\t set the gain") ;
  task.setLambda(0.1) ;


  vpTRACE("Display task information " ) ;
  task.print() ;

  int iter=0 ;
  vpTRACE("\t loop") ;
  while(iter++<200)
  {
    cout << "---------------------------------------------" << iter <<endl ;
    vpColVector v ;

    if (iter==1) vpTRACE("\t\t get the robot position ") ;
    robot.getPosition(cMo) ;

    if (iter==1) vpTRACE("\t\t new rotation to realize ") ;
    cdMc = cdMo*cMo.inverse() ;
    tu.buildFrom(cdMc) ;


    if (iter==1) vpTRACE("\t\t compute the control law ") ;
    v = task.computeControlLaw() ;
    if (iter==1) task.print() ;

    if (iter==1) vpTRACE("\t\t send the camera velocity to the controller ") ;
    robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;


    cout << task.error.sumSquare() <<endl ; ;
  }

  vpTRACE("Display task information " ) ;
  task.print() ;
}

