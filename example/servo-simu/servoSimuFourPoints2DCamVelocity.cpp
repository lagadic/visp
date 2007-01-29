/****************************************************************************
 *
 * $Id: servoSimuFourPoints2DCamVelocity.cpp,v 1.1 2007-01-29 11:05:59 asaunier Exp $
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
 * Simulation of a 2D visual servoing using 4 points as visual feature.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/


/*!
  \example servoSimuFourPoints2DCamVelocity.cpp
  \brief Simulation of a 2D visual servoing:
  - servo on 4 points,
  - eye-in-hand control law,
  - camera velocity are computed,
  - no display.

  Interaction matrix is computed as the mean of the current and desired
  interaction matrix.

*/

#include <visp/vpDebug.h>
#include <visp/vpConfig.h>


#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpServo.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpFeatureBuilder.h>

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
Simulation of a 2D visual servoing:\n\
- servo on 4 points,\n\
- eye-in-hand control law,\n\
- articular velocity are computed,\n\
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

  int i ;
  vpServo task ;
  vpRobotCamera robot ;


  cout << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << " Test program for vpServo "  <<endl ;
  cout << " Eye-in-hand task control,  articular velocity are computed" << endl ;
  cout << " Simulation " << endl ;
  cout << " task : servo 4 points " << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << endl ;


  vpTRACE("sets the initial camera location " ) ;
  vpHomogeneousMatrix cMo ;
  cMo[0][3] = 0.1 ;
  cMo[1][3] = 0.2 ;
  cMo[2][3] = 2 ;
  robot.setPosition(cMo) ;


  vpTRACE("sets the point coordinates in the world frame "  ) ;
  vpPoint point[4] ;
  point[0].setWorldCoordinates(-1,-1,0) ;
  point[1].setWorldCoordinates(1,-1,0) ;
  point[2].setWorldCoordinates(1,1,0) ;
  point[3].setWorldCoordinates(-1,1,0) ;

  vpTRACE("project : computes  the point coordinates in the camera frame and its 2D coordinates"  ) ;
  for (i = 0 ; i < 4 ; i++)
    point[i].track(cMo) ;

  vpTRACE("sets the desired position of the point ") ;
  vpFeaturePoint p[4] ;
  for (i = 0 ; i < 4 ; i++)
    vpFeatureBuilder::create(p[i], point[i])  ;  //retrieve x,y and Z of the vpPoint structure


  vpTRACE("sets the desired position of the point ") ;
  vpFeaturePoint pd[4] ;

  pd[0].buildFrom(-0.1,-0.1,1) ;
  pd[1].buildFrom(0.1,-0.1,1) ;
  pd[2].buildFrom(0.1,0.1,1) ;
  pd[3].buildFrom(-0.1,0.1,1) ;

  vpTRACE("define the task") ;
  vpTRACE("\t we want an eye-in-hand control law") ;
  vpTRACE("\t articular velocity are computed") ;
  task.setServo(vpServo::EYEINHAND_L_cVe_eJe) ;
  task.setInteractionMatrixType(vpServo::MEAN) ;


  vpTRACE("Set the position of the camera in the end-effector frame ") ;
  vpHomogeneousMatrix cMe ;
  vpTwistMatrix cVe(cMe) ;
  task.set_cVe(cVe) ;

  vpTRACE("Set the Jacobian (expressed in the end-effector frame)") ;
  vpMatrix eJe ;
  robot.get_eJe(eJe) ;
  task.set_eJe(eJe) ;

  vpTRACE("\t we want to see a point on a point..") ;
  for (i = 0 ; i < 4 ; i++)
    task.addFeature(p[i],pd[i]) ;

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


      if (iter==1)
	{
	  vpTRACE("Set the Jacobian (expressed in the end-effector frame)") ;
	  vpTRACE("since q is modified eJe is modified") ;
	}
      robot.get_eJe(eJe) ;
      task.set_eJe(eJe) ;


      if (iter==1) vpTRACE("\t\t get the robot position ") ;
      robot.getPosition(cMo) ;
      if (iter==1) vpTRACE("\t\t new point position ") ;
      for (i = 0 ; i < 4 ; i++)
	{
	  point[i].track(cMo) ;
	  //retrieve x,y and Z of the vpPoint structure
	  vpFeatureBuilder::create(p[i],point[i])  ;

	}

      if (iter==1) vpTRACE("\t\t compute the control law ") ;
      v = task.computeControlLaw() ;

      if (iter==1)
	{
	  vpTRACE("Display task information " ) ;
	  task.print() ;
	}

      if (iter==1) vpTRACE("\t\t send the camera velocity to the controller ") ;
      robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

      vpTRACE("\t\t || s - s* || = %.4f", task.error.sumSquare()) ;
    }

  vpTRACE("Display task information " ) ;
  task.print() ;
}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
