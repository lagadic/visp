/****************************************************************************
 *
 * $Id: testInteractiveInit.cpp,v 1.11 2006-09-18 09:18:59 fspindle Exp $
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
 * Simulation of a visual servoing with visualization.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testInteractiveInit.cpp
  \brief Visual servoing experiment on 4 points with a visualization
  from the camera and from an external view.
*/

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>


#ifdef VISP_HAVE_SOQT

#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpTime.h>
#include <visp/vpSimulator.h>


#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpServo.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>

#define GETOPTARGS	"i:h"
#define SAVE 0

/*

  Print the program options.

  \param ipath: Input image path.
  \param opath : Output image path.
  \param user : Username.

 */
void usage(char *name, char *badparam, string ipath)
{
  fprintf(stdout, "\n\
Test image conversions.\n\
\n\
SYNOPSIS\n\
  %s [-p <input image path>] [-h]\n\
", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read \"ViSP-images/iv/4points.iv\"\n\
     cad model.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
\n\
  -h\n\
     Print the help.\n\n",
	  ipath.c_str());

}

/*!

  Set the program options.

  \param ipath: Input image path.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, char **argv, string &ipath)
{
  char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'i': ipath = optarg; break;
    case 'h': usage(argv[0], NULL, ipath); return false; break;

    default:
      usage(argv[0], optarg, ipath); return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath);
    cerr << "ERROR: " << endl;
    cerr << "  Bad argument " << optarg << endl << endl;
    return false;
  }

  return true;
}

static
void *mainLoop (void *_simu)
{
  vpSimulator *simu = (vpSimulator *)_simu ;
  simu->initMainApplication() ;

  while (1)
  {int i ;

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
    vpPoseVector vcMo ;

    vcMo[0] = 0.3 ;
    vcMo[1] = 0.2 ;
    vcMo[2] = 3 ;
    vcMo[3] = 0 ;
    vcMo[4] = +vpMath::rad(0)  ;
    vcMo[5] = vpMath::rad(40) ;

    vpHomogeneousMatrix cMo(vcMo)  ;
    robot.setPosition(cMo) ;
    simu->setCameraPosition(cMo) ;

    simu->getCameraPosition(cMo) ;
    robot.setPosition(cMo) ;

    vpCameraParameters cam ;

    vpTRACE("sets the point coordinates in the world frame "  ) ;
    vpPoint point[4] ;
    point[0].setWorldCoordinates(-0.1,-0.1,0) ;
    point[1].setWorldCoordinates(0.1,-0.1,0) ;
    point[2].setWorldCoordinates(0.1,0.1,0) ;
    point[3].setWorldCoordinates(-0.1,0.1,0) ;

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
    task.setInteractionMatrixType(vpServo::CURRENT) ;


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
    task.setLambda(0.05) ;


    vpTRACE("Display task information " ) ;
    task.print() ;

    vpTime::wait(1000); // Sleep 1s
    cout << "\nEnter a character to continue or CTRL-C to quit... " <<endl ;
    {    int a ; cin >> a ; }

    int iter=0 ;
    vpTRACE("\t loop") ;
    while(iter++<100)
    {
      vpColVector v ;

      robot.get_eJe(eJe) ;
      task.set_eJe(eJe) ;

      robot.getPosition(cMo) ;
      for (i = 0 ; i < 4 ; i++)
      {
	point[i].track(cMo) ;
	vpFeatureBuilder::create(p[i],point[i])  ;
      }

      v = task.computeControlLaw() ;
      robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

      vpTime::wait(100) ;

      simu->setCameraPosition(cMo) ;



    }
    vpTRACE("Display task information " ) ;
    task.print() ;
    cout << "\nEnter a character to continue..." <<endl ;
    {    int a ; cin >> a ; }

  }

  simu->closeMainApplication() ;

  cout << "\nEnter CTRL-C to quit... " <<endl ;

  void *a=NULL ;
  return a ;
  // return (void *);
}


int
main(int argc, char ** argv)
{
  string env_ipath;
  string opt_ipath;
  string ipath;
  string filename;
  string username;

  // Get the VISP_IMAGE_PATH environment variable value
  char *ptenv = getenv("VISP_INPUT_IMAGE_PATH");
  if (ptenv != NULL)
    env_ipath = ptenv;

  // Set the default input path
  if (! env_ipath.empty())
    ipath = env_ipath;

  // Read the command line options
  if (getOptions(argc, argv, opt_ipath) == false) {
    exit (-1);
  }

  // Get the option values
  if (!opt_ipath.empty())
    ipath = opt_ipath;

  // Compare ipath and env_ipath. If they differ, we take into account
  // the input path comming from the command line option
  if (opt_ipath.empty()) {
    if (ipath != env_ipath) {
      cout << endl
	   << "WARNING: " << endl;
      cout << "  Since -i <visp image path=" << ipath << "> "
	   << "  is different from VISP_IMAGE_PATH=" << env_ipath << endl
	   << "  we skip the environment variable." << endl;
    }
  }

  // Test if an input path is set
  if (opt_ipath.empty() && env_ipath.empty()){
    usage(argv[0], NULL, ipath);
    cerr << endl
	 << "ERROR:" << endl;
    cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH "
	 << endl
	 << "  environment variable to specify the location of the " << endl
	 << "  image path where test images are located." << endl << endl;
    exit(-1);
  }

  vpCameraParameters cam ;
  vpHomogeneousMatrix fMo ; fMo[2][3] = 0 ;

  vpSimulator simu ;
  simu.initInternalViewer(300, 300) ;
  simu.initExternalViewer(300, 300) ;

  vpTime::wait(1000) ;
  simu.setZoomFactor(0.2) ;

  // Load the cad model
  filename = ipath +  vpIoTools::path("/ViSP-images/iv/4points.iv");
  simu.load(filename.c_str()) ;

  simu.setInternalCameraParameters(cam) ;
  simu.setExternalCameraParameters(cam) ;
  simu.initApplication(&mainLoop) ;

  simu.mainLoop() ;
}

#else
int
main()
{  vpTRACE("You should install Coin3D and SoQT") ;

}
#endif
