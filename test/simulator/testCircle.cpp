/****************************************************************************
 *
 * $Id: testCircle.cpp,v 1.13 2006-10-30 15:50:37 mtallur Exp $
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
  \example testCircle.cpp
  \brief Visual servoing experiment on a circle with a visualization
  from the camera and from an external view
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
#include <visp/vpFeatureEllipse.h>
#include <visp/vpCircle.h>
#include <visp/vpServo.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>


#define GETOPTARGS	"cdi:h"
#define SAVE 0

bool opt_click_allowed = true;
bool opt_display = true;

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
  %s [-p <input image path>] [-c] [-d] [-h]\n\
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
  -c\n\
     Disable keyboard input. Useful to automate the \n\
     execution of this program without humain intervention.\n\
\n\
  -d                                             \n\
     Disable the image display. This can be useful \n\
     for automatic tests using crontab under Unix or \n\
     using the task manager under Windows.\n\
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
bool getOptions(int argc, char **argv, string &ipath, bool &click_allowed, bool &display)
{
  char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'c': click_allowed = false; break;
    case 'd': display = false; break;
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

  vpPoseVector vcMo ;

  vcMo[0] = 0.3 ;
  vcMo[1] = 0.2 ;
  vcMo[2] = 3 ;
  vcMo[3] = 0 ;
  vcMo[4] = vpMath::rad(45)  ;
  vcMo[5] = vpMath::rad(40) ;
  vpHomogeneousMatrix cMo(vcMo) ; ;

  vpHomogeneousMatrix cMod ;
  cMod[0][3] = 0 ;
  cMod[1][3] = 0 ;
  cMod[2][3] = 1 ;

  int it =0 ;
  int pos = 2 ;
  while (pos!=0)
  {


  vpServo task ;
  vpRobotCamera robot ;

  cout << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << " Test program for vpServo "  <<endl ;
  cout << " Simulation " << endl ;
  cout << " task : servo a circle " << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << endl ;


  vpTRACE("sets the initial camera location " ) ;


  robot.setPosition(cMo) ;
  simu->setCameraPosition(cMo) ;


  if (pos==1)  cMod[2][3] = 0.32 ;

  vpTRACE("sets the circle coordinates in the world frame "  ) ;
  vpCircle circle ;
  circle.setWorldCoordinates(0,0,1,0,0,0,0.1) ;

  vpTRACE("sets the desired position of the visual feature ") ;
  vpFeatureEllipse pd ;
  circle.track(cMod) ;
  vpFeatureBuilder::create(pd,circle)  ;

  vpTRACE("project : computes  the circle coordinates in the camera frame and its 2D coordinates"  ) ;

  vpTRACE("sets the current position of the visual feature ") ;
  vpFeatureEllipse p ;
  circle.track(cMo) ;
  vpFeatureBuilder::create(p,circle)  ;

  vpTRACE("define the task") ;
  vpTRACE("\t we want an eye-in-hand control law") ;
  vpTRACE("\t robot is controlled in the camera frame") ;
  task.setServo(vpServo::EYEINHAND_CAMERA) ;
  task.setInteractionMatrixType(vpServo::CURRENT) ;

  vpTRACE("\t we want to see a circle on a circle..") ;
  cout << endl ;
  task.addFeature(p,pd) ;

  vpTRACE("\t set the gain") ;
  if (pos==2)  task.setLambda(0.0251) ;
  else task.setLambda(0.0251) ;


  vpTRACE("Display task information " ) ;
  task.print() ;

  vpTime::wait(1000); // Sleep 1s
  if (opt_click_allowed) {
    cout << "\nEnter a character to continue... " <<endl ;
    {    int a ; cin >> a ; }
  }

  int iter=0 ;
  vpTRACE("\t loop") ;
  int itermax ;
  if (pos==2) itermax = 75 ; else itermax = 100 ;
    char name[FILENAME_MAX] ;
  while(iter++<itermax)
  {
    cout << "---------------------------------------------" << iter <<endl ;
    vpColVector v ;

    if (iter==1) vpTRACE("\t\t get the robot position ") ;
    robot.getPosition(cMo) ;
    if (iter==1) vpTRACE("\t\t new circle position ") ;
    //retrieve x,y and Z of the vpCircle structure

    circle.track(cMo) ;
    vpFeatureBuilder::create(p,circle);

    if (iter==1) vpTRACE("\t\t compute the control law ") ;
    v = task.computeControlLaw() ;
    //  vpTRACE("computeControlLaw" ) ;
    cout << "Task rank: " << task.rankJ1 <<endl ;
    if (iter==1) vpTRACE("\t\t send the camera velocity to the controller ") ;
    robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

    simu->setCameraPosition(cMo) ;
    vpTime::wait(40) ;

    if(SAVE==1)
    {
      sprintf(name,"/tmp/image.%04d.external.png",it) ;
      cout << "Save " << name << endl ;
      simu->write(vpSimulator::EXTERNAL,name) ;
      sprintf(name,"/tmp/image.%04d.internal.png",iter) ;
      cout << "Save " << name << endl ;
      simu->write(vpSimulator::INTERNAL,name) ;
      it++ ;
    }
    //  vpTRACE("\t\t || s - s* || ") ;
    //  cout << task.error.sumSquare() <<endl ; ;
  }
  pos-- ;
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
  if (getOptions(argc, argv, opt_ipath, opt_click_allowed, opt_display) == false) {
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

  if (opt_display) {
  
    vpSimulator simu ;
    simu.initInternalViewer(300, 300) ;
    simu.initExternalViewer(300, 300) ;

    vpTime::wait(1000) ;
    simu.setZoomFactor(0.2) ;
    simu.addAbsoluteFrame() ;

    // Load the cad model
    filename = ipath +  vpIoTools::path("/ViSP-images/iv/circle.iv");
    simu.load(filename.c_str(),fMo) ;

    simu.setInternalCameraParameters(cam) ;
    simu.initApplication(&mainLoop) ;

    simu.mainLoop() ;
  }
}


#else
int
main()
{  vpTRACE("You should install Coin3D and SoQT") ;

}
#endif
