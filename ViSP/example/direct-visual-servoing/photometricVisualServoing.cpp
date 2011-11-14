/****************************************************************************
 *
 * $Id$
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
 *
 * Authors:
 * Eric Marchand
 * Christophe Collewet
 *
 *****************************************************************************/

/*!
  \example photometricVisualServoing.cpp

  Implemented from C. Collewet, E. Marchand, F. Chaumette. Visual
  servoing set free from image processing. In IEEE Int. Conf. on
  Robotics and Automation, ICRA'08, Pages 81-86, Pasadena, Californie,
  Mai 2008.
*/


#include <visp/vpDebug.h>

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageTools.h>

#include <visp/vpCameraParameters.h>
#include <visp/vpTime.h>
#include <visp/vpRobotCamera.h>

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayX.h>

#include <visp/vpFeatureLuminance.h>
#include <visp/vpParseArgv.h>

#include <visp/vpImageSimulator.h>
#include <stdlib.h>
#define  Z             1

#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>

// List of allowed command line options
#define GETOPTARGS	"cdi:n:h"

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath : Input image path.
  \param niter : Number of iterations.

*/
void usage(const char *name, const char *badparam, std::string ipath, int niter)
{
  fprintf(stdout, "\n\
Tracking of Surf key-points.\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] [-c] [-d] [-n <number of iterations>] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read \"ViSP-images/doisneau/doisneau.jpg\"\n\
     images. \n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
\n\
  -c\n\
     Disable the mouse click. Useful to automaze the \n\
     execution of this program without humain intervention.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -n %%d                                               %d\n\
     Number of iterations.\n\
\n\
  -h\n\
     Print the help.\n",
	  ipath.c_str(), niter);

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}
/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath : Input image path.
  \param click_allowed : Mouse click activation.
  \param display : Display activation.
  \param niter : Number of iterations.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &ipath,
		bool &click_allowed, bool &display, int &niter)
{
  const char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'c': click_allowed = false; break;
    case 'd': display = false; break;
    case 'i': ipath = optarg; break;
    case 'n': niter = atoi(optarg); break;
    case 'h': usage(argv[0], NULL, ipath, niter); return false; break;

    default:
      usage(argv[0], optarg, ipath, niter);
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath, niter);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}



int
main(int argc, const char ** argv)
{
  std::string env_ipath;
  std::string opt_ipath;
  std::string ipath;
  std::string filename;
  bool opt_click_allowed = true;
  bool opt_display = true;
  int opt_niter = 400;
  
  // Get the VISP_IMAGE_PATH environment variable value
  char *ptenv = getenv("VISP_INPUT_IMAGE_PATH");
  if (ptenv != NULL)
    env_ipath = ptenv;

  // Set the default input path
  if (! env_ipath.empty())
    ipath = env_ipath;


  // Read the command line options
  if (getOptions(argc, argv, opt_ipath, opt_click_allowed,
		 opt_display, opt_niter) == false) {
    return (-1);
  }

  // Get the option values
  if (!opt_ipath.empty())
    ipath = opt_ipath;

  // Compare ipath and env_ipath. If they differ, we take into account
  // the input path comming from the command line option
  if (!opt_ipath.empty() && !env_ipath.empty()) {
    if (ipath != env_ipath) {
      std::cout << std::endl
	   << "WARNING: " << std::endl;
      std::cout << "  Since -i <visp image path=" << ipath << "> "
	   << "  is different from VISP_IMAGE_PATH=" << env_ipath << std::endl
	   << "  we skip the environment variable." << std::endl;
    }
  }

  // Test if an input path is set
  if (opt_ipath.empty() && env_ipath.empty()){
    usage(argv[0], NULL, ipath, opt_niter);
    std::cerr << std::endl
	 << "ERROR:" << std::endl;
    std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH "
	 << std::endl
	 << "  environment variable to specify the location of the " << std::endl
	 << "  image path where test images are located." << std::endl << std::endl;
    exit(-1);
  }

  vpImage<unsigned char> Itexture ;
  filename = ipath +  vpIoTools::path("/ViSP-images/Klimt/Klimt.pgm");
  vpImageIo::read(Itexture,filename) ;

  vpColVector X[4];
  for (int i = 0; i < 4; i++) X[i].resize(3);
  // Top left corner
  X[0][0] = -0.3;
  X[0][1] = -0.215;
  X[0][2] = 0;
  
  // Top right corner
  X[1][0] = 0.3;
  X[1][1] = -0.215;
  X[1][2] = 0;
  
  // Bottom right corner
  X[2][0] = 0.3;
  X[2][1] = 0.215;
  X[2][2] = 0;
  
  //Bottom left corner
  X[3][0] = -0.3;
  X[3][1] = 0.215;
  X[3][2] = 0;

  vpImageSimulator sim;

  sim.setInterpolationType(vpImageSimulator::BILINEAR_INTERPOLATION) ;
  sim.init(Itexture, X);


    
  
  vpCameraParameters cam(870, 870, 160, 120);

  // ----------------------------------------------------------
  // Create the framegraber (here a simulated image)
  vpImage<unsigned char> I(240,320,0) ;
  vpImage<unsigned char> Id ;

  //camera desired position
  vpHomogeneousMatrix cdMo ;
  cdMo[2][3] = 1 ;

  //set the robot at the desired position
  sim.setCameraPosition(cdMo) ;
  sim.getImage(I,cam);  // and aquire the image Id
  Id = I ;


  // display the image
  #if defined VISP_HAVE_X11
  vpDisplayX d;
  #elif defined VISP_HAVE_GDI
  vpDisplayGDI d;
  #elif defined VISP_HAVE_GTK
  vpDisplayGTK d;
  #endif

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK) 
  if (opt_display) {
    d.init(I, 20, 10, "Photometric visual servoing : s") ;
    vpDisplay::display(I);
    vpDisplay::flush(I);
  }
  if (opt_display && opt_click_allowed) {
    std::cout << "Click in the image to continue..." << std::endl;
    vpDisplay::getClick(I) ;
  }
#endif


  // ----------------------------------------------------------
  // position the robot at the initial position
  // ----------------------------------------------------------

  //camera desired position
  vpHomogeneousMatrix cMo ;
  cMo.buildFrom(0,0,1.2,vpMath::rad(15),vpMath::rad(-5),vpMath::rad(20));
  
  //set the robot at the desired position
  sim.setCameraPosition(cMo) ;
  I =0 ;
  sim.getImage(I,cam);  // and aquire the image Id
  
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK) 
  if (opt_display) {
    vpDisplay::display(I) ;
    vpDisplay::flush(I) ;
  }
  if (opt_display && opt_click_allowed) {
    std::cout << "Click in the image to continue..." << std::endl;
    vpDisplay::getClick(I) ;
  }
#endif  
 
  vpImage<unsigned char> Idiff ;
  Idiff = I ;


  vpImageTools::imageDifference(I,Id,Idiff) ;


  // Affiche de l'image de difference
  #if defined VISP_HAVE_X11
  vpDisplayX d1;
  #elif defined VISP_HAVE_GDI
  vpDisplayGDI d1;
  #elif defined VISP_HAVE_GTK
  vpDisplayGTK d1;
  #endif
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK) 
  if (opt_display) {
    d1.init(Idiff, 40+(int)I.getWidth(), 10, "photometric visual servoing : s-s* ") ;
    vpDisplay::display(Idiff) ;
    vpDisplay::flush(Idiff) ;
  }
#endif
  // create the robot (here the INRIA Rennes Afma6)
  vpRobotCamera robot ;
  robot.init();
  robot.setSamplingTime(0.04);
  robot.setPosition(cMo) ;
    
  // ------------------------------------------------------
  // Visual feature, interaction matrix, error
  // s, Ls, Lsd, Lt, Lp, etc
  // ------------------------------------------------------

  // current visual feature built from the image 
  // (actually, this is the image...)
  vpFeatureLuminance sI ;
  sI.init( I.getHeight(), I.getWidth(), Z) ;
  sI.setCameraParameters(cam) ;
  sI.buildFrom(I) ;
  

  // desired visual feature built from the image 
  vpFeatureLuminance sId ;
  sId.init(I.getHeight(), I.getWidth(),  Z) ;
  sId.setCameraParameters(cam) ;
  sId.buildFrom(Id) ;

 
  
  // Matrice d'interaction, Hessien, erreur,...
  vpMatrix Lsd;   // matrice d'interaction a la position desiree
  vpMatrix Hsd;  // hessien a la position desiree
  vpMatrix H ; // Hessien utilise pour le levenberg-Marquartd
  vpColVector error ; // Erreur I-I*

  // Compute the interaction matrix
  // link the variation of image intensity to camera motion

  // here it is computed at the desired position
  sId.interaction(Lsd) ;

  
  // Compute the Hessian H = L^TL
  Hsd = Lsd.AtA() ;

  // Compute the Hessian diagonal for the Levenberg-Marquartd 
  // optimization process
  unsigned int n = 6 ;
  vpMatrix diagHsd(n,n) ;
  diagHsd.eye(n);
  for(unsigned int i = 0 ; i < n ; i++) diagHsd[i][i] = Hsd[i][i];



  // ------------------------------------------------------
  // Control law
  double lambda ; //gain
  vpColVector e ;
  vpColVector v ; // camera velocity send to the robot


  // ----------------------------------------------------------
  // Minimisation

  double mu ;  // mu = 0 : Gauss Newton ; mu != 0  : LM
  double lambdaGN;


  mu       =  0.01;
  lambda   = 30 ;
  lambdaGN = 30;






  // set a velocity control mode 
  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;

  // ----------------------------------------------------------
  int iter   = 1;
  int iterGN = 90 ; // swicth to Gauss Newton after iterGN iterations
  
  double normeError = 0;
  do
    {

      std::cout << "--------------------------------------------" << iter++ << std::endl ;


      //  Acquire the new image
      sim.setCameraPosition(cMo) ;
      sim.getImage(I,cam) ;
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK) 
      if (opt_display) {
      vpDisplay::display(I) ;
      vpDisplay::flush(I) ;
      }
#endif
      vpImageTools::imageDifference(I,Id,Idiff) ;
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_GTK) 
      if (opt_display) {
      vpDisplay::display(Idiff) ;
      vpDisplay::flush(Idiff) ;
      }
#endif
      // Compute current visual feature
      sI.buildFrom(I) ;

      
      // compute current error
      sI.error(sId,error) ;


      
      normeError = (error.sumSquare());
      std::cout << "|e| "<<normeError <<std::endl ;
      
     
     // double t = vpTime::measureTimeMs() ;


      // ---------- Methode Levenberg Marquardt --------------
      {
	if (iter > iterGN)
	  {
	    mu = 0.0001 ; 
	    lambda = lambdaGN;
	  }

	// Compute the levenberg Marquartd term
	{
	  H = ((mu * diagHsd) + Hsd).inverseByLU(); 
	}
	//	compute the control law 
	e = H * Lsd.t() *error ;

	v = - lambda*e;
      }

      std::cout << "lambda = " << lambda << "  mu = " << mu ;
      std::cout << " |Tc| = " << sqrt(v.sumSquare()) << std::endl;
      
      // send the robot velocity
      robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;
      robot.getPosition(cMo) ;

    }
  while(normeError > 10000 && iter < opt_niter);


  v = 0 ;
  robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;
 
}
