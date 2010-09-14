/****************************************************************************
 *
 * $Id:  2457 2010-01-07 10:41:18Z Eric Marchand $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
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


#include <visp/vpConfig.h>
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


using namespace std ;



#define  Z             1



int
main(int argc, char *argv[])
{
  

  vpImage<unsigned char> Itexture ;
  vpImageIo::read(Itexture,"/udd/marchand/images/doisneau2.jpeg") ;

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
  d.init(I, 20, 10, "Photometric visual servoing : s") ;

  vpDisplay::display(I);
  vpDisplay::flush(I);
  vpDisplay::getClick(I) ;


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
  
  
  vpDisplay::display(I) ;
  vpDisplay::flush(I) ;
  vpDisplay::getClick(I) ;
  
 
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
  d1.init(Idiff, 680, 10, "photometric visual servoing : s-s* ") ;
  vpDisplay::display(Idiff) ;
  vpDisplay::flush(Idiff) ;




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
  int n = 6 ;
  vpMatrix diagHsd(n,n) ;
  diagHsd.eye(n);
  for(int i = 0 ; i < n ; i++) diagHsd[i][i] = Hsd[i][i];



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

  do
    {

      cout << "--------------------------------------------" << iter++ << endl ;


      //  Acquire the new image
      sim.setCameraPosition(cMo) ;
      sim.getImage(I,cam) ;
      vpDisplay::display(I) ;
      vpDisplay::flush(I) ;

      vpImageTools::imageDifference(I,Id,Idiff) ;
      vpDisplay::display(Idiff) ;
      vpDisplay::flush(Idiff) ;

      // Compute current visual feature
      sI.buildFrom(I) ;

      
      // compute current error
      sI.error(sId,error) ;


      
      double normeError = (error.sumSquare());
      cout << "|e| "<<normeError <<endl ;
      
     
      double t = vpTime::measureTimeMs() ;


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

      cout << "lambda = " << lambda << "  mu = " << mu ;
      cout << " |Tc| = " << sqrt(v.sumSquare()) << endl;
      
      // send the robot velocity
      robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;
      robot.getPosition(cMo) ;

    }
  while(iter < 1800);


  v = 0 ;
  robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;
 
}
