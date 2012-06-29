/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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
 * Demonstration of the wireframe simulator with a simple visual servoing
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

/*!
  \file servoSimuCylinder.cpp

  \brief Demonstration of the wireframe simulator with a simple visual servoing.
*/

/*!
  \example servoSimuCylinder.cpp

  Demonstration of the wireframe simulator with a simple visual servoing.
*/



#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpTime.h>

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpServo.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <visp/vpWireFrameSimulator.h>
#include <visp/vpCylinder.h>
#include <stdlib.h>
#define GETOPTARGS	"dh"

#if (defined (VISP_HAVE_X11) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_D3D9) || defined(VISP_HAVE_GTK))

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Demonstration of the wireframe simulator with a simple visual servoing.\n\
          \n\
The visual servoing consists in bringing the camera at a desired position\n\
from the object.\n\
          \n\
The visual features used to compute the pose of the camera and \n\
thus the control law are two lines. These features are computed thanks \n\
to the equation of a cylinder.\n\
          \n\
This demonstration explains also how to move the object around a world \n\
reference frame. Here, the movment is a rotation around the x and y axis \n\
at a given distance from the world frame. In fact the object trajectory \n\
is on a sphere whose center is the origin of the world frame.\n\
          \n\
SYNOPSIS\n\
  %s [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                          \n\
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
  \param display : Display activation.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, bool &display)
{
  const char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
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
  
  // Read the command line options
  if (getOptions(argc, argv, opt_display) == false) {
    exit (-1);
  }
  
  vpImage<vpRGBa> Iint(480,640,255);
  vpImage<vpRGBa> Iext(480,640,255);

#if defined VISP_HAVE_X11
  vpDisplayX display[2];
#elif defined VISP_HAVE_OPENCV
  vpDisplayOpenCV display[2];
#elif defined VISP_HAVE_GDI
  vpDisplayGDI display[2];
#elif defined VISP_HAVE_D3D9
  vpDisplayD3D display[2];
#elif defined VISP_HAVE_GTK
  vpDisplayGTK display[2];
#endif
  
  if (opt_display)
  {
    try
    {
      // Display size is automatically defined by the image (I) size
      display[0].init(Iint, 100, 100,"The internal view") ;
      display[1].init(Iext, 100, 100,"The first external view") ;
      vpDisplay::setWindowPosition (Iint, 0, 0);
      vpDisplay::setWindowPosition (Iext, 700, 0);
      vpDisplay::display(Iint);
      vpDisplay::flush(Iint);
      vpDisplay::display(Iext);
      vpDisplay::flush(Iext);
    }
    catch(...)
    {
      vpERROR_TRACE("Error while displaying the image") ;
      exit(-1);
    }
  }

  vpServo task;
  vpRobotCamera robot ;
  float sampling_time = 0.040f; // Sampling period in second
  robot.setSamplingTime(sampling_time);

  //cMo initial
  vpPoseVector cMoi(0,0.1,0.3,vpMath::rad(35),vpMath::rad(25),vpMath::rad(75));

  vpHomogeneousMatrix cMo(cMoi);
  
  // Create a cylinder
  vpCylinder cylinder(0,0,1,0,0,0,0.1);
  
  // Projection of the cylinder
  cylinder.track(cMo);

  //Set the current visual feature
  vpFeatureLine l[2];
  vpFeatureBuilder::create(l[0], cylinder, vpCylinder::line1);
  vpFeatureBuilder::create(l[1], cylinder, vpCylinder::line2);
  
  //cMo desired
  vpHomogeneousMatrix cdMo(vpHomogeneousMatrix(0.0,0.0,0.5,vpMath::rad(90),vpMath::rad(0),vpMath::rad(0)));
  
  // Projection of the cylinder
  cylinder.track(cdMo);

  vpFeatureLine ld[2];
  vpFeatureBuilder::create(ld[0], cylinder, vpCylinder::line1);
  vpFeatureBuilder::create(ld[1], cylinder, vpCylinder::line2);

  task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
  task.setInteractionMatrixType(vpServo::DESIRED);
  
  vpHomogeneousMatrix cMe;
  vpVelocityTwistMatrix cVe(cMe);
  task.set_cVe(cVe);

  vpMatrix eJe;
  robot.get_eJe(eJe);
  task.set_eJe(eJe);
  
  for (int i = 0 ; i < 2 ; i++)
    task.addFeature(l[i],ld[i]) ;
  
  task.setLambda(1);
  
  
  vpWireFrameSimulator sim;
  
  //Set the scene
  sim.initScene(vpWireFrameSimulator::CYLINDER, vpWireFrameSimulator::D_STANDARD);
  
  //Set the initial and the desired position of the camera.
  sim.setCameraPositionRelObj(cMoi) ;
  sim.setDesiredCameraPosition(cdMo);
  
  //Set the External camera position
  vpHomogeneousMatrix camMf(vpHomogeneousMatrix(0.0,0,3.5,vpMath::rad(0),vpMath::rad(30),0));
  sim.setExternalCameraPosition(camMf);
  
  //Move the object in the world reference frame
  sim.set_fMo(vpHomogeneousMatrix(0.0,0.0,0.0,0,0,0));
  
  //Set the parameters of the cameras (internal and external)
  vpCameraParameters camera(1000,1000,320,240);
  sim.setInternalCameraParameters(camera);
  sim.setExternalCameraParameters(camera);
  
  int stop = 10;
  
  if (opt_display)
  {
    stop = 2500;
    
    //Get the internal and external views
    sim.getInternalImage(Iint);
    sim.getExternalImage(Iext);

    //Display the object frame (current and desired position)
    vpDisplay::displayFrame(Iint,cMo,camera,0.2,vpColor::none);
    vpDisplay::displayFrame(Iint,cdMo,camera,0.2,vpColor::none);

    //Display the object frame the world reference frame and the camera frame
    vpDisplay::displayFrame(Iext,camMf*sim.get_fMo()*cMo.inverse(),camera,0.2,vpColor::none);
    vpDisplay::displayFrame(Iext,camMf*sim.get_fMo(),camera,0.2,vpColor::none);
    vpDisplay::displayFrame(Iext,camMf,camera,0.2,vpColor::none);

    vpDisplay::flush(Iint);
    vpDisplay::flush(Iext);
    
    std::cout << "Click on a display" << std::endl;
    while (!vpDisplay::getClick(Iint,false) && !vpDisplay::getClick(Iext,false)){};
  }
  
  robot.setPosition(sim.get_cMo());

  //Print the task
  task.print();

  int iter = 0;
  vpColVector v ;

  //Set the secondary task parameters
  vpColVector e1(6) ; e1 = 0 ;
  vpColVector e2(6) ; e2 = 0 ;
  vpColVector proj_e1 ;
  vpColVector proj_e2 ;
  iter = 0;
  double rapport = 0;
  double vitesse = 0.3;
  int tempo = 600;

  while(iter++ < stop)
  {
    if (opt_display)
    {
      vpDisplay::display(Iint) ;
      vpDisplay::display(Iext) ;
    }

    double t = vpTime::measureTimeMs();

    robot.get_eJe(eJe) ;
    task.set_eJe(eJe) ;

    robot.getPosition(cMo) ;

    cylinder.track(cMo) ;
    vpFeatureBuilder::create(l[0], cylinder, vpCylinder::line1);
    vpFeatureBuilder::create(l[1], cylinder, vpCylinder::line2);

    v = task.computeControlLaw() ;
    
    //Compute the velocity with the secondary task
    if ( iter%tempo < 200  &&  iter%tempo >= 0)
    {
      e2 = 0;
      e1[0] = -fabs(vitesse)  ;
      proj_e1 = task.secondaryTask(e1);
      rapport = -vitesse/proj_e1[0];
      proj_e1 *= rapport ;
      v += proj_e1 ;
    }

    if ( iter%tempo < 300 &&  iter%tempo >= 200)
    {
      e1 = 0;
      e2[1] = -fabs(vitesse)  ;
      proj_e2 = task.secondaryTask(e2);
      rapport = -vitesse/proj_e2[1];
      proj_e2 *= rapport ;
      v += proj_e2 ;
    }

    if ( iter%tempo < 500 &&  iter%tempo >= 300)
    {
      e2 = 0;
      e1[0] = -fabs(vitesse)  ;
      proj_e1 = task.secondaryTask(e1);
      rapport = vitesse/proj_e1[0];
      proj_e1 *= rapport ;
      v += proj_e1 ;
    }

    if ( iter%tempo < 600 &&  iter%tempo >= 500)
    {
      e1 = 0;
      e2[1] = -fabs(vitesse)  ;
      proj_e2 = task.secondaryTask(e2);
      rapport = vitesse/proj_e2[1];
      proj_e2 *= rapport ;
      v += proj_e2 ;
    }
    
    robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

    //Get the camera position
    robot.getPosition(cMo) ;
    
    //Compute the movement of the object around the world reference frame.
    vpHomogeneousMatrix a(0.0,0.0,0.0,vpMath::rad(0*iter),0,0);
    vpHomogeneousMatrix b(0,0.0,0.0,vpMath::rad(0*iter),0,0);
    vpHomogeneousMatrix c(0,0.0,0.0,0,vpMath::rad(0*iter),0);

    vpHomogeneousMatrix cMf = cMo*sim.get_fMo().inverse(); //The camera position in the world frame

    sim.set_fMo(b*c*a);  //Move the object in the simulator

    //Indicates to the task the movement of the object
    cMo = cMf*b*c*a;
    robot.setPosition(cMo);
    sim.setCameraPositionRelObj(cMo);

    if (opt_display)
    {
      //Get the internal and external views
      sim.getInternalImage(Iint);
      sim.getExternalImage(Iext);

      //Display the object frame (current and desired position)
      vpDisplay::displayFrame(Iint,cMo,camera,0.2,vpColor::none);
      vpDisplay::displayFrame(Iint,cdMo,camera,0.2,vpColor::none);

      //Display the object frame the world reference frame and the camera frame
      vpDisplay::displayFrame(Iext,sim.getExternalCameraPosition()*sim.get_fMo()*cMo.inverse(),camera,0.2,vpColor::none);
      vpDisplay::displayFrame(Iext,sim.getExternalCameraPosition()*sim.get_fMo(),camera,0.2,vpColor::none);

      vpDisplay::displayFrame(Iext,sim.getExternalCameraPosition(),camera,0.2,vpColor::none);;

      vpDisplay::flush(Iint);
      vpDisplay::flush(Iext);
    }

    vpTime::wait(t, sampling_time * 1000); // Wait 40 ms

    vpTRACE("\t\t || s - s* || ") ;
    std::cout << (task.getError() ).sumSquare() <<std::endl ;
  }

  task.print() ;
  task.kill() ;

  return 0;
}
#else
int
main()
{
  vpERROR_TRACE("You do not have X11, OpenCV, GDI, D3D9 or GTK display functionalities...");
}

#endif
