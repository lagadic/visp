/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
 * Demonstration of the wireframe simulator with a simple visual servoing
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

/*!
  \file servoSimuSphere.cpp

  \brief Demonstration of the wireframe simulator with a simple visual servoing.
*/

/*!
  \example servoSimuSphere.cpp

  Demonstration of the wireframe simulator with a simple visual servoing.
*/
#include <stdlib.h>

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
#include <visp/vpSphere.h>
#include <visp/vpGenericFeature.h>
#include <visp/vpServo.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpWireFrameSimulator.h>

#define GETOPTARGS	"dh"

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath : Input image path.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Demonstration of the wireframe simulator with a simple visual servoing.\n\
\n\
The visual servoing consists in bringing the camera at a desired position from the object.\n\
\n\
The virtual visual features used to compute the pose of the camera and thus the control law are special moments computed with the sphere's parameters.\n\
\n\
SYNOPSIS\n\
  %s [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
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


/*
  Computes the virtual visual features corresponding to the sphere and stores it in the generic feature.
  
  The visual feature vector is computed thanks to the following formula : s = {sx, sy, sz}
  sx = gx*h2/(sqrt(h2+1)
  sx = gy*h2/(sqrt(h2+1)
  sz = sqrt(h2+1)
  
  with gx and gy the center of gravity of the ellipse,
  with h2 = (gx²+gy²)/(4*n20*gy²+4*n02*gx²-8n11gxgy)
  with n20,n02,n11 the second order moments of the sphere
*/
void computeVisualFeatures(const vpSphere sphere, vpGenericFeature &s)
{
  double gx = sphere.get_x();
  double gy = sphere.get_y();
  double m02 = sphere.get_mu02();
  double m20 = sphere.get_mu20();
  double m11 = sphere.get_mu11();
  double h2;
  if (gx != 0 || gy != 0)
    h2 = (vpMath::sqr(gx)+vpMath::sqr(gy))/(4*m20*vpMath::sqr(gy)+4*m02*vpMath::sqr(gx)-8*m11*gx*gy);
  else
    h2 = 1/(4*m20);
  
  double sx = gx*h2/(sqrt(h2+1));
  double sy = gy*h2/(sqrt(h2+1));
  double sz = sqrt(h2+1); //(h2-(vpMath::sqr(sx)+vpMath::sqr(sy)-1))/(2*sqrt(h2));
  
  
  
  s.set_s(sx,sy,sz);
}

/*
  Computes the interaction matrix such as L = [-1/R*I3 [s]x]
  
  with R the radius of the sphere
  with I3 the 3x3 identity matrix
  with [s]x the skew matrix related to s
*/
void computeInteractionMatrix(const vpGenericFeature s,const vpSphere sphere, vpMatrix &L)
{
  L = 0;
  L[0][0] = -1/sphere.getR();
  L[1][1] = -1/sphere.getR();
  L[2][2] = -1/sphere.getR();
  
  double s0,s1,s2;
  s.get_s(s0,s1,s2);
  
  vpTranslationVector c(s0,s1,s2);
  vpMatrix sk;
  sk = c.skew();
  
  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
      L[i][j+3] = sk[i][j];
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
  vpImage<vpRGBa> Iext1(480,640,255);
  vpImage<vpRGBa> Iext2(480,640,255);

  #if defined VISP_HAVE_X11
  vpDisplayX display[3];
  #elif defined VISP_HAVE_OPENCV
  vpDisplayOpenCV display[3];
  #elif defined VISP_HAVE_GDI
  vpDisplayGDI display[3];
  #elif defined VISP_HAVE_D3D9
  vpDisplayD3D display[3];
  #elif defined VISP_HAVE_GTK
  vpDisplayGTK display[3];
  #endif
  
  if (opt_display)
  {
    try
    {
      // Display size is automatically defined by the image (I) size
      display[0].init(Iint, 100, 100,"The internal view") ;
      display[1].init(Iext1, 100, 100,"The first external view") ;
      display[2].init(Iext2, 100, 100,"The second external view") ;
      vpDisplay::setWindowPosition (Iint, 0, 0);
      vpDisplay::setWindowPosition (Iext1, 700, 0);
      vpDisplay::setWindowPosition (Iext2, 0, 550);
      vpDisplay::display(Iint);
      vpDisplay::flush(Iint);
      vpDisplay::display(Iext1);
      vpDisplay::flush(Iext1);
      vpDisplay::display(Iext2);
      vpDisplay::flush(Iext2);
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
  vpPoseVector cMoi(0,0.1,2.0,vpMath::rad(35),vpMath::rad(25),0);

  vpHomogeneousMatrix cMo(cMoi);
  robot.setPosition(cMo);
  
  //The sphere
  vpSphere sphere(0,0,0,0.15);
  
  // Projection of the sphere
  sphere.track(cMo);

  //Set the current visual feature
  vpGenericFeature s(3);
  computeVisualFeatures(sphere, s);
  
  //cMo desired
  vpHomogeneousMatrix cdMo(vpHomogeneousMatrix(0.0,0.0,0.8,vpMath::rad(0),vpMath::rad(0),vpMath::rad(0)));
  
  // Projection of the points
  sphere.track(cdMo);

  vpGenericFeature sd(3);
  computeVisualFeatures(sphere, sd);
  
  vpMatrix L(3,6);
  computeInteractionMatrix(sd,sphere,L);
  sd.setInteractionMatrix(L);
    
  task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
  task.setInteractionMatrixType(vpServo::DESIRED);
  
  vpHomogeneousMatrix cMe;
  vpVelocityTwistMatrix cVe(cMe);
  task.set_cVe(cVe);

  vpMatrix eJe;
  robot.get_eJe(eJe);
  task.set_eJe(eJe);

  task.addFeature(s,sd) ;
  
  task.setLambda(10);
  
  
  vpWireFrameSimulator sim;
  
  //Set the scene
  sim.initScene(vpWireFrameSimulator::SPHERE, vpWireFrameSimulator::MOTIF_STANDARD);
  
  //Set the initial and the desired position of the camera.
  sim.setCameraPosition(cMoi) ;
  sim.setDesiredCameraPosition(cdMo);
  
  //Set the External camera position
  vpHomogeneousMatrix camMw(vpHomogeneousMatrix(0.0,0,3.5,vpMath::rad(0),vpMath::rad(30),0));
  sim.setExternalCameraPosition(camMw);
  
  //Computes the position of a camera which is fixed in the object frame
  vpHomogeneousMatrix camoMw;
  vpHomogeneousMatrix temp(vpHomogeneousMatrix(0,0.0,2.5,0,vpMath::rad(140),0)*(sim.get_wMo().inverse()));
  vpTranslationVector T;
  vpRotationMatrix R;
  temp.extract(T);
  temp.extract(R);
  camoMw.buildFrom(T,R);
  
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
    sim.getExternalImage(Iext1);
    sim.getExternalImage(Iext2,camoMw);

    //Display the object frame (current and desired position)
    vpDisplay::displayFrame(Iint,cMo,camera,0.2,vpColor::none);
    vpDisplay::displayFrame(Iint,cdMo,camera,0.2,vpColor::none);
  
    //Display the object frame the world reference frame and the camera frame
    vpDisplay::displayFrame(Iext1,camMw*sim.get_wMo()*cMo.inverse(),camera,0.2,vpColor::none);
    vpDisplay::displayFrame(Iext1,camMw*sim.get_wMo(),camera,0.2,vpColor::none);
    vpDisplay::displayFrame(Iext1,camMw,camera,0.2,vpColor::none);

    //Display the world reference frame and the object frame
    vpDisplay::displayFrame(Iext2,camoMw,camera,0.2,vpColor::none);
    vpDisplay::displayFrame(Iext2,camoMw*sim.get_wMo(),camera,0.05,vpColor::none);

    vpDisplay::flush(Iint);
    vpDisplay::flush(Iext1);
    vpDisplay::flush(Iext2);
    
    std::cout << "Click on a display" << std::endl;
    while (!vpDisplay::getClick(Iint,false) && !vpDisplay::getClick(Iext1,false) && !vpDisplay::getClick(Iext2,false)){};
  }

  //Print the task
  task.print() ;

  int iter = 0;
  vpColVector v ;

  while(iter++ < stop)
  {
    if (opt_display)
    {
      vpDisplay::display(Iint) ;
      vpDisplay::display(Iext1) ;
      vpDisplay::display(Iext2) ;
    }

    double t = vpTime::measureTimeMs();

    robot.get_eJe(eJe) ;
    task.set_eJe(eJe) ;

    robot.getPosition(cMo) ;

    sphere.track(cMo);

    //Set the current visual feature
    computeVisualFeatures(sphere, s);

    v = task.computeControlLaw() ;
    robot.setVelocity(vpRobot::CAMERA_FRAME, v);
    sim.setCameraPosition(cMo);

    //Compute the position of the external view which is fixed in the object frame
    vpHomogeneousMatrix temp(vpHomogeneousMatrix(0,0.0,2.5,0,vpMath::rad(150),0)*(sim.get_wMo().inverse()));
    vpTranslationVector T;
    vpRotationMatrix R;
    temp.extract(T);
    temp.extract(R);
    camoMw.buildFrom(T,R);

    if (opt_display)
    {
      //Get the internal and external views
      sim.getInternalImage(Iint);
      sim.getExternalImage(Iext1);
      sim.getExternalImage(Iext2,camoMw);

      //Display the object frame (current and desired position)
      vpDisplay::displayFrame(Iint,cMo,camera,0.2,vpColor::none);
      vpDisplay::displayFrame(Iint,cdMo,camera,0.2,vpColor::none);
    
      //Display the object frame the world reference frame and the camera frame
      vpDisplay::displayFrame(Iext1,sim.get_cMw()*sim.get_wMo()*cMo.inverse(),camera,0.2,vpColor::none);
      vpDisplay::displayFrame(Iext1,sim.get_cMw()*sim.get_wMo(),camera,0.2,vpColor::none);

      vpDisplay::displayFrame(Iext1,sim.get_cMw(),camera,0.2,vpColor::none);

      //Display the world reference frame and the object frame
      vpDisplay::displayFrame(Iext2,camoMw,camera,0.2,vpColor::none);
      vpDisplay::displayFrame(Iext2,camoMw*sim.get_wMo(),camera,0.05,vpColor::none);

      vpDisplay::flush(Iint);
      vpDisplay::flush(Iext1);
      vpDisplay::flush(Iext2);
    }

    vpTime::wait(t, sampling_time * 1000); // Wait 40 ms

    vpTRACE("\t\t || s - s* || ") ;
    std::cout << task.error.sumSquare() <<std::endl ;
  }

  task.print() ;
  task.kill() ;

  return 0;
}
