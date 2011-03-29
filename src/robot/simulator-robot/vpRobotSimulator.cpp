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
 * Description:
 * Basic class used to make robot simulators.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

#include <visp/vpConfig.h>
#include <visp/vpRobotSimulator.h>
#include <visp/vpSimulatorViper850.h>


#if defined(WIN32) || defined(VISP_HAVE_PTHREAD)


/*!
  Basic constructor
*/
vpRobotSimulator::vpRobotSimulator():vpWireFrameSimulator(), vpRobot()
{
  samplingTime = 10;
  velocity.resize(6);
  I.resize(480,640);
  I = 255;
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_D3D9) || defined(VISP_HAVE_GTK) 
  display.init(I, 0, 0,"The External view");
#endif
  robotStop = false;
  jointLimit = false;
  displayBusy = false;
  displayType = MODEL_3D;
  displayAllowed = true;
  singularityManagement = true;
  robotArms = NULL;
  
 //pid_t pid = getpid();
 // setpriority (PRIO_PROCESS, pid, 19);
}


vpRobotSimulator::vpRobotSimulator(bool display):vpWireFrameSimulator(), vpRobot()
{
  samplingTime = 10;
  velocity.resize(6);
  I.resize(480,640);
  I = 255;
  
  displayAllowed = display;
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_D3D9) || defined(VISP_HAVE_GTK) 
  if (display)
    this->display.init(I, 0, 0,"The External view");
#endif  
  robotStop = false;
  jointLimit = false;
  displayBusy = false;
  displayType = MODEL_3D;
  singularityManagement = true;
  robotArms = NULL;
  
  
 //pid_t pid = getpid();
 // setpriority (PRIO_PROCESS, pid, 19);
}



/*!
  Basic destructor
*/
vpRobotSimulator::~vpRobotSimulator()
{
}

/*!
  Initialize the display. It enables to choose the type of scene which will be used to display the object
  at the current position and at the desired position.
  
  It exists several default scenes you can use. Use the vpSceneObject and the vpSceneDesiredObject attributes to use them in this method. The corresponding files are stored in the "data" folder which is in the ViSP build directory.

  \param obj : Type of scene used to display the object at the current position.
  \param desiredObject : Type of scene used to display the object at the desired pose (in the internal view).
*/
void
vpRobotSimulator::initScene(vpSceneObject obj, vpSceneDesiredObject desiredObject)
{
  vpWireFrameSimulator::initScene(obj, desiredObject);
  displayCamera = false;
}

/*!
  Initialize the display. It enables to choose the type of scene which will be used to display the object
  at the current position and at the desired position.
  
  Here you can use the scene you want. You have to set the path to the .bnd file which is a scene file.
  It is also possible to use a vrml (.wrl) file.

  \param obj : Path to the scene file you want to use.
  \param desiredObject : Path to the scene file you want to use.
*/
void
vpRobotSimulator::initScene(const char* obj, const char* desiredObject)
{
  vpWireFrameSimulator::initScene(obj, desiredObject);
  displayCamera = false;
}

/*!
  Initialize the display. It enables to choose the type of object which will be used to display the object
  at the current position. The object at the desired position is not displayed.
  
  It exists several default scenes you can use. Use the vpSceneObject attributes to use them in this method. The corresponding files are stored in the "data" folder which is in the ViSP build directory.

  \param obj : Type of scene used to display the object at the current position.
*/
void
vpRobotSimulator::initScene(vpSceneObject obj)
{
  vpWireFrameSimulator::initScene(obj);
  displayCamera = false;
}

/*!
  Initialize the display. It enables to choose the type of scene which will be used to display the object
  at the current position. The object at the desired position is not displayed.
  
  Here you can use the scene you want. You have to set the path to the .bnd file which is a scene file, or the vrml file.

  \param obj : Path to the scene file you want to use.
*/
void
vpRobotSimulator::initScene(const char* obj)
{
  vpWireFrameSimulator::initScene(obj);
  displayCamera = false;
}

/*!
  Get the view of the camera's robot.
  
  According to the initialisation method you used, the current position and maybee the desired position of the object are displayed.
  
  \param I : The image where the internal view is displayed.
  
  \warning : The objects are displayed thanks to overlays. The image I is not modified.
*/
void
vpRobotSimulator::getInternalView(vpImage<vpRGBa> &I)
{
  
  if (!sceneInitialized)
    throw;

  double u;
  double v;
  //if(px_int != 1 && py_int != 1)
  // we assume px_int and py_int > 0
  if( (std::fabs(px_int-1.) > vpMath::maximum(px_int,1.)*std::numeric_limits<double>::epsilon()) 
      && (std::fabs(py_int-1) > vpMath::maximum(py_int,1.)*std::numeric_limits<double>::epsilon()))
  {
    u = (double)I.getWidth()/(2*px_int);
    v = (double)I.getHeight()/(2*py_int);
  }
  else
  {
    u = (double)I.getWidth()/(vpMath::minimum(I.getWidth(),I.getHeight()));
    v = (double)I.getHeight()/(vpMath::minimum(I.getWidth(),I.getHeight()));
  }

  float o44c[4][4],o44cd[4][4],x,y,z;
  Matrix id = IDENTITY_MATRIX;

  vpHomogeneousMatrix* fMit = new vpHomogeneousMatrix[size_fMi];
  get_fMi(fMit);
  this->cMo = fMit[size_fMi-1].inverse()*fMo;
  this->cMo = rotz*cMo;
  
  vp2jlc_matrix(cMo.inverse(),o44c);
  vp2jlc_matrix(cdMo.inverse(),o44cd);
  
  while (get_displayBusy()) vpTime::wait(2);

  add_vwstack ("start","cop", o44c[3][0],o44c[3][1],o44c[3][2]);
  x = o44c[2][0] + o44c[3][0];
  y = o44c[2][1] + o44c[3][1];
  z = o44c[2][2] + o44c[3][2];
  add_vwstack ("start","vrp", x,y,z);
  add_vwstack ("start","vpn", o44c[2][0],o44c[2][1],o44c[2][2]);
  add_vwstack ("start","vup", o44c[1][0],o44c[1][1],o44c[1][2]);
  add_vwstack ("start","window", -u, u, -v, v);
  if (displayObject)
    display_scene(id,this->scene,I, curColor);

  add_vwstack ("start","cop", o44cd[3][0],o44cd[3][1],o44cd[3][2]);
  x = o44cd[2][0] + o44cd[3][0];
  y = o44cd[2][1] + o44cd[3][1];
  z = o44cd[2][2] + o44cd[3][2];
  add_vwstack ("start","vrp", x,y,z);
  add_vwstack ("start","vpn", o44cd[2][0],o44cd[2][1],o44cd[2][2]);
  add_vwstack ("start","vup", o44cd[1][0],o44cd[1][1],o44cd[1][2]);
  add_vwstack ("start","window", -u, u, -v, v);
  if (displayDesiredObject)
  {
    if (desiredObject == D_TOOL) display_scene(o44cd,desiredScene,I, vpColor::red);
    else display_scene(id,desiredScene,I, desColor);
  }
  delete[] fMit;
  set_displayBusy(false);
}

/*!
  Get the view of the camera's robot.
  
  According to the initialisation method you used, the current position and maybee the desired position of the object are displayed.
  
  \param I : The image where the internal view is displayed.
  
  \warning : The objects are displayed thanks to overlays. The image I is not modified.
*/
void
vpRobotSimulator::getInternalView(vpImage<unsigned char> &I)
{
  
  if (!sceneInitialized)
    throw;

  double u;
  double v;
  //if(px_int != 1 && py_int != 1)
  // we assume px_int and py_int > 0
  if( (std::fabs(px_int-1.) > vpMath::maximum(px_int,1.)*std::numeric_limits<double>::epsilon()) 
      && (std::fabs(py_int-1) > vpMath::maximum(py_int,1.)*std::numeric_limits<double>::epsilon()))
  {
    u = (double)I.getWidth()/(2*px_int);
    v = (double)I.getHeight()/(2*py_int);
  }
  else
  {
    u = (double)I.getWidth()/(vpMath::minimum(I.getWidth(),I.getHeight()));
    v = (double)I.getHeight()/(vpMath::minimum(I.getWidth(),I.getHeight()));
  }

  float o44c[4][4],o44cd[4][4],x,y,z;
  Matrix id = IDENTITY_MATRIX;

  vpHomogeneousMatrix* fMit = new vpHomogeneousMatrix[size_fMi];
  get_fMi(fMit);
  this->cMo = fMit[size_fMi-1].inverse()*fMo;
  this->cMo = rotz*cMo;
  
  vp2jlc_matrix(cMo.inverse(),o44c);
  vp2jlc_matrix(cdMo.inverse(),o44cd);
  
  while (get_displayBusy()) vpTime::wait(2);

  add_vwstack ("start","cop", o44c[3][0],o44c[3][1],o44c[3][2]);
  x = o44c[2][0] + o44c[3][0];
  y = o44c[2][1] + o44c[3][1];
  z = o44c[2][2] + o44c[3][2];
  add_vwstack ("start","vrp", x,y,z);
  add_vwstack ("start","vpn", o44c[2][0],o44c[2][1],o44c[2][2]);
  add_vwstack ("start","vup", o44c[1][0],o44c[1][1],o44c[1][2]);
  add_vwstack ("start","window", -u, u, -v, v);
  if (displayObject)
  {
    display_scene(id,this->scene,I, curColor);
  }

  add_vwstack ("start","cop", o44cd[3][0],o44cd[3][1],o44cd[3][2]);
  x = o44cd[2][0] + o44cd[3][0];
  y = o44cd[2][1] + o44cd[3][1];
  z = o44cd[2][2] + o44cd[3][2];
  add_vwstack ("start","vrp", x,y,z);
  add_vwstack ("start","vpn", o44cd[2][0],o44cd[2][1],o44cd[2][2]);
  add_vwstack ("start","vup", o44cd[1][0],o44cd[1][1],o44cd[1][2]);
  add_vwstack ("start","window", -u, u, -v, v);
  if (displayDesiredObject)
  {
    if (desiredObject == D_TOOL) display_scene(o44cd,desiredScene,I, vpColor::red);
    else display_scene(id,desiredScene,I, desColor);
  }
  delete[] fMit;
  set_displayBusy(false);
}

/*!
  Get the pose between the object and the robot's camera.
     
  \return The pose between the object and the fixed world frame.
*/
vpHomogeneousMatrix
vpRobotSimulator::get_cMo()
{
  vpHomogeneousMatrix cMoTemp;
  vpHomogeneousMatrix* fMit = new vpHomogeneousMatrix[size_fMi];
  get_fMi(fMit);
  cMoTemp = fMit[size_fMi-1].inverse()*fMo;
  delete[] fMit;
  return cMoTemp;
}

#endif
