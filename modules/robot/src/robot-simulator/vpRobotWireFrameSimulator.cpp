/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Basic class used to make robot simulators.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_MODULE_GUI) && ((defined(_WIN32) && !defined(WINRT_8_0)) || defined(VISP_HAVE_PTHREAD))
#include <visp3/robot/vpRobotWireFrameSimulator.h>
#include <visp3/robot/vpSimulatorViper850.h>

#include "../wireframe-simulator/vpBound.h"
#include "../wireframe-simulator/vpScene.h"
#include "../wireframe-simulator/vpVwstack.h"

/*!
  Basic constructor
*/
vpRobotWireFrameSimulator::vpRobotWireFrameSimulator()
  : vpWireFrameSimulator(), vpRobotSimulator(), I(), tcur(0), tprev(0), robotArms(NULL), size_fMi(8), fMi(NULL),
    artCoord(), artVel(), velocity(),
#if defined(_WIN32)
#elif defined(VISP_HAVE_PTHREAD)
    thread(), attr(),
#endif
    mutex_fMi(), mutex_artVel(), mutex_artCoord(), mutex_velocity(), mutex_display(), displayBusy(false),
    robotStop(false), jointLimit(false), jointLimitArt(false), singularityManagement(true), cameraParam(),
#if defined(VISP_HAVE_DISPLAY)
    display(),
#endif
    displayType(MODEL_3D), displayAllowed(true), constantSamplingTimeMode(false), setVelocityCalled(false),
    verbose_(false)
{
  setSamplingTime(0.010);
  velocity.resize(6);
  I.resize(480, 640);
  I = 255;
#if defined(VISP_HAVE_DISPLAY)
  display.init(I, 0, 0, "The External view");
#endif

  // pid_t pid = getpid();
  // setpriority (PRIO_PROCESS, pid, 19);
}

/*!
  Default constructor.
  \param do_display : When true, enables the display of the external view.
  */
vpRobotWireFrameSimulator::vpRobotWireFrameSimulator(bool do_display)
  : vpWireFrameSimulator(), vpRobotSimulator(), I(), tcur(0), tprev(0), robotArms(NULL), size_fMi(8), fMi(NULL),
    artCoord(), artVel(), velocity(),
#if defined(_WIN32)
#elif defined(VISP_HAVE_PTHREAD)
    thread(), attr(),
#endif
    /* thread(), attr(), */ mutex_fMi(), mutex_artVel(), mutex_artCoord(), mutex_velocity(), mutex_display(),
    displayBusy(false), robotStop(false), jointLimit(false), jointLimitArt(false), singularityManagement(true),
    cameraParam(),
#if defined(VISP_HAVE_DISPLAY)
    display(),
#endif
    displayType(MODEL_3D), displayAllowed(do_display), constantSamplingTimeMode(false), setVelocityCalled(false),
    verbose_(false)
{
  setSamplingTime(0.010);
  velocity.resize(6);
  I.resize(480, 640);
  I = 255;

#if defined(VISP_HAVE_DISPLAY)
  if (do_display)
    this->display.init(I, 0, 0, "The External view");
#endif

  // pid_t pid = getpid();
  // setpriority (PRIO_PROCESS, pid, 19);
}

/*!
  Basic destructor
*/
vpRobotWireFrameSimulator::~vpRobotWireFrameSimulator() {}

/*!
  Initialize the display. It enables to choose the type of scene which will be
  used to display the object at the current position and at the desired
  position.

  It exists several default scenes you can use. Use the vpSceneObject and the
  vpSceneDesiredObject attributes to use them in this method. The
  corresponding files are stored in the "data" folder which is in the ViSP
  build directory.

  \param obj : Type of scene used to display the object at the current
  position. \param desired_object : Type of scene used to display the object
  at the desired pose (in the internal view).
*/
void vpRobotWireFrameSimulator::initScene(const vpSceneObject &obj, const vpSceneDesiredObject &desired_object)
{
  if (displayCamera) {
    free_Bound_scene(&(this->camera));
  }
  vpWireFrameSimulator::initScene(obj, desired_object);
  if (displayCamera) {
    free_Bound_scene(&(this->camera));
  }
  displayCamera = false;
}

/*!
  Initialize the display. It enables to choose the type of scene which will be
  used to display the object at the current position and at the desired
  position.

  Here you can use the scene you want. You have to set the path to the .bnd
  file which is a scene file. It is also possible to use a vrml (.wrl) file.

  \param obj : Path to the scene file you want to use.
  \param desired_object : Path to the scene file you want to use.
*/
void vpRobotWireFrameSimulator::initScene(const char *obj, const char *desired_object)
{
  if (displayCamera) {
    free_Bound_scene(&(this->camera));
  }
  vpWireFrameSimulator::initScene(obj, desired_object);
  if (displayCamera) {
    free_Bound_scene(&(this->camera));
  }
  displayCamera = false;
}

/*!
  Initialize the display. It enables to choose the type of object which will
  be used to display the object at the current position. The object at the
  desired position is not displayed.

  It exists several default scenes you can use. Use the vpSceneObject
  attributes to use them in this method. The corresponding files are stored in
  the "data" folder which is in the ViSP build directory.

  \param obj : Type of scene used to display the object at the current
  position.
*/
void vpRobotWireFrameSimulator::initScene(const vpSceneObject &obj)
{
  if (displayCamera) {
    free_Bound_scene(&(this->camera));
  }
  vpWireFrameSimulator::initScene(obj);
  if (displayCamera) {
    free_Bound_scene(&(this->camera));
  }
  displayCamera = false;
}

/*!
  Initialize the display. It enables to choose the type of scene which will be
  used to display the object at the current position. The object at the
  desired position is not displayed.

  Here you can use the scene you want. You have to set the path to the .bnd
  file which is a scene file, or the vrml file.

  \param obj : Path to the scene file you want to use.
*/
void vpRobotWireFrameSimulator::initScene(const char *obj)
{
  if (displayCamera) {
    free_Bound_scene(&(this->camera));
  }
  vpWireFrameSimulator::initScene(obj);
  if (displayCamera) {
    free_Bound_scene(&(this->camera));
  }
  displayCamera = false;
}

/*!
  Get the view of the camera's robot.

  According to the initialisation method you used, the current position and
  maybee the desired position of the object are displayed.

  \param I_ : The image where the internal view is displayed.

  \warning : The objects are displayed thanks to overlays. The image I is not
  modified.
*/
void vpRobotWireFrameSimulator::getInternalView(vpImage<vpRGBa> &I_)
{

  if (!sceneInitialized)
    throw;

  double u;
  double v;
  // if(px_int != 1 && py_int != 1)
  // we assume px_int and py_int > 0
  if ((std::fabs(px_int - 1.) > vpMath::maximum(px_int, 1.) * std::numeric_limits<double>::epsilon()) &&
      (std::fabs(py_int - 1) > vpMath::maximum(py_int, 1.) * std::numeric_limits<double>::epsilon())) {
    u = (double)I_.getWidth() / (2 * px_int);
    v = (double)I_.getHeight() / (2 * py_int);
  } else {
    u = (double)I_.getWidth() / (vpMath::minimum(I_.getWidth(), I_.getHeight()));
    v = (double)I_.getHeight() / (vpMath::minimum(I_.getWidth(), I_.getHeight()));
  }

  float o44c[4][4], o44cd[4][4], x, y, z;
  Matrix id = IDENTITY_MATRIX;

  vpHomogeneousMatrix *fMit = new vpHomogeneousMatrix[size_fMi];
  get_fMi(fMit);
  this->cMo = fMit[size_fMi - 1].inverse() * fMo;
  this->cMo = rotz * cMo;

  vp2jlc_matrix(cMo.inverse(), o44c);
  vp2jlc_matrix(cdMo.inverse(), o44cd);

  while (get_displayBusy())
    vpTime::wait(2);

  add_vwstack("start", "cop", o44c[3][0], o44c[3][1], o44c[3][2]);
  x = o44c[2][0] + o44c[3][0];
  y = o44c[2][1] + o44c[3][1];
  z = o44c[2][2] + o44c[3][2];
  add_vwstack("start", "vrp", x, y, z);
  add_vwstack("start", "vpn", o44c[2][0], o44c[2][1], o44c[2][2]);
  add_vwstack("start", "vup", o44c[1][0], o44c[1][1], o44c[1][2]);
  add_vwstack("start", "window", -u, u, -v, v);
  if (displayObject)
    display_scene(id, this->scene, I_, curColor);

  add_vwstack("start", "cop", o44cd[3][0], o44cd[3][1], o44cd[3][2]);
  x = o44cd[2][0] + o44cd[3][0];
  y = o44cd[2][1] + o44cd[3][1];
  z = o44cd[2][2] + o44cd[3][2];
  add_vwstack("start", "vrp", x, y, z);
  add_vwstack("start", "vpn", o44cd[2][0], o44cd[2][1], o44cd[2][2]);
  add_vwstack("start", "vup", o44cd[1][0], o44cd[1][1], o44cd[1][2]);
  add_vwstack("start", "window", -u, u, -v, v);
  if (displayDesiredObject) {
    if (desiredObject == D_TOOL)
      display_scene(o44cd, desiredScene, I_, vpColor::red);
    else
      display_scene(id, desiredScene, I_, desColor);
  }
  delete[] fMit;
  set_displayBusy(false);
}

/*!
  Get the view of the camera's robot.

  According to the initialisation method you used, the current position and
  maybee the desired position of the object are displayed.

  \param I_ : The image where the internal view is displayed.

  \warning : The objects are displayed thanks to overlays. The image I is not
  modified.
*/
void vpRobotWireFrameSimulator::getInternalView(vpImage<unsigned char> &I_)
{

  if (!sceneInitialized)
    throw;

  double u;
  double v;
  // if(px_int != 1 && py_int != 1)
  // we assume px_int and py_int > 0
  if ((std::fabs(px_int - 1.) > vpMath::maximum(px_int, 1.) * std::numeric_limits<double>::epsilon()) &&
      (std::fabs(py_int - 1) > vpMath::maximum(py_int, 1.) * std::numeric_limits<double>::epsilon())) {
    u = (double)I.getWidth() / (2 * px_int);
    v = (double)I.getHeight() / (2 * py_int);
  } else {
    u = (double)I_.getWidth() / (vpMath::minimum(I_.getWidth(), I_.getHeight()));
    v = (double)I_.getHeight() / (vpMath::minimum(I_.getWidth(), I_.getHeight()));
  }

  float o44c[4][4], o44cd[4][4], x, y, z;
  Matrix id = IDENTITY_MATRIX;

  vpHomogeneousMatrix *fMit = new vpHomogeneousMatrix[size_fMi];
  get_fMi(fMit);
  this->cMo = fMit[size_fMi - 1].inverse() * fMo;
  this->cMo = rotz * cMo;

  vp2jlc_matrix(cMo.inverse(), o44c);
  vp2jlc_matrix(cdMo.inverse(), o44cd);

  while (get_displayBusy())
    vpTime::wait(2);

  add_vwstack("start", "cop", o44c[3][0], o44c[3][1], o44c[3][2]);
  x = o44c[2][0] + o44c[3][0];
  y = o44c[2][1] + o44c[3][1];
  z = o44c[2][2] + o44c[3][2];
  add_vwstack("start", "vrp", x, y, z);
  add_vwstack("start", "vpn", o44c[2][0], o44c[2][1], o44c[2][2]);
  add_vwstack("start", "vup", o44c[1][0], o44c[1][1], o44c[1][2]);
  add_vwstack("start", "window", -u, u, -v, v);
  if (displayObject) {
    display_scene(id, this->scene, I_, curColor);
  }

  add_vwstack("start", "cop", o44cd[3][0], o44cd[3][1], o44cd[3][2]);
  x = o44cd[2][0] + o44cd[3][0];
  y = o44cd[2][1] + o44cd[3][1];
  z = o44cd[2][2] + o44cd[3][2];
  add_vwstack("start", "vrp", x, y, z);
  add_vwstack("start", "vpn", o44cd[2][0], o44cd[2][1], o44cd[2][2]);
  add_vwstack("start", "vup", o44cd[1][0], o44cd[1][1], o44cd[1][2]);
  add_vwstack("start", "window", -u, u, -v, v);
  if (displayDesiredObject) {
    if (desiredObject == D_TOOL)
      display_scene(o44cd, desiredScene, I_, vpColor::red);
    else
      display_scene(id, desiredScene, I_, desColor);
  }
  delete[] fMit;
  set_displayBusy(false);
}

/*!
  Get the pose between the object and the robot's camera.

  \return The pose between the object and the fixed world frame.
*/
vpHomogeneousMatrix vpRobotWireFrameSimulator::get_cMo()
{
  vpHomogeneousMatrix cMoTemp;
  vpHomogeneousMatrix *fMit = new vpHomogeneousMatrix[size_fMi];
  get_fMi(fMit);
  cMoTemp = fMit[size_fMi - 1].inverse() * fMo;
  delete[] fMit;
  return cMoTemp;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning:
// libvisp_robot.a(vpRobotWireFrameSimulator.cpp.o) has no symbols
void dummy_vpRobotWireFrameSimulator(){};
#endif
