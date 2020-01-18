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
 * Example of visual servoing with moments using a polygon as object container
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

/*!
  \example servoMomentPolygon.cpp
  Example of moment-based visual servoing with Images
*/

#include <iostream>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMomentCommon.h>
#include <visp3/core/vpMomentDatabase.h>
#include <visp3/core/vpMomentObject.h>
#include <visp3/core/vpPlane.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/robot/vpSimulatorAfma6.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureMomentCommon.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>

#if !defined(_WIN32) && !defined(VISP_HAVE_PTHREAD)
// Robot simulator used in this example is not available
int main()
{
  std::cout << "Can't run this example since vpSimulatorAfma6 capability is "
               "not available."
            << std::endl;
  std::cout << "You should install pthread third-party library." << std::endl;
  return EXIT_SUCCESS;
}
// No display available
#elif !defined(VISP_HAVE_X11) && !defined(VISP_HAVE_OPENCV) && !defined(VISP_HAVE_GDI) && !defined(VISP_HAVE_D3D9) &&  \
    !defined(VISP_HAVE_GTK)
int main()
{
  std::cout << "Can't run this example since no display capability is available." << std::endl;
  std::cout << "You should install one of the following third-party library: "
               "X11, OpenCV, GDI, GTK."
            << std::endl;
  return EXIT_SUCCESS;
}
#else

// setup robot parameters
void paramRobot();

// update moment objects and interface
void refreshScene(vpMomentObject &obj);
// initialize scene in the interface
void initScene();
// initialize the moment features
void initFeatures();

void init(vpHomogeneousMatrix &cMo, vpHomogeneousMatrix &cdMo);
void execute(unsigned int nbIter); // launch the simulation
void setInteractionMatrixType(vpServo::vpServoIteractionMatrixType type);
double error();
void planeToABC(vpPlane &pl, double &A, double &B, double &C);
void paramRobot();
void removeJointLimits(vpSimulatorAfma6 &robot);

int main()
{
  try { // intial pose
    vpHomogeneousMatrix cMo(-0.1, -0.1, 1.5, -vpMath::rad(20), -vpMath::rad(20), -vpMath::rad(30));
    // Desired pose
    vpHomogeneousMatrix cdMo(vpHomogeneousMatrix(0.0, 0.0, 1.0, vpMath::rad(0), vpMath::rad(0), -vpMath::rad(0)));

    // init and run the simulation
    init(cMo, cdMo);
    execute(1500);
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}

// init the right display
#if defined VISP_HAVE_X11
vpDisplayX displayInt;
#elif defined VISP_HAVE_OPENCV
vpDisplayOpenCV displayInt;
#elif defined VISP_HAVE_GDI
vpDisplayGDI displayInt;
#elif defined VISP_HAVE_D3D9
vpDisplayD3D displayInt;
#elif defined VISP_HAVE_GTK
vpDisplayGTK displayInt;
#endif

// start and destination positioning matrices
vpHomogeneousMatrix cMo;
vpHomogeneousMatrix cdMo;

vpSimulatorAfma6 robot(false); // robot used in this simulation
vpImage<vpRGBa> Iint(480, 640,
                     255);                             // internal image used for interface display
vpServo task;                                          // servoing task
vpCameraParameters cam;                                // robot camera parameters
double _error;                                         // current error
vpServo::vpServoIteractionMatrixType interaction_type; // current or desired
vpImageSimulator imsim;                                // image simulator used to simulate the
                                                       // perspective-projection camera

// source and destination objects for moment manipulation
vpMomentObject src(6);
vpMomentObject dst(6);
// moment sets and their corresponding features
vpMomentCommon *moments;
vpMomentCommon *momentsDes;
vpFeatureMomentCommon *featureMoments;
vpFeatureMomentCommon *featureMomentsDes;

using namespace std;

void initScene()
{
  vector<vpPoint> src_pts;
  vector<vpPoint> dst_pts;

  double x[5] = {0.2, 0.2, -0.2, -0.2, 0.2};
  double y[5] = {-0.1, 0.1, 0.1, -0.1, -0.1};
  int nbpoints = 4;

  for (int i = 0; i < nbpoints; i++) {
    vpPoint p(x[i], y[i], 0.0);
    p.track(cMo);
    src_pts.push_back(p);
  }

  src.setType(vpMomentObject::DENSE_POLYGON);
  src.fromVector(src_pts);
  for (int i = 0; i < nbpoints; i++) {
    vpPoint p(x[i], y[i], 0.0);
    p.track(cdMo);
    dst_pts.push_back(p);
  }
  dst.setType(vpMomentObject::DENSE_POLYGON);
  dst.fromVector(dst_pts);
}

void refreshScene(vpMomentObject &obj)
{
  double x[5] = {0.2, 0.2, -0.2, -0.2, 0.2};
  double y[5] = {-0.1, 0.1, 0.1, -0.1, -0.1};
  int nbpoints = 5;
  vector<vpPoint> cur_pts;

  for (int i = 0; i < nbpoints; i++) {
    vpPoint p(x[i], y[i], 0.0);
    p.track(cMo);
    cur_pts.push_back(p);
  }
  obj.fromVector(cur_pts);
}

void init(vpHomogeneousMatrix &_cMo, vpHomogeneousMatrix &_cdMo)
{
  cMo = _cMo;
  cdMo = _cdMo;
  interaction_type = vpServo::CURRENT;
  displayInt.init(Iint, 700, 0, "Visual servoing with moments");

  paramRobot(); // set up robot parameters

  task.setServo(vpServo::EYEINHAND_CAMERA);
  initScene();    // initialize graphical scene (for interface)
  initFeatures(); // initialize moment features
}

void initFeatures()
{
  // A,B,C parameters of source and destination plane
  double A;
  double B;
  double C;
  double Ad;
  double Bd;
  double Cd;
  // init main object: using moments up to order 6

  // Initializing values from regular plane (with ax+by+cz=d convention)
  vpPlane pl;
  pl.setABCD(0, 0, 1.0, 0);
  pl.changeFrame(cMo);
  planeToABC(pl, A, B, C);

  pl.setABCD(0, 0, 1.0, 0);
  pl.changeFrame(cdMo);
  planeToABC(pl, Ad, Bd, Cd);

  // extracting initial position (actually we only care about Zdst)
  vpTranslationVector vec;
  cdMo.extract(vec);

  ///////////////////////////// initializing moments and features
  ////////////////////////////////////
  // don't need to be specific, vpMomentCommon automatically loads
  // Xg,Yg,An,Ci,Cj,Alpha moments
  moments = new vpMomentCommon(vpMomentCommon ::getSurface(dst), vpMomentCommon::getMu3(dst),
                               vpMomentCommon::getAlpha(dst), vec[2]);
  momentsDes = new vpMomentCommon(vpMomentCommon::getSurface(dst), vpMomentCommon::getMu3(dst),
                                  vpMomentCommon::getAlpha(dst), vec[2]);
  // same thing with common features
  featureMoments = new vpFeatureMomentCommon(*moments);
  featureMomentsDes = new vpFeatureMomentCommon(*momentsDes);

  moments->updateAll(src);
  momentsDes->updateAll(dst);

  featureMoments->updateAll(A, B, C);
  featureMomentsDes->updateAll(Ad, Bd, Cd);

  // setup the interaction type
  task.setInteractionMatrixType(interaction_type);
  //////////////////////////////////add useful features to
  /// task//////////////////////////////
  task.addFeature(featureMoments->getFeatureGravityNormalized(), featureMomentsDes->getFeatureGravityNormalized());
  task.addFeature(featureMoments->getFeatureAn(), featureMomentsDes->getFeatureAn());
  // the moments are different in case of a symmetric object
  task.addFeature(featureMoments->getFeatureCInvariant(), featureMomentsDes->getFeatureCInvariant(),
                  (1 << 10) | (1 << 11));
  task.addFeature(featureMoments->getFeatureAlpha(), featureMomentsDes->getFeatureAlpha());

  task.setLambda(0.4);
}

void execute(unsigned int nbIter)
{
  // init main object: using moments up to order 5
  vpMomentObject obj(6);
  // setting object type (disrete, continuous[form polygon])
  obj.setType(vpMomentObject::DENSE_POLYGON);

  vpTRACE("Display task information ");
  task.print();

  vpDisplay::display(Iint);
  robot.getInternalView(Iint);
  vpDisplay::flush(Iint);
  unsigned int iter = 0;

  ///////////////////SIMULATION LOOP/////////////////////////////
  while (iter++ < nbIter) {
    vpColVector v;
    double t = vpTime::measureTimeMs();
    // get the cMo
    cMo = robot.get_cMo();
    // setup the plane in A,B,C style
    vpPlane pl;
    double A, B, C;
    pl.setABCD(0, 0, 1.0, 0);
    pl.changeFrame(cMo);
    planeToABC(pl, A, B, C);

    // track points, draw points and add refresh our object
    refreshScene(obj);
    // this is the most important thing to do: update our moments
    moments->updateAll(obj);
    // and update our features. Do it in that order. Features need to use the
    // information computed by moments
    featureMoments->updateAll(A, B, C);

    vpDisplay::display(Iint);
    robot.getInternalView(Iint);
    vpDisplay::flush(Iint);

    if (iter == 1)
      vpDisplay::getClick(Iint);
    v = task.computeControlLaw();

    // pilot robot using position control. The displacement is t*v with t=10ms
    // step
    robot.setPosition(vpRobot::CAMERA_FRAME, 0.01 * v);

    vpTime::wait(t, 10);
    _error = (task.getError()).sumSquare();
  }

  task.kill();

  vpTRACE("\n\nClick in the internal view window to end...");
  vpDisplay::getClick(Iint);

  delete moments;
  delete momentsDes;
  delete featureMoments;
  delete featureMomentsDes;
}

void setInteractionMatrixType(vpServo::vpServoIteractionMatrixType type) { interaction_type = type; }
double error() { return _error; }

void removeJointLimits(vpSimulatorAfma6 &robot_)
{
  vpColVector limMin(6);
  vpColVector limMax(6);
  limMin[0] = vpMath::rad(-3600);
  limMin[1] = vpMath::rad(-3600);
  limMin[2] = vpMath::rad(-3600);
  limMin[3] = vpMath::rad(-3600);
  limMin[4] = vpMath::rad(-3600);
  limMin[5] = vpMath::rad(-3600);

  limMax[0] = vpMath::rad(3600);
  limMax[1] = vpMath::rad(3600);
  limMax[2] = vpMath::rad(3600);
  limMax[3] = vpMath::rad(3600);
  limMax[4] = vpMath::rad(3600);
  limMax[5] = vpMath::rad(3600);

  robot_.setJointLimit(limMin, limMax);
}

void planeToABC(vpPlane &pl, double &A, double &B, double &C)
{
  if (fabs(pl.getD()) < std::numeric_limits<double>::epsilon()) {
    std::cout << "Invalid position:" << std::endl;
    std::cout << cMo << std::endl;
    std::cout << "Cannot put plane in the form 1/Z=Ax+By+C." << std::endl;
    throw vpException(vpException::divideByZeroError, "invalid position!");
  }
  A = -pl.getA() / pl.getD();
  B = -pl.getB() / pl.getD();
  C = -pl.getC() / pl.getD();
}

void paramRobot()
{
  /*Initialise the robot and especially the camera*/
  robot.init(vpAfma6::TOOL_CCMOP, vpCameraParameters::perspectiveProjWithoutDistortion);
  robot.setCurrentViewColor(vpColor(150, 150, 150));
  robot.setDesiredViewColor(vpColor(200, 200, 200));
  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
  removeJointLimits(robot);
  robot.initScene(vpWireFrameSimulator::RECTANGLE, vpWireFrameSimulator::D_STANDARD);
  /*Initialise the position of the object relative to the pose of the robot's
   * camera*/
  robot.initialiseObjectRelativeToCamera(cMo);

  /*Set the desired position (for the displaypart)*/
  robot.setDesiredCameraPosition(cdMo);
  robot.getCameraParameters(cam, Iint);
}

#endif
