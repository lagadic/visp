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
 * Example of visual servoing with moments using an image as object
 * container
 *
 * Authors:
 * Filip Novotny
 * Manikandan.B
 *****************************************************************************/

/*!
  \example servoMomentImage.cpp
  Example of moment-based visual servoing with Images
*/

#define PRINT_CONDITION_NUMBER

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
#include <visp3/core/vpPoseVector.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/robot/vpImageSimulator.h>
#include <visp3/robot/vpSimulatorCamera.h>
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

void init_visp_plot(vpPlot &);

int main()
{
  try {
    // intial pose
    vpHomogeneousMatrix cMo(-0.1, -0.1, 1.5, -vpMath::rad(20), -vpMath::rad(20), -vpMath::rad(30));
    // Desired pose
    vpHomogeneousMatrix cdMo(vpHomogeneousMatrix(0.0, -0.0, 1.0, vpMath::rad(0), vpMath::rad(0), -vpMath::rad(0)));

    // init the simulation
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

vpSimulatorCamera robot;           // robot used in this simulation
vpImage<vpRGBa> Iint(480, 640, 0); // internal image used for interface
                                   // display
vpServo task;                      // servoing task
vpCameraParameters cam;            // robot camera parameters
double _error;                     // current error
vpImageSimulator imsim;            // image simulator used to simulate the
                                   // perspective-projection camera

// several images used in the simulation
vpImage<unsigned char> cur_img(480, 640, 0);
vpImage<unsigned char> src_img(480, 640, 0);
vpImage<unsigned char> dst_img(480, 640, 0);
vpImage<vpRGBa> start_img(480, 640, 0);
vpServo::vpServoIteractionMatrixType interaction_type; // current or desired
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
  vpColVector X[4];
  for (int i = 0; i < 4; i++)
    X[i].resize(3);
  X[0][0] = -0.2;
  X[0][1] = -0.1;
  X[0][2] = 0;

  X[1][0] = 0.2;
  X[1][1] = -0.1;
  X[1][2] = 0;

  X[2][0] = 0.2;
  X[2][1] = 0.1;
  X[2][2] = 0;

  X[3][0] = -0.2;
  X[3][1] = 0.1;
  X[3][2] = 0;
  // init source and destination images
  vpImage<unsigned char> tmp_img(480, 640, 255);
  vpImage<vpRGBa> tmp_start_img(480, 640, vpRGBa(255, 0, 0));

  vpImageSimulator imsim_start;
  imsim_start.setInterpolationType(vpImageSimulator::BILINEAR_INTERPOLATION);
  imsim_start.init(tmp_start_img, X);
  imsim_start.setCameraPosition(cdMo);
  imsim_start.getImage(start_img, cam);

  imsim.setInterpolationType(vpImageSimulator::BILINEAR_INTERPOLATION);
  imsim.init(tmp_img, X);

  imsim.setCameraPosition(cMo);
  imsim.getImage(src_img, cam);

  src.setType(vpMomentObject::DENSE_FULL_OBJECT);
  src.fromImage(src_img, 128, cam);

  dst.setType(vpMomentObject::DENSE_FULL_OBJECT);
  imsim.setCameraPosition(cdMo);
  imsim.getImage(dst_img, cam);
  dst.fromImage(dst_img, 128, cam);
}

void refreshScene(vpMomentObject &obj)
{
  cur_img = 0;
  imsim.setCameraPosition(cMo);
  imsim.getImage(cur_img, cam);
  obj.fromImage(cur_img, 128, cam);
}

void init(vpHomogeneousMatrix &_cMo, vpHomogeneousMatrix &_cdMo)
{
  cMo = _cMo;   // init source matrix
  cdMo = _cdMo; // init destination matrix

  interaction_type = vpServo::CURRENT; // use interaction matrix for current position

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
  // init main object: using moments up to order 5

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
                               vpMomentCommon::getAlpha(dst), vec[2], true);
  momentsDes = new vpMomentCommon(vpMomentCommon::getSurface(dst), vpMomentCommon::getMu3(dst),
                                  vpMomentCommon::getAlpha(dst), vec[2], true);
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

  task.setLambda(1.);
}

void execute(unsigned int nbIter)
{

  vpPlot ViSP_plot;
  init_visp_plot(ViSP_plot); // Initialize plot object

  // init main object: using moments up to order 6
  vpMomentObject obj(6);
  // setting object type (disrete, continuous[form polygon])
  obj.setType(vpMomentObject::DENSE_FULL_OBJECT);

  vpTRACE("Display task information ");
  task.print();

  vpDisplay::display(Iint);
  vpDisplay::flush(Iint);
  unsigned int iter = 0;

  vpHomogeneousMatrix wMo; // Set to identity
  vpHomogeneousMatrix wMc; // Camera position in the world frame
  wMc = wMo * cMo.inverse();
  robot.setPosition(wMc);
  float sampling_time = 0.010f; // Sampling period in seconds
  robot.setSamplingTime(sampling_time);

  // For plotting
  vpPoseVector currentpose;
  vpColVector err_features;

  ///////////////////SIMULATION LOOP/////////////////////////////
  while (iter++ < nbIter) {

    vpColVector v;
    double t = vpTime::measureTimeMs();
    // get the cMo
    wMc = robot.getPosition();
    cMo = wMc.inverse() * wMo;
    currentpose.buildFrom(cMo); // For plot
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
    // some graphics again
    imsim.setCameraPosition(cMo);

    Iint = start_img;

    imsim.getImage(Iint, cam);
    vpDisplay::display(Iint);

    vpDisplay::flush(Iint);

    if (iter == 1)
      vpDisplay::getClick(Iint);
    v = task.computeControlLaw();
    // pilot robot using position control. The displacement is t*v with t=10ms
    // step  robot.setPosition(vpRobot::CAMERA_FRAME,0.01*v);

    err_features = task.error;
    std::cout << " || s - s* || = " << task.error.sumSquare() << std::endl;

    robot.setVelocity(vpRobot::CAMERA_FRAME, v);
    vpTime::wait(t, sampling_time * 1000); // Wait 10 ms

    ViSP_plot.plot(0, iter, v);
    ViSP_plot.plot(1, iter, currentpose);  // Plot the velocities
    ViSP_plot.plot(2, iter, err_features); // cMo as translations and theta_u

    _error = (task.getError()).sumSquare();

#if defined(PRINT_CONDITION_NUMBER)
    /*
     * Condition number of interaction matrix
     */
    vpMatrix Linteraction = task.L;
    vpMatrix tmpry, U;
    vpColVector singularvals;
    Linteraction.svd(singularvals, tmpry);
    double condno = static_cast<double>(singularvals.getMaxValue() / singularvals.getMinValue());
    std::cout << "Condition Number: " << condno << std::endl;
#endif
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

void paramRobot() { cam = vpCameraParameters(640, 480, 320, 240); }

void init_visp_plot(vpPlot &ViSP_plot)
{
  /* -------------------------------------
   * Initialize ViSP Plotting
   * -------------------------------------
   */
  const unsigned int NbGraphs = 3;                            // No. of graphs
  const unsigned int NbCurves_in_graph[NbGraphs] = {6, 6, 6}; // Curves in each graph

  ViSP_plot.init(NbGraphs, 800, 800, 10, 10, "Visual Servoing results...");

  vpColor Colors[6] = {// Colour for s1, s2, s3,  in 1st plot
                       vpColor::red, vpColor::green, vpColor::blue, vpColor::orange, vpColor::cyan, vpColor::purple};

  for (unsigned int p = 0; p < NbGraphs; p++) {
    ViSP_plot.initGraph(p, NbCurves_in_graph[p]);
    for (unsigned int c = 0; c < NbCurves_in_graph[p]; c++)
      ViSP_plot.setColor(p, c, Colors[c]);
  }

  ViSP_plot.setTitle(0, "Robot velocities");
  ViSP_plot.setLegend(0, 0, "v_x");
  ViSP_plot.setLegend(0, 1, "v_y");
  ViSP_plot.setLegend(0, 2, "v_z");
  ViSP_plot.setLegend(0, 3, "w_x");
  ViSP_plot.setLegend(0, 4, "w_y");
  ViSP_plot.setLegend(0, 5, "w_z");

  ViSP_plot.setTitle(1, "Camera pose cMo");
  ViSP_plot.setLegend(1, 0, "tx");
  ViSP_plot.setLegend(1, 1, "ty");
  ViSP_plot.setLegend(1, 2, "tz");
  ViSP_plot.setLegend(1, 3, "tu_x");
  ViSP_plot.setLegend(1, 4, "tu_y");
  ViSP_plot.setLegend(1, 5, "tu_z");

  ViSP_plot.setTitle(2, "Error in visual features: ");
  ViSP_plot.setLegend(2, 0, "x_n");
  ViSP_plot.setLegend(2, 1, "y_n");
  ViSP_plot.setLegend(2, 2, "a_n");
  ViSP_plot.setLegend(2, 3, "sx");
  ViSP_plot.setLegend(2, 4, "sy");
  ViSP_plot.setLegend(2, 5, "alpha");
}
#endif
