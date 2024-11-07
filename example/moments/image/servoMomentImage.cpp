/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 */

/*!
  \example servoMomentImage.cpp
  Example of moment-based visual servoing with Images
*/

#define PRINT_CONDITION_NUMBER

#include <iostream>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpConfig.h>
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

#if !defined(VISP_HAVE_DISPLAY)
int main()
{
  std::cout << "Can't run this example since no display capability is available." << std::endl;
  std::cout << "You should install one of the following third-party library: X11, OpenCV, GDI, GTK." << std::endl;
  return EXIT_SUCCESS;
}
#elif !defined(VISP_HAVE_THREADS)
int main()
{
  std::cout << "Can't run this example since multi-threading capability is not available." << std::endl;
  std::cout << "You should maybe enable cxx11 standard." << std::endl;
  return EXIT_SUCCESS;
}
#else

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

#ifndef DOXYGEN_SHOULD_SKIP_THIS
class servoMoment
{
public:
  servoMoment()
    : m_width(640), m_height(480), m_cMo(), m_cdMo(), m_robot(), m_Iint(m_height, m_width, vpRGBa(0)), m_task(), m_cam(),
    m_error(0), m_imsim(), m_cur_img(m_height, m_width, 0), m_src_img(m_height, m_width, 0),
    m_dst_img(m_height, m_width, 0), m_start_img(m_height, m_width, vpRGBa(0)), m_interaction_type(), m_src(6), m_dst(6),
    m_moments(nullptr), m_momentsDes(nullptr), m_featureMoments(nullptr), m_featureMomentsDes(nullptr), m_displayInt(nullptr)
  { }
  ~servoMoment()
  {
#ifdef VISP_HAVE_DISPLAY
    if (m_displayInt) {
      delete m_displayInt;
    }
#endif
    delete m_moments;
    delete m_momentsDes;
    delete m_featureMoments;
    delete m_featureMomentsDes;
  }

  // setup robot parameters
  void paramRobot() { m_cam = vpCameraParameters(600, 600, m_width / 2., m_height / 2.); }

  // update moment objects and interface
  void refreshScene(vpMomentObject &obj)
  {
    m_cur_img = 0;
    m_imsim.setCameraPosition(m_cMo);
    m_imsim.getImage(m_cur_img, m_cam);
    obj.fromImage(m_cur_img, 128, m_cam);
  }

  // initialize scene in the interface
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
    vpImage<unsigned char> tmp_img(m_height, m_width, 255);
    vpImage<vpRGBa> tmp_start_img(m_height, m_width, vpRGBa(255, 0, 0));

    vpImageSimulator imsim_start;
    imsim_start.setInterpolationType(vpImageSimulator::BILINEAR_INTERPOLATION);
    imsim_start.init(tmp_start_img, X);
    imsim_start.setCameraPosition(m_cdMo);
    imsim_start.getImage(m_start_img, m_cam);

    m_imsim.setInterpolationType(vpImageSimulator::BILINEAR_INTERPOLATION);
    m_imsim.init(tmp_img, X);

    m_imsim.setCameraPosition(m_cMo);
    m_imsim.getImage(m_src_img, m_cam);

    m_src.setType(vpMomentObject::DENSE_FULL_OBJECT);
    m_src.fromImage(m_src_img, 128, m_cam);

    m_dst.setType(vpMomentObject::DENSE_FULL_OBJECT);
    m_imsim.setCameraPosition(m_cdMo);
    m_imsim.getImage(m_dst_img, m_cam);
    m_dst.fromImage(m_dst_img, 128, m_cam);
  }

  // initialize the moment features
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
    pl.changeFrame(m_cMo);
    planeToABC(pl, A, B, C);

    pl.setABCD(0, 0, 1.0, 0);
    pl.changeFrame(m_cdMo);
    planeToABC(pl, Ad, Bd, Cd);

    // extracting initial position (actually we only care about Zdst)
    vpTranslationVector vec;
    m_cdMo.extract(vec);

    ///////////////////////////// initializing moments and features
    ////////////////////////////////////
    // don't need to be specific, vpMomentCommon automatically loads
    // Xg,Yg,An,Ci,Cj,Alpha moments
    m_moments = new vpMomentCommon(vpMomentCommon::getSurface(m_dst), vpMomentCommon::getMu3(m_dst),
      vpMomentCommon::getAlpha(m_dst), vec[2], true);
    m_momentsDes = new vpMomentCommon(vpMomentCommon::getSurface(m_dst), vpMomentCommon::getMu3(m_dst),
      vpMomentCommon::getAlpha(m_dst), vec[2], true);
    // same thing with common features
    m_featureMoments = new vpFeatureMomentCommon(*m_moments);
    m_featureMomentsDes = new vpFeatureMomentCommon(*m_momentsDes);

    m_moments->updateAll(m_src);
    m_momentsDes->updateAll(m_dst);

    m_featureMoments->updateAll(A, B, C);
    m_featureMomentsDes->updateAll(Ad, Bd, Cd);

    // setup the interaction type
    m_task.setInteractionMatrixType(m_interaction_type);
    //////////////////////////////////add useful features to
    /// task//////////////////////////////
    m_task.addFeature(m_featureMoments->getFeatureGravityNormalized(),
      m_featureMomentsDes->getFeatureGravityNormalized());
    m_task.addFeature(m_featureMoments->getFeatureAn(), m_featureMomentsDes->getFeatureAn());
    // the moments are different in case of a symmetric object
    m_task.addFeature(m_featureMoments->getFeatureCInvariant(), m_featureMomentsDes->getFeatureCInvariant(),
      (1 << 10) | (1 << 11));
    m_task.addFeature(m_featureMoments->getFeatureAlpha(), m_featureMomentsDes->getFeatureAlpha());

    m_task.setLambda(1.);
  }

  void init(vpHomogeneousMatrix &cMo, vpHomogeneousMatrix &cdMo)
  {
    m_cMo = cMo;   // init source matrix
    m_cdMo = cdMo; // init destination matrix

    m_interaction_type = vpServo::CURRENT; // use interaction matrix for current position

#ifdef VISP_HAVE_DISPLAY
    // init the right display
#if defined(VISP_HAVE_X11)
    m_displayInt = new vpDisplayX;
#elif defined(HAVE_OPENCV_HIGHGUI)
    m_displayInt = new vpDisplayOpenCV;
#elif defined(VISP_HAVE_GDI)
    m_displayInt = new vpDisplayGDI;
#elif defined(VISP_HAVE_D3D9)
    m_displayInt = new vpDisplayD3D;
#elif defined(VISP_HAVE_GTK)
    m_displayInt = new vpDisplayGTK;
#endif
    m_displayInt->init(m_Iint, 50, 50, "Visual servoing with moments");
#endif

    paramRobot(); // set up robot parameters

    m_task.setServo(vpServo::EYEINHAND_CAMERA);
    initScene();    // initialize graphical scene (for interface)
    initFeatures(); // initialize moment features
  }

  // launch the simulation
  void execute(unsigned int nbIter)
  {
    vpPlot ViSP_plot;
    init_visp_plot(ViSP_plot); // Initialize plot object

    // init main object: using moments up to order 6
    vpMomentObject obj(6);
    // setting object type (disrete, continuous[form polygon])
    obj.setType(vpMomentObject::DENSE_FULL_OBJECT);

    std::cout << "Display task information " << std::endl;
    m_task.print();

    vpDisplay::display(m_Iint);
    vpDisplay::flush(m_Iint);
    unsigned int iter = 0;

    vpHomogeneousMatrix wMo; // Set to identity
    vpHomogeneousMatrix wMc; // Camera position in the world frame
    wMc = wMo * m_cMo.inverse();
    m_robot.setPosition(wMc);
    double sampling_time = 0.010; // Sampling period in seconds
    m_robot.setSamplingTime(sampling_time);

    ///////////////////SIMULATION LOOP/////////////////////////////
    while (iter++ < nbIter) {

      vpColVector v;
      double t = vpTime::measureTimeMs();
      // get the cMo
      wMc = m_robot.getPosition();
      m_cMo = wMc.inverse() * wMo;
      // setup the plane in A,B,C style
      vpPlane pl;
      double A, B, C;
      pl.setABCD(0, 0, 1.0, 0);
      pl.changeFrame(m_cMo);
      planeToABC(pl, A, B, C);

      // track points, draw points and add refresh our object
      refreshScene(obj);
      // this is the most important thing to do: update our moments
      m_moments->updateAll(obj);
      // and update our features. Do it in that order. Features need to use the
      // information computed by moments
      m_featureMoments->updateAll(A, B, C);
      // some graphics again
      m_imsim.setCameraPosition(m_cMo);

      m_Iint = m_start_img;

      m_imsim.getImage(m_Iint, m_cam);
      vpDisplay::display(m_Iint);

      if (iter == 1) {
        vpDisplay::displayText(m_Iint, 20, 20, "Click to start servoing", vpColor::red);
        vpDisplay::flush(m_Iint);
        vpDisplay::getClick(m_Iint);
      }
      v = m_task.computeControlLaw();

      std::cout << " || s - s* || = " << m_task.error.sumSquare() << std::endl;

      m_robot.setVelocity(vpRobot::CAMERA_FRAME, v);

      ViSP_plot.plot(0, iter, v);
      ViSP_plot.plot(1, iter, vpPoseVector(m_cMo)); // Plot the velocities
      ViSP_plot.plot(2, iter, m_task.getError());   // cMo as translations and theta_u

      m_error = (m_task.getError()).sumSquare();

#if defined(PRINT_CONDITION_NUMBER)
      /*
       * Condition number of interaction matrix
       */
      vpMatrix Linteraction = m_task.L;
      vpMatrix tmpry, U;
      vpColVector singularvals;
      Linteraction.svd(singularvals, tmpry);
      double condno = static_cast<double>(singularvals.getMaxValue() / singularvals.getMinValue());
      std::cout << "Condition Number: " << condno << std::endl;
#endif
      vpDisplay::displayText(m_Iint, 20, 20, "Click to stop visual servo...", vpColor::red);
      if (vpDisplay::getClick(m_Iint, false)) {
        break;
      }
      vpDisplay::flush(m_Iint);
      vpTime::wait(t, sampling_time * 1000); // Wait 10 ms
    }

    m_imsim.getImage(m_Iint, m_cam);
    vpDisplay::display(m_Iint);
    vpDisplay::displayText(m_Iint, 20, 20, "Click to quit...", vpColor::red);
    vpDisplay::flush(m_Iint);
    vpDisplay::getClick(m_Iint);
  }

  void setInteractionMatrixType(vpServo::vpServoIteractionMatrixType type) { m_interaction_type = type; }

  double error() { return m_error; }

  void planeToABC(vpPlane &pl, double &A, double &B, double &C)
  {
    if (fabs(pl.getD()) < std::numeric_limits<double>::epsilon()) {
      std::cout << "Invalid position:" << std::endl;
      std::cout << m_cMo << std::endl;
      std::cout << "Cannot put plane in the form 1/Z=Ax+By+C." << std::endl;
      throw vpException(vpException::divideByZeroError, "invalid position!");
    }
    A = -pl.getA() / pl.getD();
    B = -pl.getB() / pl.getD();
    C = -pl.getC() / pl.getD();
  }

  void init_visp_plot(vpPlot &ViSP_plot)
  {
    /* -------------------------------------
     * Initialize ViSP Plotting
     * -------------------------------------
     */
    const unsigned int NbGraphs = 3;                            // No. of graphs
    const unsigned int NbCurves_in_graph[NbGraphs] = { 6, 6, 6 }; // Curves in each graph

    ViSP_plot.init(NbGraphs, 800, 800, 100 + static_cast<int>(m_width), 50, "Visual Servoing results...");

    vpColor Colors[6] = {// Colour for s1, s2, s3,  in 1st plot
                         vpColor::red, vpColor::green, vpColor::blue, vpColor::orange, vpColor::cyan, vpColor::purple };

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

protected:
  // start and destination positioning matrices
  unsigned int m_width;
  unsigned int m_height;

  // start and destination positioning matrices
  vpHomogeneousMatrix m_cMo;
  vpHomogeneousMatrix m_cdMo;

  vpSimulatorCamera m_robot; // robot used in this simulation
  vpImage<vpRGBa> m_Iint;    // internal image used for interface display
  vpServo m_task;            // servoing task
  vpCameraParameters m_cam;  // robot camera parameters
  double m_error;            // current error
  vpImageSimulator m_imsim;  // image simulator used to simulate the perspective-projection camera

  // several images used in the simulation
  vpImage<unsigned char> m_cur_img;
  vpImage<unsigned char> m_src_img;
  vpImage<unsigned char> m_dst_img;
  vpImage<vpRGBa> m_start_img;
  vpServo::vpServoIteractionMatrixType m_interaction_type; // current or desired
  // source and destination objects for moment manipulation
  vpMomentObject m_src;
  vpMomentObject m_dst;

  // moment sets and their corresponding features
  vpMomentCommon *m_moments;
  vpMomentCommon *m_momentsDes;
  vpFeatureMomentCommon *m_featureMoments;
  vpFeatureMomentCommon *m_featureMomentsDes;

  vpDisplay *m_displayInt;
};
#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS

int main()
{
  try {
    // intial pose
    vpHomogeneousMatrix cMo(-0.1, -0.1, 1.5, -vpMath::rad(20), -vpMath::rad(20), -vpMath::rad(30));
    // Desired pose
    vpHomogeneousMatrix cdMo(vpHomogeneousMatrix(0.0, -0.0, 1.0, vpMath::rad(0), vpMath::rad(0), -vpMath::rad(0)));

    servoMoment servo;
    // init the simulation
    servo.init(cMo, cdMo);

    servo.execute(1500);
    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}

#endif
