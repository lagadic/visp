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
 * Simulation of a visual servoing with visualization.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file manSimu4Points.cpp
  \brief Visual servoing experiment on 4 points with a visualization
  from the camera and from an external view using vpSimulator.
*/

/*!
  \example manSimu4Points.cpp
  Visual servoing experiment on 4 points with a visualization
  from the camera and from an external view using vpSimulator.
*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>

#if (defined(VISP_HAVE_COIN3D_AND_GUI))

#include <visp3/ar/vpSimulator.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpTime.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>

static void *mainLoop(void *_simu)
{
  // pointer copy of the vpSimulator instance
  vpSimulator *simu = static_cast<vpSimulator *>(_simu);

  // Simulation initialization
  simu->initMainApplication();

  /////////////////////////////////////////
  // sets the initial camera location
  vpHomogeneousMatrix cMo(-0.3, -0.2, 3, vpMath::rad(0), vpMath::rad(0), vpMath::rad(40));
  vpHomogeneousMatrix wMo; // Set to identity
  vpHomogeneousMatrix wMc; // Camera position in the world frame

  ///////////////////////////////////
  // Initialize the robot
  vpSimulatorCamera robot;
  robot.setSamplingTime(0.04); // 40ms
  wMc = wMo * cMo.inverse();
  robot.setPosition(wMc);
  // Send the robot position to the visualizator
  simu->setCameraPosition(cMo);
  // Initialize the camera parameters
  vpCameraParameters cam;
  simu->getCameraParameters(cam);

  ////////////////////////////////////////
  // Desired visual features initialization

  // sets the points coordinates in the object frame (in meter)
  vpPoint point[4];
  point[0].setWorldCoordinates(-0.1, -0.1, 0);
  point[1].setWorldCoordinates(0.1, -0.1, 0);
  point[2].setWorldCoordinates(0.1, 0.1, 0);
  point[3].setWorldCoordinates(-0.1, 0.1, 0);

  // sets the desired camera location
  vpHomogeneousMatrix cMo_d(0, 0, 1, 0, 0, 0);

  // computes the 3D point coordinates in the camera frame and its 2D
  // coordinates
  for (int i = 0; i < 4; i++)
    point[i].project(cMo_d);

  // creates the associated features
  vpFeaturePoint pd[4];
  for (int i = 0; i < 4; i++)
    vpFeatureBuilder::create(pd[i], point[i]);

  ///////////////////////////////////////
  // Current visual features initialization

  // computes the 3D point coordinates in the camera frame and its 2D
  // coordinates
  for (int i = 0; i < 4; i++)
    point[i].project(cMo);

  // creates the associated features
  vpFeaturePoint p[4];
  for (int i = 0; i < 4; i++)
    vpFeatureBuilder::create(p[i], point[i]);

  /////////////////////////////////
  // Task defintion
  vpServo task;
  // we want an eye-in-hand control law ;
  task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
  task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);

  // Set the position of the camera in the end-effector frame
  vpHomogeneousMatrix cMe;
  vpVelocityTwistMatrix cVe(cMe);
  task.set_cVe(cVe);
  // Set the Jacobian (expressed in the end-effector frame)
  vpMatrix eJe;
  robot.get_eJe(eJe);
  task.set_eJe(eJe);

  // we want to see a point on a point
  for (int i = 0; i < 4; i++)
    task.addFeature(p[i], pd[i]);
  // Set the gain
  task.setLambda(1.0);
  // Print the current information about the task
  task.print();

  vpTime::wait(500);
  ////////////////////////////////////////////////
  // The control loop
  int k = 0;
  while (k++ < 200) {
    double t = vpTime::measureTimeMs();

    // Update the current features
    for (int i = 0; i < 4; i++) {
      point[i].project(cMo);
      vpFeatureBuilder::create(p[i], point[i]);
    }

    // Update the robot Jacobian
    robot.get_eJe(eJe);
    task.set_eJe(eJe);

    // Compute the control law
    vpColVector v = task.computeControlLaw();

    // Send the computed velocity to the robot and compute the new robot
    // position
    robot.setVelocity(vpRobot::ARTICULAR_FRAME, v);
    wMc = robot.getPosition();
    cMo = wMc.inverse() * wMo;

    // Send the robot position to the visualizator
    simu->setCameraPosition(cMo);

    // Print the current information about the task
    task.print();

    // Wait 40 ms
    vpTime::wait(t, 40);
  }
  task.kill();
  simu->closeMainApplication();

  void *a = NULL;
  return a;
  // return (void *);
}

int main()
{
  try {
    vpSimulator simu;

    // Internal view initialization : view from the robot camera
    simu.initInternalViewer(480, 360);
    // External view initialization : view from an external camera
    simu.initExternalViewer(300, 300);

    // Inernal camera paramters initialization
    vpCameraParameters cam(800, 800, 240, 180);
    simu.setInternalCameraParameters(cam);

    vpTime::wait(1000);
    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    std::string ipath = vpIoTools::getViSPImagesDataPath();
    std::string filename = "./4points.iv";

    // Set the default input path
    if (!ipath.empty())
      filename = vpIoTools::createFilePath(ipath, "iv/4points.iv");

    std::cout << "Load : " << filename << std::endl << "This file should be in the working directory" << std::endl;

    simu.load(filename.c_str());

    // Run the main loop
    simu.initApplication(&mainLoop);
    // Run the simulator
    simu.mainLoop();
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}

#else
int main()
{
  std::cout << "You do not have Coin3D and SoQT or SoWin or SoXt functionalities enabled..." << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- Install Coin3D and SoQT or SoWin or SoXt, configure ViSP again using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
}
#endif
