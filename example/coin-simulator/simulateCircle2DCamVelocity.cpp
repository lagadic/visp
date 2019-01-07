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
  \file simulateCircle2DCamVelocity.cpp
  \brief Visual servoing experiment on a circle with a visualization
  from the camera and from an external view using vpSimulator.
*/

/*!
  \example simulateCircle2DCamVelocity.cpp
  Visual servoing experiment on a circle with a visualization
  from the camera and from an external view using vpSimulator.
*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>

#ifdef VISP_HAVE_COIN3D_AND_GUI
#include <visp3/ar/vpSimulator.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpCircle.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpTime.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureEllipse.h>
#include <visp3/vs/vpServo.h>

#define GETOPTARGS "cdi:h"
#define SAVE 0

/*!

Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath : Input image path.

*/
void usage(const char *name, const char *badparam, std::string ipath)
{
  fprintf(stdout, "\n\
Simulation Servo Circle\n\
          \n\
SYNOPSIS\n\
  %s [-i <input image path>] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read \"iv/4points.iv\"\n\
     cad model.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
 \n\
  -d                                             \n\
     Disable the image display. This can be useful \n\
     for automatic tests using crontab under Unix or \n\
     using the task manager under Windows.\n\
                  \n\
  -h\n\
     Print the help.\n\n", ipath.c_str());

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!

Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath : Input image path.
  \param display : Set as true, activates the image display. This is
  the default configuration. When set to false, the display is
  disabled. This can be useful for automatic tests using crontab
  under Unix or using the task manager under Windows.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &ipath, bool &display)
{
  const char *optarg;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'i':
      ipath = optarg;
      break;
    case 'd':
      display = false;
      break;
    case 'h':
      usage(argv[0], NULL, ipath);
      return false;
      break;

    default:
      usage(argv[0], optarg, ipath);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}

static void *mainLoop(void *_simu)
{
  vpSimulator *simu = static_cast<vpSimulator *>(_simu);
  simu->initMainApplication();

  vpPoseVector vcMo;

  vcMo[0] = 0.3;
  vcMo[1] = 0.2;
  vcMo[2] = 3;
  vcMo[3] = 0;
  vcMo[4] = vpMath::rad(45);
  vcMo[5] = vpMath::rad(40);
  vpHomogeneousMatrix cMo(vcMo);
  vpHomogeneousMatrix wMo; // Set to identity
  vpHomogeneousMatrix wMc; // Robot (=camera) location in the world frame

  vpHomogeneousMatrix cMod;
  cMod[0][3] = 0;
  cMod[1][3] = 0;
  cMod[2][3] = 1;

  int it = 0;
  unsigned int pos = 2;
  while (pos != 0) {
    vpServo task;
    vpSimulatorCamera robot;

    float sampling_time = 0.040f; // Sampling period in second
    robot.setSamplingTime(sampling_time);
    robot.setMaxTranslationVelocity(4.);

    // Sets the initial camera location
    wMc = wMo * cMo.inverse();
    robot.setPosition(wMc);
    simu->setCameraPosition(cMo);

    if (pos == 1)
      cMod[2][3] = 0.32;

    // Sets the circle coordinates in the world frame
    vpCircle circle;
    circle.setWorldCoordinates(0, 0, 1, 0, 0, 0, 0.1);

    // Sets the desired position of the visual feature
    vpFeatureEllipse pd;
    circle.track(cMod);
    vpFeatureBuilder::create(pd, circle);

    // Project : computes the circle coordinates in the camera frame and its
    // 2D coordinates Sets the current position of the visual feature
    vpFeatureEllipse p;
    circle.track(cMo);
    vpFeatureBuilder::create(p, circle);

    // Define the task
    // We want an eye-in-hand control law
    // Robot is controlled in the camera frame
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);

    // We want to see a circle on a circle
    std::cout << std::endl;
    task.addFeature(p, pd);

    // Set the gain
    task.setLambda(1.0);

    // Display task information
    task.print();

    vpTime::wait(1000); // Sleep 1s

    unsigned int iter = 0;
    // Visual servoing loop
    unsigned int itermax;
    if (pos == 2)
      itermax = 75;
    else
      itermax = 100;
    while (iter++ < itermax) {
      double t = vpTime::measureTimeMs();

      if (iter == 1)
        std::cout << "get the robot position" << std::endl;
      wMc = robot.getPosition();
      if (iter == 1)
        std::cout << "new circle position" << std::endl;
      // retrieve x,y and Z of the vpCircle structure

      cMo = wMc.inverse() * wMo;
      circle.track(cMo);
      vpFeatureBuilder::create(p, circle);

      if (iter == 1)
        std::cout << "compute the control law" << std::endl;
      vpColVector v = task.computeControlLaw();
      if (iter == 1) {
        std::cout << "Task rank: " << task.getTaskRank() << std::endl;
        std::cout << "send the camera velocity to the controller" << std::endl;
      }
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

      simu->setCameraPosition(cMo);

      if (SAVE == 1) {
        char name[FILENAME_MAX];
        sprintf(name, "/tmp/image.%04d.external.png", it);
        std::cout << "Save " << name << std::endl;
        simu->write(name);
        sprintf(name, "/tmp/image.%04u.internal.png", iter);
        std::cout << "Save " << name << std::endl;
        simu->write(name);
        it++;
      }
      //  std::cout << "\t\t || s - s* || "
      //  std::cout << ( task.getError() ).sumSquare() <<std::endl ; ;
      vpTime::wait(t, sampling_time * 1000); // Wait 40 ms
    }
    pos--;
    task.kill();
  }

  simu->closeMainApplication();

  void *a = NULL;
  return a;
}

int main(int argc, const char **argv)
{
  try {
    std::string env_ipath;
    std::string opt_ipath;
    std::string ipath;
    std::string filename;
    bool opt_display = true;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty())
      ipath = env_ipath;

    // Read the command line options
    if (getOptions(argc, argv, opt_ipath, opt_display) == false) {
      exit(-1);
    }

    // Get the option values
    if (!opt_ipath.empty())
      ipath = opt_ipath;

    // Compare ipath and env_ipath. If they differ, we take into account
    // the input path comming from the command line option
    if (!opt_ipath.empty() && !env_ipath.empty()) {
      if (ipath != env_ipath) {
        std::cout << std::endl << "WARNING: " << std::endl;
        std::cout << "  Since -i <visp image path=" << ipath << "> "
                  << "  is different from VISP_INPUT_IMAGE_PATH=" << env_ipath << std::endl
                  << "  we skip the environment variable." << std::endl;
      }
    }

    // Test if an input path is set
    if (opt_ipath.empty() && env_ipath.empty()) {
      usage(argv[0], NULL, ipath);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
                << "  environment variable to specify the location of the " << std::endl
                << "  image path where test images are located." << std::endl
                << std::endl;
      exit(-1);
    }

    vpCameraParameters cam;
    vpHomogeneousMatrix fMo;
    fMo[2][3] = 0;

    if (opt_display) {

      vpSimulator simu;
      simu.initInternalViewer(300, 300);
      simu.initExternalViewer(300, 300);

      vpTime::wait(1000);
      simu.setZoomFactor(1.0f);
      simu.addAbsoluteFrame();

      // Load the cad model
      filename = vpIoTools::createFilePath(ipath, "iv/circle.iv");
      simu.load(filename.c_str(), fMo);

      simu.setInternalCameraParameters(cam);

      simu.initApplication(&mainLoop);
      simu.mainLoop();
    }
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
