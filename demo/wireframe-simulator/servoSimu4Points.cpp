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
 * Demonstration of the wireframe simulator with a simple visual servoing
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

/*!
  \example servoSimu4Points.cpp

  Demonstration of the wireframe simulator with a simple visual servoing.
*/

#include <stdlib.h>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpTime.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/robot/vpWireFrameSimulator.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>

#define GETOPTARGS "dh"

#ifdef VISP_HAVE_DISPLAY

void usage(const char *name, std::string ipath, const char *badparam);
bool getOptions(int argc, const char **argv, std::string &ipath, bool &display);

/*!

  Print the program options.

  \param name : Program name.
  \param ipath : Input image path.
  \param badparam : Bad parameter name.

*/
void usage(const char *name, std::string ipath, const char *badparam)
{
  fprintf(stdout, "\n\
Demonstration of the wireframe simulator with a simple visual servoing.\n\
          \n\
The visual servoing consists in bringing the camera at a desired \n\
position from the object.\n\
          \n\
The visual features used to compute the pose of the camera and \n\
thus the control law are four points.\n\
          \n\
This demonstration explains also how to move the object around a world\n\
reference frame. Here, the movement is a rotation around the x and y axis\n\
at a given distance from the world frame. In fact the object trajectory\n\
is on a sphere whose center is the origin of the world frame.\n\
          \n\
SYNOPSIS\n\
  %s [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set mire.pgm image input path.\n\
     From this path read \"mire/mire.pgm\" image.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment variable \n\
     produces the same behaviour than using this option.\n\
            \n\
  -d \n\
     Turn off the display.\n\
            \n\
  -h\n\
     Print the help.\n", ipath.c_str());

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath : Input image path.
  \param display : Display activation.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &ipath, bool &display)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'i':
      ipath = optarg_;
      break;
    case 'd':
      display = false;
      break;
    case 'h':
      usage(argv[0], ipath, NULL);
      return false;
      break;

    default:
      usage(argv[0], ipath, optarg_);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], ipath, NULL);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

int main(int argc, const char **argv)
{
  try {
    bool opt_display = true;
    std::string opt_ipath;
    std::string env_ipath;
    std::string ipath;
    std::string filename;

    // Read the command line options
    if (getOptions(argc, argv, opt_ipath, opt_display) == false) {
      exit(-1);
    }

    vpImage<vpRGBa> Iint(480, 640, 255);
    vpImage<vpRGBa> Iext1(480, 640, 255);
    vpImage<vpRGBa> Iext2(480, 640, 255);

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

    if (opt_display) {
      // Display size is automatically defined by the image (I) size
      display[0].init(Iint, 100, 100, "The internal view");
      display[1].init(Iext1, 100, 100, "The first external view");
      display[2].init(Iext2, 100, 100, "The second external view");
      vpDisplay::setWindowPosition(Iint, 0, 0);
      vpDisplay::setWindowPosition(Iext1, 700, 0);
      vpDisplay::setWindowPosition(Iext2, 0, 550);
      vpDisplay::display(Iint);
      vpDisplay::flush(Iint);
      vpDisplay::display(Iext1);
      vpDisplay::flush(Iext1);
      vpDisplay::display(Iext2);
      vpDisplay::flush(Iext2);
    }

    vpServo task;
    vpSimulatorCamera robot;
    float sampling_time = 0.040f; // Sampling period in second
    robot.setSamplingTime(sampling_time);

    // Since the task gain lambda is very high, we need to increase default
    // max velocities
    robot.setMaxTranslationVelocity(10);
    robot.setMaxRotationVelocity(vpMath::rad(180));

    // Set initial position of the object in the camera frame
    vpHomogeneousMatrix cMo(0, 0.1, 2.0, vpMath::rad(35), vpMath::rad(25), 0);
    // Set desired position of the object in the camera frame
    vpHomogeneousMatrix cdMo(0.0, 0.0, 1.0, vpMath::rad(0), vpMath::rad(0), vpMath::rad(0));
    // Set initial position of the object in the world frame
    vpHomogeneousMatrix wMo(0.0, 0.0, 0.2, 0, 0, 0);
    // Position of the camera in the world frame
    vpHomogeneousMatrix wMc;
    wMc = wMo * cMo.inverse();

    // The four point used as visual features
    vpPoint point[4];
    point[0].setWorldCoordinates(-0.1, -0.1, 0);
    point[3].setWorldCoordinates(-0.1, 0.1, 0);
    point[2].setWorldCoordinates(0.1, 0.1, 0);
    point[1].setWorldCoordinates(0.1, -0.1, 0);

    // Projection of the points
    for (int i = 0; i < 4; i++)
      point[i].track(cMo);

    // Set the current visual feature
    vpFeaturePoint p[4];
    for (int i = 0; i < 4; i++)
      vpFeatureBuilder::create(p[i], point[i]);

    // Projection of the points
    for (int i = 0; i < 4; i++)
      point[i].track(cdMo);

    vpFeaturePoint pd[4];
    for (int i = 0; i < 4; i++)
      vpFeatureBuilder::create(pd[i], point[i]);

    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    task.setInteractionMatrixType(vpServo::DESIRED);

    vpHomogeneousMatrix cMe; // Identity
    vpVelocityTwistMatrix cVe(cMe);
    task.set_cVe(cVe);

    vpMatrix eJe;
    robot.get_eJe(eJe);
    task.set_eJe(eJe);

    for (int i = 0; i < 4; i++)
      task.addFeature(p[i], pd[i]);

    task.setLambda(10);

    std::list<vpImageSimulator> list;
    vpImageSimulator imsim;

    vpColVector X[4];
    for (int i = 0; i < 4; i++)
      X[i].resize(3);
    X[0][0] = -0.2;
    X[0][1] = -0.2;
    X[0][2] = 0;

    X[1][0] = 0.2;
    X[1][1] = -0.2;
    X[1][2] = 0;

    X[2][0] = 0.2;
    X[2][1] = 0.2;
    X[2][2] = 0;

    X[3][0] = -0.2;
    X[3][1] = 0.2;
    X[3][2] = 0;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    if (!env_ipath.empty())
      ipath = env_ipath;

    if (!opt_ipath.empty())
      ipath = opt_ipath;

    filename = vpIoTools::createFilePath(ipath, "mire/mire.pgm");

    imsim.init(filename.c_str(), X);

    list.push_back(imsim);

    vpWireFrameSimulator sim;

    // Set the scene
    sim.initScene(vpWireFrameSimulator::PLATE, vpWireFrameSimulator::D_STANDARD, list);

    // Initialize simulator frames
    sim.set_fMo(wMo);                   // Position of the object in the world reference frame
    sim.setCameraPositionRelObj(cMo);   // initial position of the camera
    sim.setDesiredCameraPosition(cdMo); // desired position of the camera

    // Set the External camera position
    vpHomogeneousMatrix camMf(vpHomogeneousMatrix(0.0, 0, 3.5, vpMath::rad(0), vpMath::rad(30), 0));
    sim.setExternalCameraPosition(camMf);

    // Computes the position of a camera which is fixed in the object frame
    vpHomogeneousMatrix camoMf(0, 0.0, 1.5, 0, vpMath::rad(140), 0);
    camoMf = camoMf * (sim.get_fMo().inverse());

    // Set the parameters of the cameras (internal and external)
    vpCameraParameters camera(1000, 1000, 320, 240);
    sim.setInternalCameraParameters(camera);
    sim.setExternalCameraParameters(camera);

    int stop = 10;

    if (opt_display) {
      stop = 2500;

      // Get the internal and external views
      sim.getInternalImage(Iint);
      sim.getExternalImage(Iext1);
      sim.getExternalImage(Iext2, camoMf);

      // Display the object frame (current and desired position)
      vpDisplay::displayFrame(Iint, cMo, camera, 0.2, vpColor::none);
      vpDisplay::displayFrame(Iint, cdMo, camera, 0.2, vpColor::none);

      // Display the object frame the world reference frame and the camera
      // frame
      vpDisplay::displayFrame(Iext1, camMf * sim.get_fMo() * cMo.inverse(), camera, 0.2, vpColor::none);
      vpDisplay::displayFrame(Iext1, camMf * sim.get_fMo(), camera, 0.2, vpColor::none);
      vpDisplay::displayFrame(Iext1, camMf, camera, 0.2, vpColor::none);

      // Display the world reference frame and the object frame
      vpDisplay::displayFrame(Iext2, camoMf, camera, 0.2, vpColor::none);
      vpDisplay::displayFrame(Iext2, camoMf * sim.get_fMo(), camera, 0.05, vpColor::none);

      vpDisplay::flush(Iint);
      vpDisplay::flush(Iext1);
      vpDisplay::flush(Iext2);

      std::cout << "Click on a display" << std::endl;
      while (!vpDisplay::getClick(Iint, false) && !vpDisplay::getClick(Iext1, false) &&
             !vpDisplay::getClick(Iext2, false)) {
      };
    }

    robot.setPosition(wMc);
    // Print the task
    task.print();

    int iter = 0;
    vpColVector v;

    while (iter++ < stop) {
      if (opt_display) {
        vpDisplay::display(Iint);
        vpDisplay::display(Iext1);
        vpDisplay::display(Iext2);
      }

      double t = vpTime::measureTimeMs();

      robot.get_eJe(eJe);
      task.set_eJe(eJe);

      wMc = robot.getPosition();
      cMo = wMc.inverse() * wMo;
      for (int i = 0; i < 4; i++) {
        point[i].track(cMo);
        vpFeatureBuilder::create(p[i], point[i]);
      }

      v = task.computeControlLaw();
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

      // Compute the movement of the object around the world reference frame.
      vpHomogeneousMatrix a(0, 0, 0.2, 0, 0, 0);
      vpHomogeneousMatrix b(0, 0, 0, vpMath::rad(1.5 * iter), 0, 0);
      vpHomogeneousMatrix c(0, 0, 0, 0, vpMath::rad(2.5 * iter), 0);

      // Move the object in the world frame
      wMo = b * c * a;

      sim.set_fMo(wMo); // Move the object in the simulator
      sim.setCameraPositionRelObj(cMo);

      // Compute the position of the external view which is fixed in the
      // object frame
      camoMf.buildFrom(0, 0.0, 1.5, 0, vpMath::rad(150), 0);
      camoMf = camoMf * (sim.get_fMo().inverse());

      if (opt_display) {
        // Get the internal and external views
        sim.getInternalImage(Iint);
        sim.getExternalImage(Iext1);
        sim.getExternalImage(Iext2, camoMf);

        // Display the object frame (current and desired position)
        vpDisplay::displayFrame(Iint, cMo, camera, 0.2, vpColor::none);
        vpDisplay::displayFrame(Iint, cdMo, camera, 0.2, vpColor::none);

        // Display the camera frame, the object frame the world reference
        // frame
        vpDisplay::displayFrame(Iext1, sim.getExternalCameraPosition() * sim.get_fMo() * cMo.inverse(), camera, 0.2,
                                vpColor::none);
        vpDisplay::displayFrame(Iext1, sim.getExternalCameraPosition() * sim.get_fMo(), camera, 0.2, vpColor::none);
        vpDisplay::displayFrame(Iext1, sim.getExternalCameraPosition(), camera, 0.2, vpColor::none);

        // Display the world reference frame and the object frame
        vpDisplay::displayFrame(Iext2, camoMf, camera, 0.2, vpColor::none);
        vpDisplay::displayFrame(Iext2, camoMf * sim.get_fMo(), camera, 0.05, vpColor::none);

        vpDisplay::flush(Iint);
        vpDisplay::flush(Iext1);
        vpDisplay::flush(Iext2);
      }

      vpTime::wait(t, sampling_time * 1000); // Wait 40 ms

      std::cout << "|| s - s* || = " << (task.getError()).sumSquare() << std::endl;
    }

    task.print();
    task.kill();

    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
#else
int main()
{
  std::cout << "You do not have X11, or GDI (Graphical Device Interface), or GTK functionalities to display images..." << std::endl;
  std::cout << "Tip if you are on a unix-like system:" << std::endl;
  std::cout << "- Install X11, configure again ViSP using cmake and build again this example" << std::endl;
  std::cout << "Tip if you are on a windows-like system:" << std::endl;
  std::cout << "- Install GDI, configure again ViSP using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
}

#endif
