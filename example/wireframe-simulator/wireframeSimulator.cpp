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
 * Demonstration of the wireframe simulator
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

/*!
  \example wireframeSimulator.cpp

  Demonstration of the wireframe simulator.
*/

#include <stdlib.h>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/robot/vpWireFrameSimulator.h>

#define GETOPTARGS "cdh"

#ifdef VISP_HAVE_DISPLAY

void usage(const char *name, const char *badparam);
bool getOptions(int argc, const char **argv, bool &display, bool &click);

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Demonstration of the wireframe simulator.\n\
\n\
The goal of this example is to present the basic functionalities of the wire frame simulator.\n\
\n\
SYNOPSIS\n\
  %s [-c] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -c \n\
     Disable mouse click.\n\
\n\
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
  \param click : Activates mouse click.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, bool &display, bool &click)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'c':
      click = false;
      break;
    case 'd':
      display = false;
      break;
    case 'h':
      usage(argv[0], NULL);
      return false;
      break;

    default:
      usage(argv[0], optarg_);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL);
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
    bool opt_click = true;

    // Read the command line options
    if (getOptions(argc, argv, opt_display, opt_click) == false) {
      exit(-1);
    }

    /*
    Three vpImage are created : one for the main camera and the others
    for two external cameras
  */
    vpImage<vpRGBa> Iint(480, 640, 255);
    vpImage<vpRGBa> Iext1(480, 640, 255);
    vpImage<vpRGBa> Iext2(480, 640, 255);

/*
Create a display for each different cameras.
*/
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

    // The homogeneous matrix which gives the current position of the main
    // camera relative to the object
    vpHomogeneousMatrix cMo(0, 0.05, 1.3, vpMath::rad(15), vpMath::rad(25), 0);

    // The homogeneous matrix which gives the desired position of the main
    // camera relative to the object
    vpHomogeneousMatrix cdMo(vpHomogeneousMatrix(0.0, 0.0, 1.0, vpMath::rad(0), vpMath::rad(0), vpMath::rad(0)));

    // Declaration of the simulator
    vpWireFrameSimulator sim;
    /*
    Set the scene. It enables to choose the shape of the object and the shape
    of the desired object which is displayed in the main camera view. It
    exists several objects in ViSP. See the html documentation of the
    simulator class to have the complete list.

    Note : if you don't want to have a desired object displayed in the main
    camera view you can use the initObject Method.

    Here the object is a plate with 4 points and it is the same object which
    is used to display the object at the desired position.
  */
    sim.initScene(vpWireFrameSimulator::PLATE, vpWireFrameSimulator::D_STANDARD);

    /*
    The object at the current position will be displayed in blue
    The object at the desired position will be displayed in red
    The camera will be display in green
  */
    sim.setCurrentViewColor(vpColor::blue);
    sim.setDesiredViewColor(vpColor::red);
    sim.setCameraColor(vpColor::green);
    /*
    Set the current and the desired position of the camera relative to the
    object.
  */
    sim.setCameraPositionRelObj(cMo);
    sim.setDesiredCameraPosition(cdMo);
    /*
    Set the main external camera's position relative to the world reference
    frame. More information about the different frames are given in the html
    documentation.
  */
    vpHomogeneousMatrix camMw(vpHomogeneousMatrix(0.0, 0, 4.5, vpMath::rad(0), vpMath::rad(-30), 0));
    sim.setExternalCameraPosition(camMw);

    /*
    Set the parameters of the cameras (internal and external)
  */
    vpCameraParameters camera(1000, 1000, 320, 240);
    sim.setInternalCameraParameters(camera);
    sim.setExternalCameraParameters(camera);

    vpHomogeneousMatrix camoMw(vpHomogeneousMatrix(-0.3, 0.2, 2.5, vpMath::rad(0), vpMath::rad(10), 0));

    if (opt_display) {
      // Get the view of the internal camera
      sim.getInternalImage(Iint);
      // Get the view of the main external camera
      sim.getExternalImage(Iext1);
      // Get the view of an external camera that you can positionned thanks
      // to a vpHomogeneousMatrix which describes the position of the camera
      // relative to the world reference frame.
      sim.getExternalImage(Iext2, camoMw);
      // Display the views.

      vpDisplay::flush(Iint);
      vpDisplay::flush(Iext1);
      vpDisplay::flush(Iext2);
    }

    std::cout << std::endl;
    std::cout << "Here are presented the effect of the basic functions of "
                 "the simulator"
              << std::endl;
    std::cout << std::endl;

    if (opt_display) {
      if (opt_click) {
        std::cout << "Click on the internal view window to continue. the "
                     "object will move. The external cameras are fixed. The "
                     "main camera moves too because the homogeneous matrix "
                     "cMo didn't change."
                  << std::endl;
        vpDisplay::getClick(Iint);
      }
      vpDisplay::display(Iint);
      vpDisplay::display(Iext1);
      vpDisplay::display(Iext2);
    }
    /*
    To move the object you have to define a vpHomogeneousMatrix which gives
    the position of the object relative to the world refrenece frame.
  */
    vpHomogeneousMatrix mov(0.05, 0.05, 0.2, vpMath::rad(10), 0, 0);
    sim.set_fMo(mov);

    if (opt_display) {
      // Get the view of the internal camera
      sim.getInternalImage(Iint);
      // Get the view of the main external camera
      sim.getExternalImage(Iext1);
      // Get the view of an external camera that you can positionned thanks
      // to a vpHomogeneousMatrix which describes the position of the camera
      // relative to the world reference frame.
      sim.getExternalImage(Iext2, camoMw);
      // Display the views.

      vpDisplay::flush(Iint);
      vpDisplay::flush(Iext1);
      vpDisplay::flush(Iext2);
    }

    std::cout << std::endl;
    if (opt_display) {
      if (opt_click) {
        std::cout << "Click on the internal view window to continue" << std::endl;
        vpDisplay::getClick(Iint);
      }
    }
    std::cout << std::endl;
    std::cout << "Now you can move the main external camera. Click inside "
                 "the corresponding window with one of the three buttons of "
                 "your mouse and move the pointer."
              << std::endl;
    std::cout << std::endl;
    std::cout << "Click on the internal view window when you are finished" << std::endl;

    /*
    To move the main external camera you need a loop containing the
    getExternalImage method. This functionnality is only available for the
    main external camera.
  */
    if (opt_display && opt_click) {
      while (!vpDisplay::getClick(Iint, false)) {
        vpDisplay::display(Iext1);
        sim.getExternalImage(Iext1);
        vpDisplay::flush(Iext1);
      }
    }

    std::cout << std::endl;
    std::cout << "You have seen the main capabilities of the simulator. "
                 "Other specific functionalities are available. Please "
                 "refers to the html documentation to access the list of all "
                 "functions"
              << std::endl;
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_SUCCESS;
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
