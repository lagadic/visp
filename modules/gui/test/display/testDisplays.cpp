/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * Test for image display.
 */

/*!
  \example testDisplays.cpp

  \brief Test all the displays. Draws several shapes.
*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>

#include <iostream>
#include <stdlib.h>
#include <string>

#if defined(VISP_HAVE_GTK) || defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_D3D9) || defined(VISP_HAVE_OPENCV)

#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpRect.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>

#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

// List of allowed command line options
#define GETOPTARGS "hldc"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

/*!
  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
 */
static void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Test video devices or display.\n\
\n\
SYNOPSIS\n\
  %s [-l] [-c] [-d] [-h]\n\
",
name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -c\n\
     Disable the mouse click. Useful to automate the \n\
     execution of this program without human intervention.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -l\n\
     Print the list of video-devices available and exit.\n\
\n\
  -h\n\
     Print the help.\n\n");

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!
  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param list : To get the list of available display.
  \param click_allowed : Mouse click activation.
  \param display : Display activation.
  \return false if the program has to be stopped, true otherwise.
*/
static bool getOptions(int argc, const char **argv, bool &list, bool &click_allowed, bool &display)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'l':
      list = true;
      break;
    case 'h':
      usage(argv[0], nullptr);
      return false;
      break;
    case 'c':
      click_allowed = false;
      break;
    case 'd':
      display = false;
      break;

    default:
      usage(argv[0], optarg_);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], nullptr);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

template <typename Type> static void draw(vpImage<Type> &I)
{
  vpImagePoint iP1, iP2;
  unsigned int w, h;

  iP1.set_i(20);
  iP1.set_j(10);
  iP2.set_i(20);
  iP2.set_j(30);
  vpDisplay::displayArrow(I, iP1, iP2, vpColor::green, 4, 2, 3);

  iP1.set_i(20);
  iP1.set_j(60);
  vpDisplay::displayText(I, iP1, "Test...", vpColor::black);

  iP1.set_i(80);
  iP1.set_j(220);
  iP2.set_i(80);
  iP2.set_j(480);
  vpDisplay::displayCircle(I, iP1, 30, vpColor::red, false, 3);
  vpDisplay::displayCircle(I, iP2, 30, vpColor::red, true, 3);

  iP1.set_i(20);
  iP1.set_j(220);
  vpDisplay::displayCross(I, iP1, 5, vpColor::blue, 1);

  iP1.set_i(140);
  iP1.set_j(10);
  iP2.set_i(140);
  iP2.set_j(50);
  vpDisplay::displayDotLine(I, iP1, iP2, vpColor::blue, 3);

  iP1.set_i(120);
  iP1.set_j(180);
  iP2.set_i(160);
  iP2.set_j(250);
  vpDisplay::displayDotLine(I, iP1, iP2, vpColor::blue, 3);

  iP1.set_i(160);
  iP1.set_j(280);
  iP2.set_i(120);
  iP2.set_j(340);
  vpDisplay::displayDotLine(I, iP1, iP2, vpColor::blue, 3);

  iP1.set_i(220);
  iP1.set_j(400);
  iP2.set_i(120);
  iP2.set_j(400);
  vpDisplay::displayDotLine(I, iP1, iP2, vpColor::cyan, 3);

  iP1.set_i(220);
  iP1.set_j(480);
  iP2.set_i(120);
  iP2.set_j(450);
  vpDisplay::displayDotLine(I, iP1, iP2, vpColor::green, 3);

  vpHomogeneousMatrix cMo(vpTranslationVector(0.15, -0.07, 0.37), vpRotationMatrix(vpRxyzVector(0.1, -0.4, 0.41)));
  vpCameraParameters cam(600, 600, 320, 240);
  vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none, 3);

  iP1.set_i(140);
  iP1.set_j(80);
  iP2.set_i(140);
  iP2.set_j(150);
  vpDisplay::displayLine(I, iP1, iP2, vpColor::orange, 3);

  iP1.set_i(140);
  iP1.set_j(400);
  vpDisplay::displayPoint(I, iP1, vpColor::red);

  iP1.set_i(350);
  iP1.set_j(20);
  w = 60;
  h = 50;
  vpDisplay::displayRectangle(I, iP1, w, h, vpColor::red, false, 3);

  iP1.set_i(350);
  iP1.set_j(110);
  vpDisplay::displayRectangle(I, iP1, w, h, vpColor::red, true, 3);

  iP1.set_i(350);
  iP1.set_j(200);
  iP2.set_i(400);
  iP2.set_j(260);
  vpDisplay::displayRectangle(I, iP1, iP2, vpColor::orange, false, 3);

  iP1.set_i(350);
  iP1.set_j(290);
  iP2.set_i(400);
  iP2.set_j(350);
  vpRect rectangle(iP1, iP2);
  vpDisplay::displayRectangle(I, rectangle, vpColor::yellow, false, 3);

  iP1.set_i(380);
  iP1.set_j(400);
  vpDisplay::displayRectangle(I, iP1, 45, w, h, vpColor::green, 3);

  std::vector<vpImagePoint> vip;
  vip.push_back(vpImagePoint(250, 500));
  vip.push_back(vpImagePoint(350, 600));
  vip.push_back(vpImagePoint(450, 500));
  vip.push_back(vpImagePoint(350, 400));
  vpDisplay::displayPolygon(I, vip, vpColor::green, 3);

  vip.clear();
  vip.push_back(vpImagePoint(300, 500));
  vip.push_back(vpImagePoint(350, 550));
  vip.push_back(vpImagePoint(400, 500));
  vip.push_back(vpImagePoint(350, 450));
  vpDisplay::displayPolygon(I, vip, vpColor::cyan, 3, false);

  vip.clear();
  vip.push_back(vpImagePoint(250, 300));
  vip.push_back(vpImagePoint(350, 400));
  vip.push_back(vpImagePoint(450, 300));
  vip.push_back(vpImagePoint(350, 200));
  vpPolygon polygon(vip);
  vpDisplay::displayPolygon(I, polygon, vpColor::green, 3);

  vip.clear();
  vip.push_back(vpImagePoint(300, 300));
  vip.push_back(vpImagePoint(350, 350));
  vip.push_back(vpImagePoint(400, 300));
  vip.push_back(vpImagePoint(350, 250));
  polygon.build(vip);
  vpDisplay::displayPolygon(I, polygon, vpColor::cyan, 3, false);
}

template <typename Type>
static void runTest(bool opt_display, bool opt_click_allowed)
{
  vpImage<Type> Ix;
  vpImage<Type> Igtk;
  vpImage<Type> Icv;
  vpImage<Type> Igdi;
  vpImage<Type> Id3d;

#if defined(VISP_HAVE_X11)
  vpDisplayX *displayX = new vpDisplayX;
  Ix.init(480, 640, Type(255));
  if (opt_display) {
    displayX->init(Ix, 100, 100, "Display X11");
    vpDisplay::display(Ix);
    draw(Ix);
    vpDisplay::flush(Ix);
    if (opt_click_allowed)
      vpDisplay::getClick(Ix);
  }
#endif

#if defined(HAVE_OPENCV_HIGHGUI)
  vpDisplayOpenCV *displayCv = new vpDisplayOpenCV;
  Icv.init(480, 640, Type(255));
  if (opt_display) {
    displayCv->init(Icv, 100, 100, "Display OpenCV");
    vpDisplay::display(Icv);
    draw(Icv);
    vpDisplay::flush(Icv);
    if (opt_click_allowed)
      vpDisplay::getClick(Icv);
  }
#endif

#if defined(VISP_HAVE_GTK)
  vpDisplayGTK *displayGtk = new vpDisplayGTK;
  Igtk.init(480, 640, Type(255));
  if (opt_display) {
    displayGtk->init(Igtk, 100, 100, "Display GTK");
    vpDisplay::display(Igtk);
    draw(Igtk);
    vpDisplay::flush(Igtk);
    if (opt_click_allowed)
      vpDisplay::getClick(Igtk);
  }
#endif

#if defined(VISP_HAVE_GDI)

  vpDisplayGDI *displayGdi = new vpDisplayGDI;
  Igdi.init(480, 640, Type(255));
  if (opt_display) {
    displayGdi->init(Igdi, 100, 100, "Display GDI");
    vpDisplay::display(Igdi);
    draw(Igdi);
    vpDisplay::flush(Igdi);
    if (opt_click_allowed)
      vpDisplay::getClick(Igdi);
  }
#endif

#if defined(VISP_HAVE_D3D9)
  vpDisplayD3D *displayD3d = new vpDisplayD3D;
  Id3d.init(480, 640, Type(255));
  if (opt_display) {
    displayD3d->init(Id3d, 100, 100, "Display Direct 3D");
    vpDisplay::display(Id3d);
    draw(Id3d);
    vpDisplay::flush(Id3d);
    if (opt_click_allowed)
      vpDisplay::getClick(Id3d);
  }
#endif

#if defined(VISP_HAVE_X11)
  delete displayX;
#endif

#if defined(VISP_HAVE_GTK)
  delete displayGtk;
#endif

#if defined(HAVE_OPENCV_HIGHGUI)
  delete displayCv;
#endif

#if defined(VISP_HAVE_GDI)

  delete displayGdi;
#endif

#if defined(VISP_HAVE_D3D9)
  delete displayD3d;
#endif
}

int main(int argc, const char **argv)
{
  try {
    bool opt_list = false; // To print the list of video devices
    bool opt_click_allowed = true;
    bool opt_display = true;

    // Read the command line options
    if (getOptions(argc, argv, opt_list, opt_click_allowed, opt_display) == false) {
      return EXIT_FAILURE;
    }

    // Print the list of video-devices available
    if (opt_list) {
      unsigned nbDevices = 0;
      std::cout << "List of video-devices available: \n";
#if defined(VISP_HAVE_GTK)
      std::cout << "  GTK\n";
      nbDevices++;
#endif
#if defined(VISP_HAVE_X11)
      std::cout << "  X11\n";
      nbDevices++;
#endif
#if defined(VISP_HAVE_GDI)

      std::cout << "  GDI\n";
      nbDevices++;
#endif
#if defined(VISP_HAVE_D3D9)
      std::cout << "  D3D\n";
      nbDevices++;
#endif
#if defined VISP_HAVE_OPENCV
      std::cout << "  OpenCV\n";
      nbDevices++;
#endif
      if (!nbDevices) {
        std::cout << "  No display is available\n";
      }
      return EXIT_FAILURE;
    }

    // Create a color image for each display.
    runTest<vpRGBa>(opt_display, opt_click_allowed);

    // Create a grayscale image for each display.
    runTest<unsigned char>(opt_display, opt_click_allowed);

    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
    return EXIT_FAILURE;
  }
}
#else
int main()
{
  std::cout << "You do not have display functionalities..." << std::endl;
  return EXIT_SUCCESS;
}
#endif
