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
 * Test for image display.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <iostream>
#include <stdlib.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpParseArgv.h>
#if (defined(VISP_HAVE_GTK) || defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_D3D9) ||          \
     defined(VISP_HAVE_OPENCV))

/*!
  \example testVideoDeviceDual.cpp

  \brief Example showing how to open two video devices.

*/

// List of allowed command line options
#define GETOPTARGS "hlt:dc"

typedef enum { vpX11, vpGTK, vpGDI, vpD3D, vpCV } vpDisplayType;

void usage(const char *name, const char *badparam, vpDisplayType &dtype);
bool getOptions(int argc, const char **argv, vpDisplayType &dtype, bool &list, bool &click_allowed, bool &display);

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param dtype : Type of video device.

 */
void usage(const char *name, const char *badparam, vpDisplayType &dtype)
{
  fprintf(stdout, "\n\
Test to open video devices or display.\n\
\n\
SYNOPSIS\n\
  %s [-t <type of video device>] [-l] [-c] [-d] [-h]\n\
", name);

  std::string display;
  switch (dtype) {
  case vpX11:
    display = "X11";
    break;
  case vpGTK:
    display = "GTK";
    break;
  case vpGDI:
    display = "GDI";
    break;
  case vpD3D:
    display = "D3D";
    break;
  case vpCV:
    display = "CV";
    break;
  }

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -t <type of video device>                            \"%s\"\n\
     String specifying the video device to use.\n\
     Possible values:\n\
       \"X11\": only on UNIX platforms,\n\
       \"GTK\": on all plaforms,\n\
       \"GDI\": only on Windows platform (Graphics Device Interface),\n\
       \"D3D\": only on Windows platform (Direct3D).\n\
       \"CV\" : (OpenCV).\n\
\n\
  -c\n\
     Disable the mouse click. Useful to automaze the \n\
     execution of this program without humain intervention.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -l\n\
     Print the list of video-devices available and exit.\n\
\n\
  -h\n\
     Print the help.\n\n", display.c_str());

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param dtype : Type of display.
  \param list : To get the list of available display.
  \param click_allowed : Mouse click activation.
  \param display : Display activation.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, vpDisplayType &dtype, bool &list, bool &click_allowed, bool &display)
{
  const char *optarg_;
  int c;
  std::string sDisplayType;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'l':
      list = true;
      break;
    case 't':
      sDisplayType = optarg_;
      // Parse the display type option
      if (sDisplayType.compare("X11") == 0) {
        dtype = vpX11;
      } else if (sDisplayType.compare("GTK") == 0) {
        dtype = vpGTK;
      } else if (sDisplayType.compare("GDI") == 0) {
        dtype = vpGDI;
      } else if (sDisplayType.compare("D3D") == 0) {
        dtype = vpD3D;
      } else if (sDisplayType.compare("CV") == 0) {
        dtype = vpCV;
      }

      break;
    case 'h':
      usage(argv[0], NULL, dtype);
      return false;
      break;
    case 'c':
      click_allowed = false;
      break;
    case 'd':
      display = false;
      break;

    default:
      usage(argv[0], optarg_, dtype);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, dtype);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

int main(int argc, const char **argv)
{
  try {
    bool opt_list = false;   // To print the list of video devices
    vpDisplayType opt_dtype; // Type of display to use
    bool opt_click_allowed = true;
    bool opt_display = true;

// Default display is one available
#if defined VISP_HAVE_GTK
    opt_dtype = vpGTK;
#elif defined VISP_HAVE_X11
    opt_dtype = vpX11;
#elif defined VISP_HAVE_GDI
    opt_dtype = vpGDI;
#elif defined VISP_HAVE_D3D9
    opt_dtype = vpD3D;
#elif defined VISP_HAVE_OPENCV
    opt_dtype = vpCV;
#endif

    // Read the command line options
    if (getOptions(argc, argv, opt_dtype, opt_list, opt_click_allowed, opt_display) == false) {
      exit(-1);
    }

    // Print the list of video-devices available
    if (opt_list) {
      unsigned nbDevices = 0;
      std::cout << "List of video-devices available: \n";
#if defined VISP_HAVE_GTK
      std::cout << "  GTK (use \"-t GTK\" option to use it)\n";
      nbDevices++;
#endif
#if defined VISP_HAVE_X11
      std::cout << "  X11 (use \"-t X11\" option to use it)\n";
      nbDevices++;
#endif
#if defined VISP_HAVE_GDI
      std::cout << "  GDI (use \"-t GDI\" option to use it)\n";
      nbDevices++;
#endif
#if defined VISP_HAVE_D3D9
      std::cout << "  D3D (use \"-t D3D\" option to use it)\n";
      nbDevices++;
#endif
#if defined VISP_HAVE_OPENCV
      std::cout << "  CV (use \"-t CV\" option to use it)\n";
      nbDevices++;
#endif
      if (!nbDevices) {
        std::cout << "  No display is available\n";
      }
      return (0);
    }

    // Create 2 images
    vpImage<unsigned char> I1(240, 320), I2(240, 320);
    I1 = 128;
    I2 = 255;

    // Create 2 display
    vpDisplay *d1 = NULL, *d2 = NULL;

    // Initialize the displays
    switch (opt_dtype) {
    case vpX11:
      std::cout << "Requested X11 display functionnalities..." << std::endl;
#if defined VISP_HAVE_X11
      d1 = new vpDisplayX;
      d2 = new vpDisplayX;
#else
      std::cout << "  Sorry, X11 video device is not available.\n";
      std::cout << "Use \"" << argv[0] << " -l\" to print the list of available devices.\n";
      return 0;
#endif
      break;
    case vpGTK:
      std::cout << "Requested GTK display functionnalities..." << std::endl;
#if defined VISP_HAVE_GTK
      d1 = new vpDisplayGTK;
      d2 = new vpDisplayGTK;
#else
      std::cout << "  Sorry, GTK video device is not available.\n";
      std::cout << "Use \"" << argv[0] << " -l\" to print the list of available devices.\n";
      return 0;
#endif
      break;
    case vpGDI:
      std::cout << "Requested GDI display functionnalities..." << std::endl;
#if defined VISP_HAVE_GDI
      d1 = new vpDisplayGDI;
      d2 = new vpDisplayGDI;
#else
      std::cout << "  Sorry, GDI video device is not available.\n";
      std::cout << "Use \"" << argv[0] << " -l\" to print the list of available devices.\n";
      return 0;
#endif
      break;
    case vpD3D:
      std::cout << "Requested D3D display functionnalities..." << std::endl;
#if defined VISP_HAVE_D3D9
      d1 = new vpDisplayD3D;
      d2 = new vpDisplayD3D;
#else
      std::cout << "  Sorry, D3D video device is not available.\n";
      std::cout << "Use \"" << argv[0] << " -l\" to print the list of available devices.\n";
      return 0;
#endif
      break;
    case vpCV:
      std::cout << "Requested OpenCV display functionnalities..." << std::endl;
#if defined(VISP_HAVE_OPENCV)
      d1 = new vpDisplayOpenCV;
      d2 = new vpDisplayOpenCV;
#else
      std::cout << "  Sorry, OpenCV video device is not available.\n";
      std::cout << "Use \"" << argv[0] << " -l\" to print the list of available devices.\n";
      return 0;
#endif
      break;
    }

    if (opt_display) {
      int winx1 = 100, winy1 = 200;
      d1->init(I1, winx1, winy1, "Display 1");

      int winx2 = winx1 + 10 + (int)I1.getWidth(), winy2 = winy1;
      d2->init(I2, winx2, winy2, "Display 2");

      vpDisplay::display(I1);
      vpDisplay::display(I2);

      vpDisplay::flush(I1);
      vpDisplay::flush(I2);
    }

    std::cout << "A click in display 1 to exit..." << std::endl;
    if (opt_click_allowed)
      vpDisplay::getClick(I1);

    delete d1;
    delete d2;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}

#else
int main() { vpERROR_TRACE("You do not have display functionalities..."); }

#endif
