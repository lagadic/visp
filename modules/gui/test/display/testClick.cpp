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
 * Test for mouse click manipulations.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>

#include <iostream>
#include <stdlib.h>
#include <string>

#if (defined(VISP_HAVE_GTK) || defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_D3D9) ||          \
     defined(VISP_HAVE_OPENCV))

#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>

#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

/*!
  \example testClick.cpp

  \brief Mouse click manipulations.

*/

// List of allowed command line options
#define GETOPTARGS "i:hlt:dc"

typedef enum { vpX11, vpGTK, vpGDI, vpD3D, vpCV } vpDisplayType;

void usage(const char *name, const char *badparam, std::string ipath, vpDisplayType &dtype);
bool getOptions(int argc, const char **argv, std::string &ipath, vpDisplayType &dtype, bool &list, bool &click_allowed,
                bool &display);

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath: Input image path.
  \param dtype : Type of video device.

 */
void usage(const char *name, const char *badparam, std::string ipath, vpDisplayType &dtype)
{
  fprintf(stdout, "\n\
Test click functionnalities in video devices or display.\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] \n\
     [-t <type of video device>] [-l] [-c] [-d] [-h]\n\
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
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read \"Klimt/Klimt.pgm\"\n\
     and \"Klimt/Klimt.ppm\" images.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
\n\
  -t <type of video device>                            \"%s\"\n\
     String specifying the video device to use.\n\
     Possible values:\n\
       \"X11\": only on UNIX platforms,\n\
       \"GTK\": on all plaforms,\n\
       \"GDI\": only on Windows platform (Graphics Device Interface),\n\
       \"D3D\": only on Windows platform (Direct3D).\n\
       \"CV\" : (OpenCV).\n\
\n\
  -l\n\
     Print the list of video-devices available and exit.\n\
\n\
  -c\n\
     Disable the mouse click. Useful to automaze the \n\
     execution of this program without humain intervention.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -h\n\
     Print the help.\n\n", ipath.c_str(), display.c_str());

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath: Input image path.
  \param dtype : Type of video device.
  \param list : Set as true,list the available video-devices.
  \param click_allowed : Enable/disable mouse click.
  \param display : Set as true, activates the image display. This is
  the default configuration. When set to false, the display is
  disabled. This can be useful for automatic tests using crontab
  under Unix or using the task manager under Windows.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &ipath, vpDisplayType &dtype, bool &list, bool &click_allowed,
                bool &display)
{
  const char *optarg_;
  int c;
  std::string sDisplayType;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'i':
      ipath = optarg_;
      break;
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
      usage(argv[0], NULL, ipath, dtype);
      return false;
      break;
    case 'c':
      click_allowed = false;
      break;
    case 'd':
      display = false;
      break;

    default:
      usage(argv[0], optarg_, ipath, dtype);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath, dtype);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

int main(int argc, const char **argv)
{
  try {
    std::string env_ipath;
    std::string opt_ipath;
    bool opt_list = false;   // To print the list of video devices
    vpDisplayType opt_dtype; // Type of display to use
    std::string ipath;
    std::string filename;
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

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty())
      ipath = env_ipath;

    // Read the command line options
    if (getOptions(argc, argv, opt_ipath, opt_dtype, opt_list, opt_click_allowed, opt_display) == false) {
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

    // Get the option values
    if (!opt_ipath.empty())
      ipath = opt_ipath;

    // Compare ipath and env_ipath. If they differ, we take into account
    // the input path comming from the command line option
    if (!opt_ipath.empty() && !env_ipath.empty()) {
      if (ipath != env_ipath) {
        std::cout << std::endl << "WARNING: " << std::endl;
        std::cout << "  Since -i <visp image path=" << ipath << "> "
                  << "  is different from VISP_IMAGE_PATH=" << env_ipath << std::endl
                  << "  we skip the environment variable." << std::endl;
      }
    }

    // Test if an input path is set
    if (opt_ipath.empty() && env_ipath.empty()) {
      usage(argv[0], NULL, ipath, opt_dtype);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
                << "  environment variable to specify the location of the " << std::endl
                << "  image path where test images are located." << std::endl
                << std::endl;
      exit(-1);
    }

    // Create a grey level image
    vpImage<unsigned char> I;

    // Load a grey image from the disk
    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");
    vpCTRACE << "Load " << filename << std::endl;
    vpImageIo::read(I, filename);

    // Create a display for the image
    vpDisplay *display = NULL;

    switch (opt_dtype) {
    case vpX11:
      std::cout << "Requested X11 display functionnalities..." << std::endl;
#if defined VISP_HAVE_X11
      display = new vpDisplayX;
#else
      std::cout << "  Sorry, X11 video device is not available.\n";
      std::cout << "Use \"" << argv[0] << " -l\" to print the list of available devices.\n";
      return 0;
#endif
      break;
    case vpGTK:
      std::cout << "Requested GTK display functionnalities..." << std::endl;
#if defined VISP_HAVE_GTK
      display = new vpDisplayGTK;
#else
      std::cout << "  Sorry, GTK video device is not available.\n";
      std::cout << "Use \"" << argv[0] << " -l\" to print the list of available devices.\n";
      return 0;
#endif
      break;
    case vpGDI:
      std::cout << "Requested GDI display functionnalities..." << std::endl;
#if defined VISP_HAVE_GDI
      display = new vpDisplayGDI;
#else
      std::cout << "  Sorry, GDI video device is not available.\n";
      std::cout << "Use \"" << argv[0] << " -l\" to print the list of available devices.\n";
      return 0;
#endif
      break;
    case vpD3D:
      std::cout << "Requested D3D display functionnalities..." << std::endl;
#if defined VISP_HAVE_D3D9
      display = new vpDisplayD3D;
#else
      std::cout << "  Sorry, D3D video device is not available.\n";
      std::cout << "Use \"" << argv[0] << " -l\" to print the list of available devices.\n";
      return 0;
#endif
      break;
    case vpCV:
      std::cout << "Requested OpenCV display functionnalities..." << std::endl;
#if defined(VISP_HAVE_OPENCV)
      display = new vpDisplayOpenCV;
#else
      std::cout << "  Sorry, OpenCV video device is not available.\n";
      std::cout << "Use \"" << argv[0] << " -l\" to print the list of available devices.\n";
      return 0;
#endif
      break;
    }

    if (opt_display) {

      // We open a window using either X11 or GTK or GDI.
      // Its size is automatically defined by the image (I) size
      display->init(I, 100, 100, "Display...");

      // Display the image
      // The image class has a member that specify a pointer toward
      // the display that has been initialized in the display declaration
      // therefore is is no longuer necessary to make a reference to the
      // display variable.
      vpDisplay::display(I);
      // Flush the display
      vpDisplay::flush(I);
      if (opt_click_allowed) {
        std::cout << "Click on a pixel to get his coordinates...\n";
        vpImagePoint ip;
        vpMouseButton::vpMouseButtonType button;
        vpDisplay::getClick(I, ip, button);
        std::cout << "  You click down on pixel (" << ip << ") ";
        switch (button) {
        case vpMouseButton::button1:
          std::cout << "with left button.\n";
          break;
        case vpMouseButton::button2:
          std::cout << "with middle button.\n";
          break;
        case vpMouseButton::button3:
          std::cout << "with right button.\n";
          break;
        case vpMouseButton::none:
          break;
        }
        vpDisplay::getClickUp(I, ip, button);
        std::cout << "  You click up on pixel (" << ip << ") ";
        switch (button) {
        case vpMouseButton::button1:
          std::cout << "with left button.\n";
          break;
        case vpMouseButton::button2:
          std::cout << "with middle button.\n";
          break;
        case vpMouseButton::button3:
          std::cout << "with right button.\n";
          break;
        case vpMouseButton::none:
          break;
        }
        vpDisplay::getPointerPosition(I, ip);
        std::cout << "  Pointer poisition : " << ip << std::endl;
        std::cout << "A click to exit...\n";
        vpDisplay::getClick(I);
      }
    }
    delete display;
  } catch (...) {
    vpERROR_TRACE("Error while displaying the image");
    exit(-1);
  }
}

#else
int main() { vpERROR_TRACE("You do not have display functionalities..."); }

#endif
