/*
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
 * Read an image sequence from the disk and display it.
 */
/*!

  \file testMouseEvent.cpp

  \brief Read an image sequence from the disk and display it.

  The sequence is made of separate images. Each image corresponds to a
  PGM file.

*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpParseArgv.h>

#include <iomanip>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#if (defined(VISP_HAVE_GTK) || defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_D3D9))

#include <visp3/core/vpImage.h>
#include <visp3/io/vpImageIo.h>

#include <visp3/core/vpMouseButton.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayX.h>

#include <visp3/core/vpTime.h>

/*!
  \example testMouseEvent.cpp

  Read an image sequence from the disk and display it.

  The sequence is made of separate images. Each image corresponds to a
  PGM file.

*/

// List of allowed command line options
#define GETOPTARGS "cdi:Lp:ht:f:l:s:w"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

typedef enum
{
  vpX11,
  vpGTK,
  vpGDI,
  vpD3D,
} vpDisplayType;

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath : Input image path.
  \param ppath : Personal image path.
  \param first : First image.
  \param last : Last image.
  \param step : Step between two images.
  \param dtype : Type of video device.

 */
void usage(const char *name, const char *badparam, std::string ipath, std::string ppath, unsigned first, unsigned last,
           unsigned step, vpDisplayType &dtype)
{
#if VISP_HAVE_DATASET_VERSION >= 0x030600
  std::string ext("png");
#else
  std::string ext("pgm");
#endif

  fprintf(stdout, "\n\
Read an image sequence from the disk and display it.\n\
The sequence is made of separate images. Each image corresponds\n\
to a PGM file.\n\
\n\
SYNOPSIS\n\
  %s [-i <test image path>] [-p <personal image path>]\n\
     [-f <first image>] [-l <last image>] [-s <step>] \n\
     [-t <type of video device>] [-L] [-w] [-c] [-d] [-h]\n\
 ",
          name);

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
  }

  fprintf(stdout, "\n\
 OPTIONS:                                               Default\n\
  -i <test image path>                                %s\n\
     Set image input path.\n\
     From this path read \"cube/image.%%04d.%s\"\n\
     images. These images come from ViSP-images-x.y.z.tar.gz\n\
     available on the ViSP website.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
 \n\
  -p <personal image path>                             %s\n\
     Specify a personal sequence containing images \n\
     to process.\n\
     By image sequence, we mean one file per image.\n\
     The format is selected by analysing the filename extension.\n\
     Example : \"/Temp/visp-images/cube/image.%%04d.%s\"\n\
     %%04d is for the image numbering.\n\
 \n\
  -f <first image>                                     %u\n\
     First image number of the sequence.\n\
 \n\
  -l <last image>                                %u\n\
     Last image number of the sequence.\n\
 \n\
  -s <step>                                            %u\n\
     Step between two images.\n\
\n\
  -t <type of video device>                            \"%s\"\n\
     String specifying the video device to use.\n\
     Possible values:\n\
     \"X11\": only on UNIX platforms,\n\
     \"GTK\": on all plaforms,\n\
     \"GDI\": only on Windows platform (Graphics Device Interface),\n\
     \"D3D\": only on Windows platform (Direct3D).\n\
\n\
  -L\n\
     Print the list of video-devices available and exit.\n\
\n\
  -c\n\
     Disable mouse click.\n\
\n\
  -d\n\
     Disable the image display. This can be useful \n\
     for automatic tests using crontab under Unix or \n\
     using the task manager under Windows.\n\
\n\
  -w\n\
     Wait for a mouse click between two images.\n\
     If the image display is disabled (using -d)\n\
     this option is without effect.\n\
\n\
  -h\n\
     Print the help.\n\n",
          ipath.c_str(), ext.c_str(), ppath.c_str(), ext.c_str(), first, last, step, display.c_str());

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}
/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath : Input image path.
  \param ppath : Personal image path.
  \param first : First image.
  \param last : Last image.
  \param step : Step between two images.
  \param dtype : Type of video device.
  \param list : Set as true,list the available video-devices.
  \param click : Set as true, activates the mouse click.
  \param display : Set as true, activates the image display. This is
  the default configuration. When set to false, the display is
  disabled. This can be useful for automatic tests using crontab
  under Unix or using the task manager under Windows.

  \param wait : Boolean for waiting a mouse click between two images.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &ppath, unsigned &first, unsigned &last,
                unsigned &step, vpDisplayType &dtype, bool &list, bool &display, bool &click, bool &wait)
{
  const char *optarg_;
  int c;
  std::string sDisplayType;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'c':
      click = false;
      break;
    case 'd':
      display = false;
      break;
    case 't':
      sDisplayType = optarg_;
      // Parse the display type option
      if (sDisplayType.compare("X11") == 0) {
        dtype = vpX11;
      }
      else if (sDisplayType.compare("GTK") == 0) {
        dtype = vpGTK;
      }
      else if (sDisplayType.compare("GDI") == 0) {
        dtype = vpGDI;
      }
      else if (sDisplayType.compare("D3D") == 0) {
        dtype = vpD3D;
      }

      break;
    case 'i':
      ipath = optarg_;
      break;
    case 'L':
      list = true;
      break;
    case 'p':
      ppath = optarg_;
      break;
    case 'f':
      first = (unsigned)atoi(optarg_);
      break;
    case 'l':
      last = (unsigned)atoi(optarg_);
      break;
    case 's':
      step = (unsigned)atoi(optarg_);
      break;
    case 'w':
      wait = true;
      break;
    case 'h':
      usage(argv[0], nullptr, ipath, ppath, first, last, step, dtype);
      return false;
      break;

    default:
      usage(argv[0], optarg_, ipath, ppath, first, last, step, dtype);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], nullptr, ipath, ppath, first, last, step, dtype);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

int main(int argc, const char **argv)
{
  std::string env_ipath;
  std::string opt_ipath;
  std::string ipath;
  std::string opt_ppath;
  std::string dirname;
  std::string filename;
  unsigned opt_first = 30;
  unsigned opt_last = 40;
  unsigned opt_step = 1;
  vpDisplayType opt_dtype; // Type of display to use
  bool opt_list = false;   // To print the list of video devices
  bool opt_display = true;
  bool opt_click = true;
  bool opt_click_blocking = false;

#if VISP_HAVE_DATASET_VERSION >= 0x030600
  std::string ext("png");
#else
  std::string ext("pgm");
#endif

// Default display is one available
#if defined(VISP_HAVE_GTK)
  opt_dtype = vpGTK;
#elif defined(VISP_HAVE_X11)
  opt_dtype = vpX11;
#elif defined(VISP_HAVE_GDI)
  opt_dtype = vpGDI;
#elif defined(VISP_HAVE_D3D9)
  opt_dtype = vpD3D;
#endif

  // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
  // environment variable value
  env_ipath = vpIoTools::getViSPImagesDataPath();

  // Set the default input path
  if (!env_ipath.empty())
    ipath = env_ipath;

  // Read the command line options
  if (getOptions(argc, argv, opt_ipath, opt_ppath, opt_first, opt_last, opt_step, opt_dtype, opt_list, opt_display,
                 opt_click, opt_click_blocking) == false) {
    return EXIT_FAILURE;
  }
  // Print the list of video-devices available
  if (opt_list) {
    unsigned nbDevices = 0;
    std::cout << "List of video-devices available: \n";
#if defined(VISP_HAVE_GTK)
    std::cout << "  GTK (use \"-t GTK\" option to use it)\n";
    nbDevices++;
#endif
#if defined(VISP_HAVE_X11)
    std::cout << "  X11 (use \"-t X11\" option to use it)\n";
    nbDevices++;
#endif
#if defined(VISP_HAVE_GDI)

    std::cout << "  GDI (use \"-t GDI\" option to use it)\n";
    nbDevices++;
#endif
#if defined(VISP_HAVE_D3D9)
    std::cout << "  D3D (use \"-t D3D\" option to use it)\n";
    nbDevices++;
#endif
    if (!nbDevices) {
      std::cout << "  No display is available\n";
    }
    return EXIT_FAILURE;
  }

  if (!opt_display)
    opt_click_blocking = false; // turn off the waiting

  // Get the option values
  if (!opt_ipath.empty())
    ipath = opt_ipath;

  // Compare ipath and env_ipath. If they differ, we take into account
  // the input path coming from the command line option
  if (!opt_ipath.empty() && !env_ipath.empty() && opt_ppath.empty()) {
    if (ipath != env_ipath) {
      std::cout << std::endl << "WARNING: " << std::endl;
      std::cout << "  Since -i <visp image path=" << ipath << "> "
        << "  is different from VISP_IMAGE_PATH=" << env_ipath << std::endl
        << "  we skip the environment variable." << std::endl;
    }
  }

  // Test if an input path is set
  if (opt_ipath.empty() && env_ipath.empty() && opt_ppath.empty()) {
    usage(argv[0], nullptr, ipath, opt_ppath, opt_first, opt_last, opt_step, opt_dtype);
    std::cerr << std::endl << "ERROR:" << std::endl;
    std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
      << "  environment variable to specify the location of the " << std::endl
      << "  image path where test images are located." << std::endl
      << "  Use -p <personal image path> option if you want to " << std::endl
      << "  use personal images." << std::endl
      << std::endl;

    return EXIT_FAILURE;
  }

  // Declare an image, this is a gray level image (unsigned char)
  // it size is not defined yet, it will be defined when the image will
  // read on the disk
  vpImage<unsigned char> I;

  unsigned iter = opt_first;
  std::ostringstream s;
  char cfilename[FILENAME_MAX];

  if (opt_ppath.empty()) {
    // Warning : the datset is available on https://visp.inria.fr/download/
    dirname = vpIoTools::createFilePath(ipath, "cube");

    // Build the name of the image file
    s.setf(std::ios::right, std::ios::adjustfield);
    s << "image." << std::setw(4) << std::setfill('0') << iter << "." << ext;
    filename = vpIoTools::createFilePath(dirname, s.str());
  }
  else {

    snprintf(cfilename, FILENAME_MAX, opt_ppath.c_str(), iter);
    filename = cfilename;
  }
  // Read image named "filename" and put the bitmap in I
  try {
    vpImageIo::read(I, filename);
  }
  catch (...) {
    std::cerr << std::endl << "ERROR:" << std::endl;
    std::cerr << "  Cannot read " << filename << std::endl;
    std::cerr << "  Check your -i " << ipath << " option, " << std::endl
      << "  or your -p " << opt_ppath << " option " << std::endl
      << "  or VISP_INPUT_IMAGE_PATH environment variable" << std::endl;
    return EXIT_FAILURE;
  }
  // Create a display for the image
  vpDisplay *display = nullptr;

  switch (opt_dtype) {
  case vpX11:
    std::cout << "Requested X11 display functionalities..." << std::endl;
#if defined(VISP_HAVE_X11)
    display = new vpDisplayX;
#else
    std::cout << "  Sorry, X11 video device is not available.\n";
    std::cout << "Use \"" << argv[0] << " -l\" to print the list of available devices.\n";
    return EXIT_FAILURE;
#endif
    break;
  case vpGTK:
    std::cout << "Requested GTK display functionalities..." << std::endl;
#if defined(VISP_HAVE_GTK)
    display = new vpDisplayGTK;
#else
    std::cout << "  Sorry, GTK video device is not available.\n";
    std::cout << "Use \"" << argv[0] << " -l\" to print the list of available devices.\n";
    return EXIT_FAILURE;
#endif
    break;
  case vpGDI:
    std::cout << "Requested GDI display functionalities..." << std::endl;
#if defined(VISP_HAVE_GDI)

    display = new vpDisplayGDI;
#else
    std::cout << "  Sorry, GDI video device is not available.\n";
    std::cout << "Use \"" << argv[0] << " -l\" to print the list of available devices.\n";
    return EXIT_FAILURE;
#endif
    break;
  case vpD3D:
    std::cout << "Requested D3D display functionalities..." << std::endl;
#if defined(VISP_HAVE_D3D9)
    display = new vpDisplayD3D;
#else
    std::cout << "  Sorry, D3D video device is not available.\n";
    std::cout << "Use \"" << argv[0] << " -l\" to print the list of available devices.\n";
    return EXIT_FAILURE;
#endif
    break;
  }

  if (opt_display) {
    try {
      // We open a window using either X11 or GTK or GDI.
      // Its size is automatically defined by the image (I) size
      display->init(I, 100, 100, "Display...");

      // Display the image
      // The image class has a member that specify a pointer toward
      // the display that has been initialized in the display declaration
      // therefore is is no longer necessary to make a reference to the
      // display variable.
      vpDisplay::display(I);
      vpDisplay::flush(I);
    }
    catch (...) {
      vpERROR_TRACE("Error while displaying the image");
      delete display;
      return EXIT_FAILURE;
    }
  }

  // this is the loop over the image sequence
  while (iter < opt_last) {
    try {
      double tms = vpTime::measureTimeMs();

      // set the new image name
      if (opt_ppath.empty()) {
        s.str("");
        s << "image." << std::setw(4) << std::setfill('0') << iter << "." << ext;
        filename = vpIoTools::createFilePath(dirname, s.str());
      }
      else {
        snprintf(cfilename, FILENAME_MAX, opt_ppath.c_str(), iter);
        filename = cfilename;
      }

      std::cout << "read : " << filename << std::endl;
      // read the image
      vpImageIo::read(I, filename);
      if (opt_display) {
        // Display the image
        vpDisplay::display(I);
        // Flush the display
        vpDisplay::flush(I);

        if (opt_click_blocking) {
          std::cout << "A click in the image to continue..." << std::endl;
        }
        vpImagePoint ip;

        if (opt_click) {
          vpMouseButton::vpMouseButtonType button;
          bool pressed = vpDisplay::getClick(I, ip, button, opt_click_blocking);
          if (pressed) {
            switch (button) {
            case vpMouseButton::button1:
              std::cout << "Left button was pressed." << std::endl;
              break;
            case vpMouseButton::button2:
              std::cout << "Middle button was pressed." << std::endl;
              break;
            case vpMouseButton::button3:
              std::cout << "Right button was pressed. Bye. " << std::endl;
              delete display;
              return EXIT_SUCCESS;
              break;
            case vpMouseButton::none:
              break;
            }
          }
        }
        vpTime::wait(tms, 1000);
      }
      else {
     // Synchronise the loop to 40 ms
        vpTime::wait(tms, 40);
      }
    }
    catch (...) {
      delete display;
      return EXIT_FAILURE;
    }
    iter += opt_step;
  }
  delete display;
}
#else
int main() { vpERROR_TRACE("You do not have X11 or GTK display functionalities..."); }

#endif
