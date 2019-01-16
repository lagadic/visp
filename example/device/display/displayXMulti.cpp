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
 * Read an image on the disk and display it using X11.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \file displayXMulti.cpp

  \brief Read a grey level image and a color image on the disk.
  Display these two images using vpDisplayX class, display some
  features (line, circle, caracters) in overlay and finaly write the image and
  the overlayed features in an image on the disk.

*/
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>

#ifdef VISP_HAVE_X11

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>

#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpParseArgv.h>

/*!
  \example displayXMulti.cpp

  Read a grey level image and a color image on the disk.
  Display these two images using vpDisplayX class, display some
  features (line, circle, caracters) in overlay and finaly write the image and
  the overlayed features in an image on the disk.

*/

// List of allowed command line options
#define GETOPTARGS "cdi:o:h"

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath : Input image path.
  \param opath : Output image path.
  \param user : Username.

 */
void usage(const char *name, const char *badparam, std::string ipath, std::string opath, std::string user)
{
  fprintf(stdout, "\n\
Read an image on the disk, display it using X11, display some\n\
features (line, circle, caracters) in overlay and finaly write \n\
the image and the overlayed features in an image on the disk.\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] [-o <output image path>]\n\
     [-c] [-d] [-h]\n						      \
", name);

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
  -o <output image path>                               %s\n\
     Set image output path.\n\
     From this directory, creates the \"%s\"\n\
     subdirectory depending on the username, where \n\
     Klimt_grey.overlay.ppm output image is written.\n\
\n\
  -c\n\
     Disable the mouse click. Useful to automate the \n\
     execution of this program without humain intervention.\n\
\n\
  -d                                             \n\
     Disable the image display. This can be useful \n\
     for automatic tests using crontab under Unix or \n\
     using the task manager under Windows.\n\
\n\
  -h\n\
     Print the help.\n\n", ipath.c_str(), opath.c_str(), user.c_str());

  if (badparam) {
    fprintf(stderr, "ERROR: \n");
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);
  }
}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath : Input image path.
  \param opath : Output image path.
  \param click_allowed : Enable/disable mouse click.
  \param user : Username.
  \param display : Set as true, activates the image display. This is
  the default configuration. When set to false, the display is
  disabled. This can be useful for automatic tests using crontab
  under Unix or using the task manager under Windows.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &opath, bool &click_allowed,
                const std::string &user, bool &display)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'c':
      click_allowed = false;
      break;
    case 'd':
      display = false;
      break;
    case 'i':
      ipath = optarg_;
      break;
    case 'o':
      opath = optarg_;
      break;
    case 'h':
      usage(argv[0], NULL, ipath, opath, user);
      return false;
      break;

    default:
      usage(argv[0], optarg_, ipath, opath, user);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath, opath, user);
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
    std::string opt_opath;
    std::string ipath;
    std::string opath;
    std::string filename;
    std::string username;
    bool opt_click_allowed = true;
    bool opt_display = true;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty())
      ipath = env_ipath;

// Set the default output path
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
    opt_opath = "/tmp";
#elif defined(_WIN32)
    opt_opath = "C:\\temp";
#endif

    // Get the user login name
    vpIoTools::getUserName(username);

    // Read the command line options
    if (getOptions(argc, argv, opt_ipath, opt_opath, opt_click_allowed, username, opt_display) == false) {
      exit(-1);
    }

    // Get the option values
    if (!opt_ipath.empty())
      ipath = opt_ipath;
    if (!opt_opath.empty())
      opath = opt_opath;

    // Append to the output path string, the login name of the user
    std::string odirname = vpIoTools::createFilePath(opath, username);

    // Test if the output path exist. If no try to create it
    if (vpIoTools::checkDirectory(odirname) == false) {
      try {
        // Create the dirname
        vpIoTools::makeDirectory(odirname);
      } catch (...) {
        usage(argv[0], NULL, ipath, opath, username);
        std::cerr << std::endl << "ERROR:" << std::endl;
        std::cerr << "  Cannot create " << odirname << std::endl;
        std::cerr << "  Check your -o " << opath << " option " << std::endl;
        exit(-1);
      }
    }

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
      usage(argv[0], NULL, ipath, opath, username);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
                << "  environment variable to specify the location of the " << std::endl
                << "  image path where test images are located." << std::endl
                << std::endl;
      exit(-1);
    }

    // Create two color images
    vpImage<vpRGBa> I1, I2;
    vpImagePoint ip, ip1, ip2;

    try {
      // Load a grey image from the disk
      filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");
      vpImageIo::read(I1, filename);
    } catch (...) {
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Cannot read " << filename << std::endl;
      std::cerr << "  Check your -i " << ipath << " option " << std::endl
                << "  or VISP_INPUT_IMAGE_PATH environment variable." << std::endl;
      exit(-1);
    }
    try {
      // Load a color image from the disk
      filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");
      vpImageIo::read(I2, filename);
    } catch (...) {
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Cannot read " << filename << std::endl;
      std::cerr << "  Check your -i " << ipath << " option " << std::endl
                << "  or VISP_INPUT_IMAGE_PATH environment variable." << std::endl;
      exit(-1);
    }

    // For each image, open a X11 display
    vpDisplayX display1;
    vpDisplayX display2;

    if (opt_display) {
      // Attach image 1 to display 1
      display1.init(I1, 0, 0, "X11 Display 1...");
      // Attach image 2 to display 2
      display2.init(I2, 200, 200, "X11 Display 2...");
      // Display the images
      vpDisplay::display(I1);
      vpDisplay::display(I2);

      // In the first display, display in overlay horizontal red lines
      for (unsigned int i = 0; i < I1.getHeight(); i += 20) {
        ip1.set_i(i);
        ip1.set_j(0);
        ip2.set_i(i);
        ip2.set_j(I1.getWidth());
        vpDisplay::displayLine(I1, ip1, ip2, vpColor::red);
      }

      // In the first display, display in overlay vertical green dot lines
      for (unsigned int i = 0; i < I1.getWidth(); i += 20) {
        ip1.set_i(0);
        ip1.set_j(i);
        ip2.set_i(I1.getWidth());
        ip2.set_j(i);
        vpDisplay::displayDotLine(I1, ip1, ip2, vpColor::green);
      }

      // In the first display, display in overlay a blue arrow
      ip1.set_i(0);
      ip1.set_j(0);
      ip2.set_i(100);
      ip2.set_j(100);
      vpDisplay::displayArrow(I1, ip1, ip2, vpColor::blue);

      // In the first display, display in overlay some circles. The
      // position of the center is 200, 200 the radius is increased by 20
      // pixels for each circle
      for (unsigned int i = 0; i < 100; i += 20) {
        ip.set_i(200);
        ip.set_j(200);
        vpDisplay::displayCircle(I1, ip, 20 + i, vpColor::yellow);
      }

      // In the first display, display in overlay a yellow string
      ip.set_i(100);
      ip.set_j(100);
      vpDisplay::displayText(I1, ip, "ViSP is a marvelous software", vpColor::blue);

      // Flush displays. The displays must be flushed to show the overlay.
      // without this line, nothing will be displayed.
      vpDisplay::flush(I1);
      vpDisplay::flush(I2);

      // If click is allowed, wait for a blocking mouse click in the first
      // display, to display a cross at the clicked pixel position
      if (opt_click_allowed) {
        std::cout << "\nA click in the first display to draw a cross..." << std::endl;
        // Blocking wait for a click. Get the position of the selected pixel
        // (i correspond to the row and j to the column coordinates in the
        // image)
        vpDisplay::getClick(I1, ip);
        // Display a red cross on the click pixel position
        std::cout << "Cross position: " << ip << std::endl;
        vpDisplay::displayCross(I1, ip, 15, vpColor::red);
        vpDisplay::flush(I1);
      } else {
        ip.set_i(50);
        ip.set_j(50);
        // Display a red cross at position ip in the first display
        std::cout << "Cross position: " << ip << std::endl;
        vpDisplay::displayCross(I1, ip, 15, vpColor::red);
        vpDisplay::flush(I1);
      }

      // Create a color image
      vpImage<vpRGBa> Ioverlay;
      // Updates the color image with the original loaded image 1 and the
      // overlay
      vpDisplay::getImage(I1, Ioverlay);

      // Write the color image on the disk
      filename = vpIoTools::createFilePath(odirname, "Klimt_grey.overlay.ppm");
      vpImageIo::write(Ioverlay, filename);

      // If click is allowed, wait for a mouse click to close the display
      if (opt_click_allowed) {
        std::cout << "\nA click in the second display to close the windows "
                     "and exit..."
                  << std::endl;
        // Wait for a blocking mouse click
        vpDisplay::getClick(I2);
      }
    }
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
#else
int main() {
  std::cout << "You do not have X11 functionalities to display images..." << std::endl;
  std::cout << "Tip if you are on a unix-like system:" << std::endl;
  std::cout << "- Install X11, configure again ViSP using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
}
#endif
