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
 * Reading an image sequence.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file imageSequenceReader.cpp
  \brief   reading an image sequence using vpVideoReader class.
 */

/*!
  \example imageSequenceReader.cpp
  Reading an image sequence using vpVideoReader class.
 */

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/io/vpVideoReader.h>

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_GTK)

// List of allowed command line options
#define GETOPTARGS "cdi:p:f:h"

void usage(const char *name, const char *badparam, std::string ipath, std::string ppath);
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &ppath, int &first, bool &click_allowed,
                bool &display);

/*!

Print the program options.

\param name : Program name.
\param badparam : Bad parameter name.
\param ipath : Input image sequence path.
\param ppath : Personal image sequence path.

 */
void usage(const char *name, const char *badparam, std::string ipath, std::string ppath)
{
  fprintf(stdout, "\n\
Read an image sequence on the disk.\n\
\n\
SYNOPSIS\n\
  %s [-i <input images path>] [-p <personal image sequence path>]\n\
     [-c][-d][-h]\n						      \
", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input images path>                                %s\n\
     Set ViSP-images input path.\n\
     From this path read \"cube/image.%%04d.pgm\"\n\
     images.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
\n\
  -p <personal image sequence path>                             %s\n\
     Specify a personal folder containing an image sequence \n\
     to process.\n\
     Example : \"/Temp/ViSP-images/cube/image.%%04d.pgm\"\n\
     %%04d is for the image numbering.\n\
\n\
  -f <index of the first frame>                             \n\
     Specify the first image index.\n\
\n\
  -c\n\
     Disable the mouse click. Useful to automaze the \n\
     execution of this program without humain intervention.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -h\n\
     Print the help.\n\n", ipath.c_str(), ppath.c_str());

  if (badparam) {
    fprintf(stderr, "ERROR: \n");
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);
  }
}
/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath : Input images path.
  \param ppath : Personal image sequence path.
  \param first : first frame index.
  \param click_allowed : Mouse click activation.
  \param display : Display activation.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &ppath, int &first, bool &click_allowed,
                bool &display)
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
    case 'p':
      ppath = optarg_;
      break;
    case 'f':
      first = atoi(optarg_);
      break;
    case 'h':
      usage(argv[0], NULL, ipath, ppath);
      return false;
      break;

    default:
      usage(argv[0], optarg_, ipath, ppath);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath, ppath);
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
    std::string ipath;
    std::string opt_ppath;
    std::string filename;
    int opt_first = 1;
    bool opt_click_allowed = true;
    bool opt_display = true;

    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << "  videoImageSequenceReader.cpp" << std::endl << std::endl;

    std::cout << "  reading an image sequence" << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << std::endl;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty())
      ipath = env_ipath;

    // Read the command line options
    if (getOptions(argc, argv, opt_ipath, opt_ppath, opt_first, opt_click_allowed, opt_display) == false) {
      exit(-1);
    }

    // Get the option values
    if (!opt_ipath.empty())
      ipath = opt_ipath;

    // Compare ipath and env_ipath. If they differ, we take into account
    // the input path comming from the command line option
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
      usage(argv[0], NULL, ipath, opt_ppath);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
                << "  environment variable to specify the location of the " << std::endl
                << "  video path where test images are located." << std::endl
                << std::endl;
      exit(-1);
    }

    /////////////////////////////////////////////////////////////////////

    // vpImage is a template class you can declare vpImage of ...
    // everything...
    vpImage<vpRGBa> I;

    // Create the video Reader
    vpVideoReader reader;

    if (opt_ppath.empty()) {
      filename = vpIoTools::createFilePath(ipath, "mire-2/image.%04d.pgm");
    } else {
      filename.assign(opt_ppath);
    }

    // Initialize the reader and get the first frame.
    reader.setFileName(filename);
    reader.setFirstFrameIndex(opt_first);
    reader.open(I);
    std::cout << "Current image number: " << reader.getFrameIndex() << std::endl;
    if ((opt_first) != reader.getFrameIndex()) {
      std::cout << "Unable to get requested image number: " << opt_first << std::endl;
      return EXIT_FAILURE;
    }

// We open a window using either X11, GTK, GDI or OpenCV.
#if defined VISP_HAVE_X11
    vpDisplayX display;
#elif defined VISP_HAVE_GTK
    vpDisplayGTK display;
#elif defined VISP_HAVE_GDI
    vpDisplayGDI display;
#elif defined VISP_HAVE_OPENCV
    vpDisplayOpenCV display;
#endif

    if (opt_display) {
      // Display size is automatically defined by the image (I) size
      display.init(I, 100, 100, "Display video frame");
      vpDisplay::display(I);
      vpDisplay::flush(I);
    }

    if (opt_display && opt_click_allowed) {
      std::cout << "Click in the image to read and display the second frame" << std::endl;
      vpDisplay::getClick(I);
    }

    reader.getFrame(I, opt_first + 1);
    std::cout << "Current image number (should be " << opt_first + 1 << "): " << reader.getFrameIndex() << std::endl;
    if ((opt_first + 1) != reader.getFrameIndex()) {
      std::cout << "Unable to get requested image number: " << opt_first + 1 << std::endl;
      return EXIT_FAILURE;
    }

    if (opt_display) {
      vpDisplay::display(I);
      vpDisplay::flush(I);
    }

    if (opt_display && opt_click_allowed) {
      std::cout << "Click on the image to read and display the last frame" << std::endl;
      vpDisplay::getClick(I);
    }

    reader.getFrame(I, reader.getLastFrameIndex());
    std::cout << "Current image number (should be " << reader.getLastFrameIndex() << "): " << reader.getFrameIndex()
              << std::endl;

    if (opt_display) {
      vpDisplay::display(I);
      vpDisplay::flush(I);
    }

    if (opt_display && opt_click_allowed) {
      std::cout << "Click to see the video" << std::endl;
      vpDisplay::getClick(I);
    }

    int lastFrame = reader.getLastFrameIndex();

    for (int i = opt_first; i <= lastFrame; i++) {
      reader.getFrame(I, i);
      std::cout << "Current image number: " << reader.getFrameIndex() << std::endl;
      if (opt_display) {
        vpDisplay::display(I);
        vpDisplay::flush(I);
      }
    }

    if (opt_display && opt_click_allowed) {
      std::cout << "Click to exit the test" << std::endl;
      vpDisplay::getClick(I);
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
  std::cout << "Sorry, no display is available. We quit this example." << std::endl;
  return EXIT_SUCCESS;
}
#endif
