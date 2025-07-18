/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
  \file grabDisk.cpp

  \brief Example of image sequence reading from the disk using vpDiskGrabber class.

  The sequence is made of successive images.
*/

#include <stdlib.h>
#include <iostream>
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_DISPLAY)

#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpDiskGrabber.h>
#include <visp3/io/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS "b:de:f:hi:l:s:z:"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

/*!
  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath : Input image path.
  \param basename : Input image base name.
  \param ext : Input image extension.
  \param first : First image number to read.
  \param last : Number of images to read.
  \param step : Step between two successive images to read.
  \param nzero : Number of zero for the image number coding.
 */
void usage(const char *name, const char *badparam, const std::string &ipath, const std::string &basename,
           const std::string &ext, long first, long last, long step, unsigned int nzero)
{
  fprintf(stdout, "\n\
Read an image sequence from the disk. Display it using X11 or GTK.\n\
The sequence is made of separate images. Each image corresponds\n\
to a PGM file.\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] [-b <base name>] [-e <extension>] \n\
   [-f <first frame>] [-l <last image> [-s <step>] \n\
   [-z <number of zero>] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                     %s\n\
     Set image input path.\n\
     From this path read \"cube/image.%%04d.%s\"\n\
     images.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
\n\
  -b <base name>                                            %s\n\
     Specify the base name of the files of the sequence\n\
     containing the images to process. \n\
     By image sequence, we mean one file per image.\n\
     The format is selected by analyzing the filename extension.\n\
\n\
  -e <extension>                                            %s\n\
     Specify the extension of the files.\n\
     Not taken into account for the moment. Will be a\n\
     future feature...\n\
\n\
  -f <first frame>                                          %ld\n\
     First frame number of the sequence.\n\
\n\
  -l <last image>                                           %ld\n\
     Last frame number of the sequence.\n\
\n\
  -s <step>                                                 %ld\n\
     Step between two images.\n\
\n\
  -z <number of zero>                                       %u\n\
     Number of digits to encode the image number.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -h \n\
     Print the help.\n\n",
          ipath.c_str(), ext.c_str(), basename.c_str(), ext.c_str(), first, last, step, nzero);

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}
/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath : Input image path.
  \param basename : Input image base name.
  \param ext : Input image extension.
  \param first : First image number to read.
  \param last : Last images to read.
  \param step : Step between two successive images to read.
  \param nzero : Number of zero for the image number coding.
  \param display : Display activation.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &basename, std::string &ext, long &first,
                long &last, long &step, unsigned int &nzero, bool &display)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'b':
      basename = optarg_;
      break;
    case 'd':
      display = false;
      break;
    case 'e':
      ext = optarg_;
      break;
    case 'f':
      first = atol(optarg_);
      break;
    case 'i':
      ipath = optarg_;
      break;
    case 'l':
      last = std::atol(optarg_);
      break;
    case 's':
      step = atol(optarg_);
      break;
    case 'z':
      nzero = static_cast<unsigned int>(atoi(optarg_));
      break;
    case 'h':
      usage(argv[0], nullptr, ipath, basename, ext, first, last, step, nzero);
      return false;

    default:
      usage(argv[0], optarg_, ipath, basename, ext, first, last, step, nzero);
      return false;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], nullptr, ipath, basename, ext, first, last, step, nzero);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

/*!
  \example grabDisk.cpp

  Example of image sequence reading from the disk using vpDiskGrabber class.

  Read an image sequence from the disk. The sequence is made of separate
  images. Each image corresponds to a PGM file. Display these images using X11
  or GTK.
*/
int main(int argc, const char **argv)
{
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  vpDisplay *display = nullptr;
#endif
  try {
    std::string env_ipath;
    std::string opt_ipath;
    std::string ipath;
    std::string opt_basename = "cube/image.";
#if defined(VISP_HAVE_DATASET)
#if VISP_HAVE_DATASET_VERSION >= 0x030600
    std::string opt_ext("png");
#else
    std::string opt_ext("pgm");
#endif
#else
    // We suppose that the user will download a recent dataset
    std::string opt_ext("png");
#endif

    bool opt_display = true;

    long opt_first = 5;
    long opt_last = 70;
    long opt_step = 1;
    unsigned int opt_nzero = 4;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty())
      ipath = env_ipath;

    // Read the command line options
    if (getOptions(argc, argv, opt_ipath, opt_basename, opt_ext, opt_first, opt_last, opt_step, opt_nzero,
                   opt_display) == false) {
      return EXIT_FAILURE;
    }

    // Get the option values
    if (!opt_ipath.empty())
      ipath = opt_ipath;

    // Compare ipath and env_ipath. If they differ, we take into account
    // the input path coming from the command line option
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
      usage(argv[0], nullptr, ipath, opt_basename, opt_ext, opt_first, opt_last, opt_step, opt_nzero);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
        << "  environment variable to specify the location of the " << std::endl
        << "  image path where test images are located." << std::endl
        << std::endl;
      return EXIT_FAILURE;
    }

    // Declare an image, this is a gray level image (unsigned char)
    // it size is not defined yet, it will be defined when the image will
    // read on the disk
    vpImage<unsigned char> I;

    // Declare a framegrabber able to read a sequence of successive
    // images from the disk
    vpDiskGrabber g;

    // Set the path to the directory containing the sequence
    g.setDirectory(ipath.c_str());
    // Set the image base name. The directory and the base name constitute
    // the constant part of the full filename
    g.setBaseName(opt_basename.c_str());
    // Set the step between two images of the sequence
    g.setStep(opt_step);
    // Set the number of digits to build the image number
    g.setNumberOfZero(opt_nzero);
    // Set the first frame number of the sequence
    g.setImageNumber(opt_first);
    // Set the image extension
    g.setExtension(opt_ext.c_str());

    // Open the framegrabber by loading the first image of the sequence
    g.open(I);

    std::cout << "Image size: width : " << I.getWidth() << " height: " << I.getHeight() << std::endl;

    // We open a window using either of the display library.
    // Its size is automatically defined by the image (I) size
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    std::shared_ptr<vpDisplay> display = vpDisplayFactory::createDisplay(I);
#else
    vpDisplay *display = vpDisplayFactory::allocateDisplay(I);
#endif

    if (opt_display) {
      display->init(I, 100, 100, "Disk Framegrabber");

      // display the image
      // The image class has a member that specify a pointer toward
      // the display that has been initialized in the display declaration
      // therefore is is no longer necessary to make a reference to the
      // display variable.
      vpDisplay::display(I);
      vpDisplay::flush(I);
    }

    // this is the loop over the image sequence
    while (g.getImageNumber() < opt_last) {
      double tms = vpTime::measureTimeMs();
      // read the image and then increment the image counter so that the next
      // call to acquire(I) will get the next image
      g.acquire(I);
      if (opt_display) {
        // Display the image
        vpDisplay::display(I);
        // Flush the display
        vpDisplay::flush(I);
      }
      // Synchronise the loop to 40 ms
      vpTime::wait(tms, 40);
    }
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
    if (display != nullptr) {
      delete display;
  }
#endif
    return EXIT_SUCCESS;
}
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
    if (display != nullptr) {
      delete display;
  }
#endif
    return EXIT_FAILURE;
  }
}

#else
int main()
{
  std::cout << "You do not have X11, or GDI (Graphical Device Interface) functionalities to display images..."
    << std::endl;
  std::cout << "Tip if you are on a unix-like system:" << std::endl;
  std::cout << "- Install X11, configure again ViSP using cmake and build again this example" << std::endl;
  std::cout << "Tip if you are on a windows-like system:" << std::endl;
  std::cout << "- Install GDI, configure again ViSP using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
}
#endif
