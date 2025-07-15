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
  \file grabDiskFloat.cpp

  \brief Example of depth map sequence reading from the disk using vpDiskGrabber
  class. Depth map sequence can result from an acquisition using a depth camera.

  The image sequence consists in successive depth maps. Several formats are supported,
  such as PFM, EXR if you have either TinyEXR or OpenCV, TIFF if you have OpenCV or
  NPY if you have MINIZ.
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
#define GETOPTARGS "df:g:hi:l:s:z:"

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
void usage(const char *name, const char *badparam, std::string ipath, std::string genericname, long int first,
           long int last, long int step, unsigned int nzero)
{
  fprintf(stdout, "\n\
Read a sequence of depth maps from the disk. Display it using X11, GDI,\n\
GTK-2 or OpenCV. The sequence is made of separate depth maps.\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] \n\
   [-f <first frame>] [-g <generic name>] [-l <last image> [-s <step>] \n\
   [-z <number of zero>] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                     %s\n\
     Set image input path. The sequence will be looked for in this folder.\n\
\n\
  -g <generic name>                                         %s\n\
     Specify the generic name of the files.\n\
     A generic name of file is for example myfile_%%04d.npy\n\
     Supported formats ar: pfm, exr, npy or tiff.\n\
     - pfm, exr and npy formats are supported natively\n\
     - tiff format is only supported if ViSP is build with\n\
       OpenCV support.\n\
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
     Print the help.\n\n", ipath.c_str(), genericname.c_str(), first, last, step, nzero);

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!
  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath : Input image path.
  \param genericname : Generic name for the images.
  \param first : First image number to read.
  \param last : Last images to read.
  \param step : Step between two successive images to read.
  \param nzero : Number of zero for the image number coding.
  \param display : Display activation.

  \return false if the program has to be stopped, true otherwise.
*/
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &genericname, long &first,
                long &last, long &step, unsigned int &nzero, bool &display)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'd':
      display = false;
      break;
    case 'f':
      first = atol(optarg_);
      break;
    case 'g':
      genericname = optarg_;
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
      usage(argv[0], nullptr, ipath, genericname, first, last, step, nzero);
      return false;

    default:
      usage(argv[0], optarg_, ipath, genericname, first, last, step, nzero);
      return false;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], nullptr, ipath, genericname, first, last, step, nzero);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

/**
 * \brief Convert a depth map into a displayable gray-shade-encoded image.
 *
 * \param[in] Idepth The depth map.
 * \param[in] Idisp The gray-shade-encoded image.
 */
void convertDepthImageToDisplayImage(const vpImage<float> &Idepth, vpImage<unsigned char> &Idisp)
{
  float max, min;
  Idepth.getMinMaxValue(min, max);
  Idisp.resize(Idepth.getHeight(), Idepth.getWidth());
  float a = 255. / (min - max);
  float b = 255. - a * min;
  int size = Idepth.getSize();
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < size; ++i) {
    Idisp.bitmap[i] = a * Idepth.bitmap[i] + b;
  }
}

/*!
  \example grabDiskFloat.cpp

  Example of depth map sequence reading from the disk using vpDiskGrabber
  class. Depth map sequence can result from an acquisition using a depth camera.

  The image sequence consists in successive depth maps. Several formats are supported,
  such as PFM, EXR if you have either TinyEXR or OpenCV, TIFF if you have OpenCV or
  NPY if you have MINIZ.

  This example can only work with visp-images > 3.6.0
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
    std::string opt_genericname = "mbt-depth/castel/castel/depth_image_%04d.pfm";

    bool opt_display = true;

    long int opt_first = 0;
    long int opt_last = 29;
    long int opt_step = 1;
    unsigned int opt_nzero = 4;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty())
      ipath = env_ipath;

    // Read the command line options
    if (getOptions(argc, argv, opt_ipath, opt_genericname, opt_first, opt_last, opt_step, opt_nzero,
                   opt_display) == false) {
      return EXIT_FAILURE;
    }

    // Get the option values
    if (!opt_ipath.empty()) {
      ipath = opt_ipath;
    }
    else {
#if defined(VISP_HAVE_DATASET)
#if VISP_HAVE_DATASET_VERSION < 0x030701
      std::cout << "This example requires visp-images 3.7.1 or a more recent version..." << std::endl;
      return EXIT_SUCCESS;
#endif
#endif
    }

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

    // Declare an image, this is a gray level image (unsigned char)
    // it size is not defined yet, it will be defined when the image will
    // read on the disk
    vpImage<float> I;
    vpImage<unsigned char> Idisp;

    // Declare a framegrabber able to read a sequence of successive
    // images from the disk
    vpDiskGrabber g;

    if (opt_genericname.empty()) {
      throw(vpException(vpException::notInitialized, "A generic name was given to the program"));
    }
    else {
      if (!ipath.empty()) {
        // Set the path to the directory containing the sequence
        opt_genericname = vpIoTools::createFilePath(ipath, opt_genericname);
      }
      std::cout << "Sequence that will be read:= " << opt_genericname << std::endl;
      g.setGenericName(opt_genericname);
    }
    // Set the step between two images of the sequence
    g.setStep(opt_step);
    // Set the number of digits to build the image number
    g.setNumberOfZero(opt_nzero);
    // Set the first frame number of the sequence
    g.setImageNumber(opt_first);

    // Open the framegrabber by loading the first image of the sequence
    g.open(I);
    Idisp.init(I.getHeight(), I.getWidth());
    convertDepthImageToDisplayImage(I, Idisp);
    std::cout << "Image size: width : " << I.getWidth() << " height: " << I.getHeight() << std::endl;

    // We open a window using either of the display library.
    // Its size is automatically defined by the image (I) size
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    std::shared_ptr<vpDisplay> display = vpDisplayFactory::createDisplay();
#else
    vpDisplay *display = vpDisplayFactory::allocateDisplay();
#endif

    if (opt_display) {
      display->init(Idisp, 100, 100, "Disk Framegrabber");

      // display the image
      // The image class has a member that specify a pointer toward
      // the display that has been initialized in the display declaration
      // therefore is is no longer necessary to make a reference to the
      // display variable.
      vpDisplay::display(Idisp);
      vpDisplay::flush(Idisp);
    }

    // this is the loop over the image sequence
    while (g.getImageNumber() < opt_last) {
      double tms = vpTime::measureTimeMs();
      // read the image and then increment the image counter so that the next
      // call to acquire(I) will get the next image
      g.acquire(I);
      if (opt_display) {
        convertDepthImageToDisplayImage(I, Idisp);
        // Display the image
        vpDisplay::display(Idisp);
        // Flush the display
        vpDisplay::flush(Idisp);
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
