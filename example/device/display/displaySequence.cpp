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
 * Read an image sequence from the disk and display it.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 * Anthony Saunier
 *
 *****************************************************************************/
/*!
  \file displaySequence.cpp

  \brief Read an image sequence from the disk and display it.

  The sequence is made of separate images. Each image corresponds to a
  PGM file.

*/

#include <iomanip>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpParseArgv.h>
#if (defined(VISP_HAVE_GTK) || defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))

#include <visp3/core/vpImage.h>
#include <visp3/io/vpImageIo.h>

#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayX.h>

#include <visp3/core/vpTime.h>

/*!
  \example displaySequence.cpp

  Read an image sequence from the disk and display it.

  The sequence is made of separate images. Each image corresponds to a
  PGM file.

*/

// List of allowed command line options
#define GETOPTARGS "di:p:hf:n:s:w"

void usage(const char *name, const char *badparam, std::string ipath, std::string ppath, unsigned first,
           unsigned nimages, unsigned step);
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &ppath, unsigned &first, unsigned &nimages,
                unsigned &step, bool &display, bool &wait);

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath : Input image path.
  \param ppath : Personal image path.
  \param first : First image.
  \param nimages : Number of images to manipulate.
  \param step : Step between two images.

 */
void usage(const char *name, const char *badparam, std::string ipath, std::string ppath, unsigned first,
           unsigned nimages, unsigned step)
{
  fprintf(stdout, "\n\
Read an image sequence from the disk and display it.\n\
The sequence is made of separate images. Each image corresponds\n\
to a PGM file.\n\
\n\
SYNOPSIS\n\
  %s [-i <test image path>] [-p <personal image path>]\n\
     [-f <first image>] [-n <number of images>] [-s <step>] \n\
     [-w] [-d] [-h]\n						      \
 ", name);

  fprintf(stdout, "\n\
 OPTIONS:                                               Default\n\
  -i <test image path>                                %s\n\
     Set image input path.\n\
     From this path read \"cube/image.%%04d.pgm\"\n\
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
     The following image file formats PNM (PGM P5, PPM P6)\n\
     are supported. The format is selected by analysing \n\
     the filename extension.\n\
     Example : \"/Temp/ViSP-images/cube/image.%%04d.pgm\"\n\
     %%04d is for the image numbering.\n\
 \n\
  -f <first image>                                     %u\n\
     First image number of the sequence.\n\
 \n\
  -n <number of images>                                %u\n\
     Number of images to load from the sequence.\n\
 \n\
  -s <step>                                            %u\n\
     Step between two images.\n\
\n\
  -d                                             \n\
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
     Print the help.\n\n", ipath.c_str(), ppath.c_str(), first, nimages, step);

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
  \param nimages : Number of images to display.
  \param step : Step between two images.
  \param display : Set as true, activates the image display. This is
  the default configuration. When set to false, the display is
  disabled. This can be useful for automatic tests using crontab
  under Unix or using the task manager under Windows.

  \param wait : Boolean for waiting a mouse click between two images.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &ppath, unsigned &first, unsigned &nimages,
                unsigned &step, bool &display, bool &wait)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
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
      first = (unsigned)atoi(optarg_);
      break;
    case 'n':
      nimages = (unsigned)atoi(optarg_);
      break;
    case 's':
      step = (unsigned)atoi(optarg_);
      break;
    case 'w':
      wait = true;
      break;
    case 'h':
      usage(argv[0], NULL, ipath, ppath, first, nimages, step);
      return false;
      break;

    default:
      usage(argv[0], optarg_, ipath, ppath, first, nimages, step);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath, ppath, first, nimages, step);
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
    std::string dirname;
    std::string filename;
    unsigned opt_first = 0;
    unsigned opt_nimages = 80;
    unsigned opt_step = 1;
    bool opt_display = true;
    bool opt_wait = false;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty())
      ipath = env_ipath;

    // Read the command line options
    if (getOptions(argc, argv, opt_ipath, opt_ppath, opt_first, opt_nimages, opt_step, opt_display, opt_wait) ==
        false) {
      exit(-1);
    }

    if (!opt_display)
      opt_wait = false; // turn off the waiting

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
      usage(argv[0], NULL, ipath, opt_ppath, opt_first, opt_nimages, opt_step);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
                << "  environment variable to specify the location of the " << std::endl
                << "  image path where test images are located." << std::endl
                << "  Use -p <personal image path> option if you want to " << std::endl
                << "  use personal images." << std::endl
                << std::endl;

      exit(-1);
    }

    // Declare an image, this is a gray level image (unsigned char)
    // it size is not defined yet, it will be defined when the image will
    // read on the disk
    vpImage<unsigned char> I;

    unsigned iter = opt_first;
    std::ostringstream s;
    char cfilename[FILENAME_MAX];

    if (opt_ppath.empty()) {

      // Warning :
      // the image sequence is not provided with the ViSP package
      // therefore the program will return you an error :
      //  !!    vpImageIoPnm.cpp: readPGM(#210) :couldn't read file
      //        ViSP-images/cube/image.0001.pgm
      //  !!    vpDotExample.cpp: main(#95) :Error while reading the image
      //  terminate called after throwing an instance of 'vpImageException'
      //
      //  The sequence is available on the visp www site
      //  https://visp.inria.fr/download/
      //  in the download section. It is named "ViSP-images.tar.gz"

      // Set the path location of the image sequence
      dirname = vpIoTools::createFilePath(ipath, "cube");

      // Build the name of the image file

      s.setf(std::ios::right, std::ios::adjustfield);
      s << "image." << std::setw(4) << std::setfill('0') << iter << ".pgm";
      filename = vpIoTools::createFilePath(dirname, s.str());
    } else {
      sprintf(cfilename, opt_ppath.c_str(), iter);
      filename = cfilename;
    }
    // Read the PGM image named "filename" on the disk, and put the
    // bitmap into the image structure I.  I is initialized to the
    // correct size
    //
    // exception readPGM may throw various exception if, for example,
    // the file does not exist, or if the memory cannot be allocated
    try {
      vpImageIo::read(I, filename);
    } catch (...) {
      // an exception is throwned if an exception from readPGM has been
      // catched here this will result in the end of the program Note that
      // another error message has been printed from readPGM to give more
      // information about the error
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Cannot read " << filename << std::endl;
      std::cerr << "  Check your -i " << ipath << " option, " << std::endl
                << "  or your -p " << opt_ppath << " option " << std::endl
                << "  or VISP_INPUT_IMAGE_PATH environment variable" << std::endl;
      exit(-1);
    }

#if defined VISP_HAVE_GTK
    vpDisplayGTK display;
#elif defined VISP_HAVE_X11
    vpDisplayX display;
#elif defined VISP_HAVE_GDI
    vpDisplayGDI display;
#endif
    if (opt_display) {

      // We open a window using either X11 or GTK or GDI.
      // Its size is automatically defined by the image (I) size
      display.init(I, 100, 100, "Display...");

      // Display the image
      // The image class has a member that specify a pointer toward
      // the display that has been initialized in the display declaration
      // therefore is is no longuer necessary to make a reference to the
      // display variable.
      vpDisplay::display(I);
      vpDisplay::flush(I);
    }

    //  double tms_1 = vpTime::measureTimeMs() ;
    unsigned niter = 0;
    // this is the loop over the image sequence
    while (iter < opt_first + opt_nimages * opt_step) {
      double tms = vpTime::measureTimeMs();

      // set the new image name

      if (opt_ppath.empty()) {
        s.str("");
        s << "image." << std::setw(4) << std::setfill('0') << iter << ".pgm";
        filename = vpIoTools::createFilePath(dirname, s.str());
      } else {
        sprintf(cfilename, opt_ppath.c_str(), iter);
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
      }
      if (opt_wait) {
        std::cout << "A click in the image to continue..." << std::endl;
        // Wait for a blocking mouse click
        vpDisplay::getClick(I);
      } else {
        // Synchronise the loop to 40 ms
        vpTime::wait(tms, 40);
      }
      niter++;

      iter += opt_step;
    }
    //  double tms_2 = vpTime::measureTimeMs() ;
    //  double tms_total = tms_2 - tms_1 ;
    //  std::cout << "Total Time : "<< tms_total<<std::endl;
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
#else
int main() {
  std::cout << "You do not have X11, or GDI (Graphical Device Interface), or GTK functionalities to display images..." << std::endl;
  std::cout << "Tip if you are on a unix-like system:" << std::endl;
  std::cout << "- Install X11, configure again ViSP using cmake and build again this example" << std::endl;
  std::cout << "Tip if you are on a windows-like system:" << std::endl;
  std::cout << "- Install GDI, configure again ViSP using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
}
#endif
