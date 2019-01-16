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
 * Example of Histogram manipulation.
 *
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file histogram.cpp

  \brief Histogram manipulation.
*/

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpHistogram.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>

#include <iomanip>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

// List of allowed command line options
#define GETOPTARGS "i:o:h"

/*!
  \example histogram.cpp

  Read a B&W image on the disk and compute the histogram.

*/

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
  %s [-i <input image path>] [-o <output histogram path>]\n\
     [-h]\n\
", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read \"Klimt/Klimt.pgm\"\n\
     image.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
\n\
  -o <output histogram path>                           %s\n\
     From this directory, creates the \"%s\"\n\
     subdirectory depending on the username, where \n\
     \"histogram.txt\" is saved.\n\
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
  \param user : Username.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &opath, const std::string &user)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
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

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty())
      ipath = env_ipath;

// Set the default output path
#if defined(_WIN32)
    opt_opath = "C:/temp";
#else
    opt_opath = "/tmp";
#endif

    // Get the user login name
    vpIoTools::getUserName(username);

    // Read the command line options
    if (getOptions(argc, argv, opt_ipath, opt_opath, username) == false) {
      exit(-1);
    }

    // Get the option values
    if (!opt_ipath.empty())
      ipath = opt_ipath;
    if (!opt_opath.empty())
      opath = opt_opath;

    // Append to the output path string, the login name of the user
    std::string dirname = vpIoTools::createFilePath(opath, username);

    // Test if the output path exist. If no try to create it
    if (vpIoTools::checkDirectory(dirname) == false) {
      try {
        // Create the dirname
        vpIoTools::makeDirectory(dirname);
      } catch (...) {
        usage(argv[0], NULL, ipath, opath, username);
        std::cerr << std::endl << "ERROR:" << std::endl;
        std::cerr << "  Cannot create " << dirname << std::endl;
        std::cerr << "  Check your -o " << opath << " option " << std::endl;
        exit(-1);
      }
    }

    // Compare ipath and env_ipath. If they differ, we take into account
    // the input path comming from the command line option
    if (opt_ipath.empty()) {
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

    // Create a grey level image
    vpImage<unsigned char> I;

    // Load a grey image from the disk
    filename = ipath;
    if (opt_ipath.empty())
      filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");

    std::cout << "Read: " << filename << std::endl;
    vpImageIo::read(I, filename);

    unsigned char distance = 60;
    vpHistogram h;

    // Computes the histogram from the image
    h.calculate(I);

    // Save the histogram
    filename = dirname + vpIoTools::path("/histogram.txt");
    std::cout << "Save the histogram in: " << filename << std::endl;
    h.write(filename);

    // Smooth the histogram
    h.smooth();
    // Save the histogram
    filename = dirname + vpIoTools::path("/histogram_smoothed.txt");
    std::cout << "Save the smoothed histogram in: " << filename << std::endl;
    h.write(filename);

    std::list<vpHistogramPeak> peaks;
    unsigned int nbpeaks = 0;

    // get all the histogram peaks
    nbpeaks = h.getPeaks(peaks);

    vpTRACE("List of peaks");
    vpTRACE("Nb peaks: %d", nbpeaks);
    if (nbpeaks) {
      for (std::list<vpHistogramPeak>::const_iterator it = peaks.begin(); it != peaks.end(); ++it) {
        vpHistogramPeak p = *it;
        vpTRACE("Peak: gray level: %d value: %d", p.getLevel(), p.getValue());
      }
    }

    // sort all the histogram peaks list to have the highest peak at the
    // beginning of the list, the smallest at the end.
    nbpeaks = h.sort(peaks);

    vpTRACE("Sorted list of peaks");
    vpTRACE("Nb peaks: %d", nbpeaks);
    if (nbpeaks) {
      for (std::list<vpHistogramPeak>::const_iterator it = peaks.begin(); it != peaks.end(); ++it) {
        vpHistogramPeak p = *it;
        vpTRACE("Peak: gray level: %d value: %d", p.getLevel(), p.getValue());
      }
    }

    // Get the two highest histogram peaks. peak1 is the highest
    vpHistogramPeak peak1, peak2;
    nbpeaks = h.getPeaks(distance, peak1, peak2);
    if (nbpeaks != 2) {
      std::cout << "Not a bimodal histogram..." << std::endl;
    } else {
      vpTRACE("Bimodal histogram: main peak1: %d-%d second peak2: %d-%d", peak1.getLevel(), peak1.getValue(),
              peak2.getLevel(), peak2.getValue());
    }

    // Get the valey between the two highest peaks
    vpHistogramValey valey;
    if (h.getValey(peak1, peak2, valey) == false) {
      vpTRACE("No valey found...");
    } else {
      vpTRACE("Valey: %d-%d", valey.getLevel(), valey.getValue());
    }

    vpHistogramValey valeyl, valeyr;

    {
      // Search the two valeys around peak1
      unsigned ret = h.getValey(distance, peak1, valeyl, valeyr);
      if (ret == 0x00) {
        vpTRACE("No left and right valey for peak %d-%d...", peak1.getLevel(), peak1.getValue());
      } else if (ret == 0x10) {
        vpTRACE("No right valey for peak %d-%d...", peak1.getLevel(), peak1.getValue());
        vpTRACE("Left valey: %d-%d", valeyl.getLevel(), valeyl.getValue());
      } else if (ret == 0x01) {
        vpTRACE("No left valey for peak %d-%d...", peak1.getLevel(), peak1.getValue());
        vpTRACE("Right valey: %d-%d", valeyr.getLevel(), valeyr.getValue());
      } else if (ret == 0x11) {
        vpTRACE("Left valey: %d-%d", valeyl.getLevel(), valeyl.getValue());
        vpTRACE("Right valey: %d-%d", valeyr.getLevel(), valeyr.getValue());
      }
    }
    {
      // Search the two valeys around peak2
      unsigned ret = h.getValey(distance, peak2, valeyl, valeyr);
      if (ret == 0x00) {
        vpTRACE("No left and right valey for peak %d-%d...", peak2.getLevel(), peak2.getValue());
      } else if (ret == 0x10) {
        vpTRACE("No right valey for peak %d-%d...", peak2.getLevel(), peak2.getValue());
        vpTRACE("Left valey: %d-%d", valeyl.getLevel(), valeyl.getValue());
      } else if (ret == 0x01) {
        vpTRACE("No left valey for peak %d-%d...", peak2.getLevel(), peak2.getValue());
        vpTRACE("Right valey: %d-%d", valeyr.getLevel(), valeyr.getValue());
      } else if (ret == 0x11) {
        vpTRACE("Left valey: %d-%d", valeyl.getLevel(), valeyl.getValue());
        vpTRACE("Right valey: %d-%d", valeyr.getLevel(), valeyr.getValue());
      }
    }

    ////////////////////////////////////////////////////////////
    // get the valey between the two highest peaks. Here we don't know
    // which of peakl or peakr is the highest.
    vpHistogramPeak peakl, peakr;
    if (h.getPeaks(distance, peakl, peakr, valey) == false) {
      std::cout << "Not a bimodal histogram..." << std::endl;
    } else {
      vpTRACE("Bimodal histogram: valey %d-%d for peakl: %d-%d peakr: %d-%d", valey.getLevel(), valey.getValue(),
              peakl.getLevel(), peakl.getValue(), peakr.getLevel(), peakr.getValue());
    }
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
