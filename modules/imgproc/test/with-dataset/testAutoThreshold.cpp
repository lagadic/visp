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
 * Test automatic thresholding.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/imgproc/vpImgproc.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>

/*!
  \example testAutoThreshold.cpp

  \brief Test automatic thresholding.
*/

// List of allowed command line options
#define GETOPTARGS "cdi:o:h"

void usage(const char *name, const char *badparam, std::string ipath, std::string opath, std::string user);
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &opath, std::string user);

/*
  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath: Input image path.
  \param opath : Output image path.
  \param user : Username.
 */
void usage(const char *name, const char *badparam, std::string ipath, std::string opath, std::string user)
{
  fprintf(stdout, "\n\
Test automatic thresholding.\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] [-o <output image path>]\n\
     [-h]\n                 \
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
  -o <output image path>                               %s\n\
     Set image output path.\n\
     From this directory, creates the \"%s\"\n\
     subdirectory depending on the username, where \n\
     output result images are written.\n\
\n\
  -h\n\
     Print the help.\n\n", ipath.c_str(), opath.c_str(), user.c_str());

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!
  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath: Input image path.
  \param opath : Output image path.
  \param user : Username.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &opath, std::string user)
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

    case 'c':
    case 'd':
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
      exit(EXIT_FAILURE);
    }

    // Get the option values
    if (!opt_ipath.empty())
      ipath = opt_ipath;
    if (!opt_opath.empty())
      opath = opt_opath;

    // Append to the output path string, the login name of the user
    opath = vpIoTools::createFilePath(opath, username);

    // Test if the output path exist. If no try to create it
    if (vpIoTools::checkDirectory(opath) == false) {
      try {
        // Create the dirname
        vpIoTools::makeDirectory(opath);
      } catch (...) {
        usage(argv[0], NULL, ipath, opt_opath, username);
        std::cerr << std::endl << "ERROR:" << std::endl;
        std::cerr << "  Cannot create " << opath << std::endl;
        std::cerr << "  Check your -o " << opt_opath << " option " << std::endl;
        exit(EXIT_FAILURE);
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
      usage(argv[0], NULL, ipath, opt_opath, username);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
                << "  environment variable to specify the location of the " << std::endl
                << "  image path where test images are located." << std::endl
                << std::endl;
      exit(EXIT_FAILURE);
    }

    //
    // Here starts really the test
    //

    filename = vpIoTools::createFilePath(ipath, "calibration/grid36-03.pgm");
    vpImage<unsigned char> I;
    vpImageIo::read(I, filename);
    std::cout << "Read: " << filename << " (" << I.getWidth() << "x" << I.getHeight() << ")" << std::endl;

    vpImage<unsigned char> I_thresh = I;

    // Huang
    double t = vpTime::measureTimeMs();
    double threshold = vp::autoThreshold(I_thresh, vp::AUTO_THRESHOLD_HUANG);
    t = vpTime::measureTimeMs() - t;
    std::cout << "\nAutomatic thresholding (Huang): " << threshold << " ; t=" << t << " ms" << std::endl;

    filename = vpIoTools::createFilePath(opath, "grid36-03_auto_thresh_huang.pgm");
    vpImageIo::write(I_thresh, filename);
    std::cout << "Write: " << filename << std::endl;

    // Intermodes
    I_thresh = I;
    t = vpTime::measureTimeMs();
    threshold = vp::autoThreshold(I_thresh, vp::AUTO_THRESHOLD_INTERMODES);
    t = vpTime::measureTimeMs() - t;
    std::cout << "\nAutomatic thresholding (Intermodes): " << threshold << " ; t=" << t << " ms" << std::endl;

    filename = vpIoTools::createFilePath(opath, "grid36-03_auto_thresh_intermodes.pgm");
    vpImageIo::write(I_thresh, filename);
    std::cout << "Write: " << filename << std::endl;

    // IsoData
    I_thresh = I;
    t = vpTime::measureTimeMs();
    threshold = vp::autoThreshold(I_thresh, vp::AUTO_THRESHOLD_ISODATA);
    t = vpTime::measureTimeMs() - t;
    std::cout << "\nAutomatic thresholding (IsoData): " << threshold << " ; t=" << t << " ms" << std::endl;

    filename = vpIoTools::createFilePath(opath, "grid36-03_auto_thresh_isodata.pgm");
    vpImageIo::write(I_thresh, filename);
    std::cout << "Write: " << filename << std::endl;

    // Mean
    I_thresh = I;
    t = vpTime::measureTimeMs();
    threshold = vp::autoThreshold(I_thresh, vp::AUTO_THRESHOLD_MEAN);
    t = vpTime::measureTimeMs() - t;
    std::cout << "\nAutomatic thresholding (Mean): " << threshold << " ; t=" << t << " ms" << std::endl;

    filename = vpIoTools::createFilePath(opath, "grid36-03_auto_thresh_mean.pgm");
    vpImageIo::write(I_thresh, filename);
    std::cout << "Write: " << filename << std::endl;

    // Otsu
    I_thresh = I;
    t = vpTime::measureTimeMs();
    threshold = vp::autoThreshold(I_thresh, vp::AUTO_THRESHOLD_OTSU);
    t = vpTime::measureTimeMs() - t;
    std::cout << "\nAutomatic thresholding (Otsu): " << threshold << " ; t=" << t << " ms" << std::endl;

    filename = vpIoTools::createFilePath(opath, "grid36-03_auto_thresh_otsu.pgm");
    vpImageIo::write(I_thresh, filename);
    std::cout << "Write: " << filename << std::endl;

    // Triangle
    I_thresh = I;
    t = vpTime::measureTimeMs();
    threshold = vp::autoThreshold(I_thresh, vp::AUTO_THRESHOLD_TRIANGLE);
    t = vpTime::measureTimeMs() - t;
    std::cout << "\nAutomatic thresholding (Triangle): " << threshold << " ; t=" << t << " ms" << std::endl;

    filename = vpIoTools::createFilePath(opath, "grid36-03_auto_thresh_Triangle.pgm");
    vpImageIo::write(I_thresh, filename);
    std::cout << "Write: " << filename << std::endl;

    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
}
