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
 * Test flood fill algorithm.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

#include <iomanip>

#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/imgproc/vpImgproc.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>

/*!
  \example testFloodFill.cpp

  \brief Test flood fill algorithm.
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
Test flood fill algorithm.\n\
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

void printImage(const vpImage<unsigned char> &I, const std::string &name)
{
  std::cout << "\n" << name << ":" << std::endl;

  std::cout << "   ";
  for (unsigned int j = 0; j < I.getWidth(); j++) {
    std::cout << std::setfill(' ') << std::setw(2) << j << " ";
  }
  std::cout << std::endl;

  std::cout << std::setfill(' ') << std::setw(3) << "+";
  for (unsigned int j = 0; j < I.getWidth(); j++) {
    std::cout << std::setw(3) << "---";
  }
  std::cout << std::endl;

  for (unsigned int i = 0; i < I.getHeight(); i++) {
    std::cout << std::setfill(' ') << std::setw(2) << i << "|";

    for (unsigned int j = 0; j < I.getWidth(); j++) {
      std::cout << std::setfill(' ') << std::setw(2) << static_cast<unsigned int>(I[i][j]) << " ";
    }

    std::cout << std::endl;
  }
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

    unsigned char image_data[8 * 8] = {1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0,
                                       1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1,
                                       1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0};
    vpImage<unsigned char> I_test_flood_fill_4_connexity(image_data, 8, 8, true);
    vpImage<unsigned char> I_test_flood_fill_8_connexity = I_test_flood_fill_4_connexity;
    printImage(I_test_flood_fill_4_connexity, "Test image data");

    unsigned char image_data_check_4_connexity[8 * 8] = {
        1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0,
        1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0};
    vpImage<unsigned char> I_check_4_connexity(image_data_check_4_connexity, 8, 8, true);

    unsigned char image_data_check_8_connexity[8 * 8] = {
        1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0};
    vpImage<unsigned char> I_check_8_connexity(image_data_check_8_connexity, 8, 8, true);

    // Test flood fill on test data 4-connexity
    vp::floodFill(I_test_flood_fill_4_connexity, vpImagePoint(2, 2), 0, 1, vpImageMorphology::CONNEXITY_4);
    printImage(I_test_flood_fill_4_connexity, "I_test_flood_fill_4_connexity");

    if (I_test_flood_fill_4_connexity != I_check_4_connexity) {
      throw vpException(vpException::fatalError, "Problem with vp::floodFill() and 4-connexity!");
    }
    std::cout << "\n(I_test_flood_fill_4_connexity == I_check_4_connexity)? "
              << (I_test_flood_fill_4_connexity == I_check_4_connexity) << std::endl;

    // Test flood fill on test data 8-connexity
    vp::floodFill(I_test_flood_fill_8_connexity, vpImagePoint(2, 2), 0, 1, vpImageMorphology::CONNEXITY_8);
    printImage(I_test_flood_fill_8_connexity, "I_test_flood_fill_8_connexity");

    if (I_test_flood_fill_8_connexity != I_check_8_connexity) {
      throw vpException(vpException::fatalError, "Problem with vp::floodFill() and 8-connexity!");
    }
    std::cout << "\n(I_test_flood_fill_8_connexity == I_check_8_connexity)? "
              << (I_test_flood_fill_8_connexity == I_check_8_connexity) << std::endl;

    // Read Klimt.ppm
    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");
    vpImage<unsigned char> I_klimt;
    vpImageIo::read(I_klimt, filename);
    std::cout << "\nRead image: " << filename << " (" << I_klimt.getWidth() << "x" << I_klimt.getHeight() << ")"
              << std::endl
              << std::endl;
    vpImageTools::binarise(I_klimt, (unsigned char)127, (unsigned char)255, (unsigned char)0, (unsigned char)255,
                           (unsigned char)255);

    int seed_x = 0;
    int seed_y = 0;

    vpImage<unsigned char> I_klimt_flood_fill_4_connexity = I_klimt;
    double t = vpTime::measureTimeMs();
    vp::floodFill(I_klimt_flood_fill_4_connexity, vpImagePoint(seed_y, seed_x), 0, 255, vpImageMorphology::CONNEXITY_4);
    t = vpTime::measureTimeMs() - t;
    std::cout << "Flood fill on Klimt image (4-connexity): " << t << " ms" << std::endl;

    filename = vpIoTools::createFilePath(opath, "Klimt_flood_fill_4_connexity.pgm");
    vpImageIo::write(I_klimt_flood_fill_4_connexity, filename);

    vpImage<unsigned char> I_klimt_flood_fill_8_connexity = I_klimt;
    t = vpTime::measureTimeMs();
    vp::floodFill(I_klimt_flood_fill_8_connexity, vpImagePoint(seed_y, seed_x), 0, 255, vpImageMorphology::CONNEXITY_8);
    t = vpTime::measureTimeMs() - t;
    std::cout << "Flood fill on Klimt image (8-connexity): " << t << " ms" << std::endl;
    filename = vpIoTools::createFilePath(opath, "Klimt_flood_fill_8_connexity.pgm");
    vpImageIo::write(I_klimt_flood_fill_8_connexity, filename);

#if VISP_HAVE_OPENCV_VERSION >= 0x020408
    cv::Mat matImg_klimt_4_connexity, matImg_klimt_8_connexity;
    vpImageConvert::convert(I_klimt, matImg_klimt_4_connexity);
    vpImageConvert::convert(I_klimt, matImg_klimt_8_connexity);

    // 4-connexity
    t = vpTime::measureTimeMs();
    cv::floodFill(matImg_klimt_4_connexity, cv::Point(seed_x, seed_y), cv::Scalar(255), 0, cv::Scalar(), cv::Scalar(),
                  4);
    t = vpTime::measureTimeMs() - t;
    std::cout << "OpenCV flood fill on Klimt image (4-connexity): " << t << " ms" << std::endl;

    vpImage<unsigned char> I_klimt_flood_fill_4_connexity_check;
    vpImageConvert::convert(matImg_klimt_4_connexity, I_klimt_flood_fill_4_connexity_check);

    filename = vpIoTools::createFilePath(opath, "Klimt_flood_fill_4_connexity_opencv.pgm");
    vpImageIo::write(I_klimt_flood_fill_4_connexity_check, filename);

    // 8-connexity
    t = vpTime::measureTimeMs();
    cv::floodFill(matImg_klimt_8_connexity, cv::Point(seed_x, seed_y), cv::Scalar(255), 0, cv::Scalar(), cv::Scalar(),
                  8);
    t = vpTime::measureTimeMs() - t;
    std::cout << "OpenCV flood fill on Klimt image (8-connexity): " << t << " ms" << std::endl;

    vpImage<unsigned char> I_klimt_flood_fill_8_connexity_check;
    vpImageConvert::convert(matImg_klimt_8_connexity, I_klimt_flood_fill_8_connexity_check);

    filename = vpIoTools::createFilePath(opath, "Klimt_flood_fill_8_connexity_opencv.pgm");
    vpImageIo::write(I_klimt_flood_fill_8_connexity_check, filename);

    // Check
    std::cout << "\n(I_klimt_flood_fill_4_connexity == "
                 "I_klimt_flood_fill_4_connexity_check)? "
              << (I_klimt_flood_fill_4_connexity == I_klimt_flood_fill_4_connexity_check) << std::endl;
    std::cout << "(I_klimt_flood_fill_8_connexity == "
                 "I_klimt_flood_fill_8_connexity_check)? "
              << (I_klimt_flood_fill_8_connexity == I_klimt_flood_fill_8_connexity_check) << std::endl;

    if (I_klimt_flood_fill_4_connexity != I_klimt_flood_fill_4_connexity_check) {
      throw vpException(vpException::fatalError, "(I_klimt_flood_fill_4_connexity != "
                                                 "I_klimt_flood_fill_4_connexity_check)");
    }
    if (I_klimt_flood_fill_8_connexity != I_klimt_flood_fill_8_connexity_check) {
      throw vpException(vpException::fatalError, "(I_klimt_flood_fill_8_connexity != "
                                                 "I_klimt_flood_fill_8_connexity_check)");
    }
#endif

    std::cout << "\nTest flood fill is ok!" << std::endl;
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
}
