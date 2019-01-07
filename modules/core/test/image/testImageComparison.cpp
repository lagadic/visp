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
 * Test the comparison of two vpImage objects of the same type.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/
/*!
  \example testImageComparison.cpp

  \brief Test the comparison of two vpImage objects of the same type.
*/

#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS "cdi:h"

void usage(const char *name, const char *badparam, std::string ipath);
bool getOptions(int argc, const char **argv, std::string &ipath);

/*
  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath: Input image path.
 */
void usage(const char *name, const char *badparam, std::string ipath)
{
  fprintf(stdout, "\n\
Test the comparison of two vpImage objects of the same type.\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>]\n\
     [-h]\n            \
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
  -h\n\
     Print the help.\n\n", ipath.c_str());

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!
  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath: Input image path.
  \return false if the program has to be stopped, true otherwise.
*/
bool getOptions(int argc, const char **argv, std::string &ipath)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'i':
      ipath = optarg_;
      break;
    case 'h':
      usage(argv[0], NULL, ipath);
      return false;
      break;

    case 'c':
    case 'd':
      break;

    default:
      usage(argv[0], optarg_, ipath);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath);
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
    std::string filename;
    std::string username;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty()) {
      ipath = env_ipath;
    }

    // Get the user login name
    vpIoTools::getUserName(username);

    // Read the command line options
    if (getOptions(argc, argv, opt_ipath) == false) {
      exit(-1);
    }

    // Get the option values
    if (!opt_ipath.empty()) {
      ipath = opt_ipath;
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
      usage(argv[0], NULL, ipath);
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

    // Load grayscale Klimt
    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");

    vpImage<unsigned char> I_Klimt1, I_Klimt2;
    vpImageIo::read(I_Klimt1, filename);
    vpImageIo::read(I_Klimt2, filename);

    std::cout << "\nI_Klimt1=" << I_Klimt1.getWidth() << "x" << I_Klimt1.getHeight() << std::endl;
    std::cout << "I_Klimt2=" << I_Klimt2.getWidth() << "x" << I_Klimt2.getHeight() << std::endl;

    std::cout << "\nThe two grayscale images are equal." << std::endl;
    std::cout << "(I_Klimt1 == I_Klimt2)=" << (I_Klimt1 == I_Klimt2) << std::endl;
    std::cout << "(I_Klimt1 != I_Klimt2)=" << (I_Klimt1 != I_Klimt2) << std::endl;

    // The two images should be equal
    if (!(I_Klimt1 == I_Klimt2) || (I_Klimt1 != I_Klimt2)) {
      std::stringstream ss;
      ss << "\nProblem when comparing two grayscale images!\n";
      ss << "(I_Klimt1 == I_Klimt2)=" << (I_Klimt1 == I_Klimt2) << std::endl;
      ss << "(I_Klimt1 != I_Klimt2)=" << (I_Klimt1 != I_Klimt2) << std::endl;

      throw vpException(vpException::fatalError, ss.str());
    }

    // Modify I_Klimt1
    if (I_Klimt1[I_Klimt1.getHeight() / 2][I_Klimt1.getWidth() / 2] < 255) {
      I_Klimt1[I_Klimt1.getHeight() / 2][I_Klimt1.getWidth() / 2]++;
    } else {
      I_Klimt1[I_Klimt1.getHeight() / 2][I_Klimt1.getWidth() / 2]--;
    }

    std::cout << "\nThe two grayscale images are different." << std::endl;
    std::cout << "(I_Klimt1 == I_Klimt2)=" << (I_Klimt1 == I_Klimt2) << std::endl;
    std::cout << "(I_Klimt1 != I_Klimt2)=" << (I_Klimt1 != I_Klimt2) << std::endl;

    // The two images should be different
    if ((I_Klimt1 == I_Klimt2) || !(I_Klimt1 != I_Klimt2)) {
      std::stringstream ss;
      ss << "\nProblem when comparing two grayscale images!\n";
      ss << "(I_Klimt1 == I_Klimt2)=" << (I_Klimt1 == I_Klimt2) << std::endl;
      ss << "(I_Klimt1 != I_Klimt2)=" << (I_Klimt1 != I_Klimt2) << std::endl;

      throw vpException(vpException::fatalError, ss.str());
    }

    // Load color Klimt
    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");

    vpImage<vpRGBa> I_color_Klimt1, I_color_Klimt2;
    vpImageIo::read(I_color_Klimt1, filename);
    I_color_Klimt2 = I_color_Klimt1;

    std::cout << "\nI_color_Klimt1=" << I_color_Klimt1.getWidth() << "x" << I_color_Klimt1.getHeight() << std::endl;
    std::cout << "I_color_Klimt2=" << I_color_Klimt2.getWidth() << "x" << I_color_Klimt2.getHeight() << std::endl;

    std::cout << "\nThe two color images are equal." << std::endl;
    std::cout << "(I_color_Klimt1 == I_color_Klimt2)=" << (I_color_Klimt1 == I_color_Klimt2) << std::endl;
    std::cout << "(I_color_Klimt1 != I_color_Klimt2)=" << (I_color_Klimt1 != I_color_Klimt2) << std::endl;

    // The two images should be equal
    if (!(I_color_Klimt1 == I_color_Klimt2) || (I_color_Klimt1 != I_color_Klimt2)) {
      std::stringstream ss;
      ss << "\nProblem when comparing two color images!\n";
      ss << "(I_color_Klimt1 == I_color_Klimt2)=" << (I_color_Klimt1 == I_color_Klimt2) << std::endl;
      ss << "(I_color_Klimt1 != I_color_Klimt2)=" << (I_color_Klimt1 != I_color_Klimt2) << std::endl;

      throw vpException(vpException::fatalError, ss.str());
    }

    // Modify I_color_Klimt2
    if (I_color_Klimt2[I_color_Klimt2.getHeight() / 2][I_color_Klimt2.getWidth() / 2].R < 255) {
      I_color_Klimt2[I_color_Klimt2.getHeight() / 2][I_color_Klimt2.getWidth() / 2].R++;
    } else {
      I_color_Klimt2[I_color_Klimt2.getHeight() / 2][I_color_Klimt2.getWidth() / 2].R--;
    }

    std::cout << "\nThe two color images are different." << std::endl;
    std::cout << "(I_color_Klimt1 == I_color_Klimt2)=" << (I_color_Klimt1 == I_color_Klimt2) << std::endl;
    std::cout << "(I_color_Klimt1 != I_color_Klimt2)=" << (I_color_Klimt1 != I_color_Klimt2) << std::endl;

    // The two images should be different
    if ((I_color_Klimt1 == I_color_Klimt2) || !(I_color_Klimt1 != I_color_Klimt2)) {
      std::stringstream ss;
      ss << "\nProblem when comparing two color images!\n";
      ss << "(I_color_Klimt1 == I_color_Klimt2)=" << (I_color_Klimt1 == I_color_Klimt2) << std::endl;
      ss << "(I_color_Klimt1 != I_color_Klimt2)=" << (I_color_Klimt1 != I_color_Klimt2) << std::endl;

      throw vpException(vpException::fatalError, ss.str());
    }

  } catch (const vpException &e) {
    std::cerr << "\nCatch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "\nThe comparison of two images of the same type is OK!" << std::endl;
  return EXIT_SUCCESS;
}
