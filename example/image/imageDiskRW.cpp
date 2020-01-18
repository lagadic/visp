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
 * Reading and writting images on the disk.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file imageDiskRW.cpp
  \brief   reading and writting of PPM image using vpImageIo class.

  Example of exception handling the exception :
  read an image that does not exist
  write in a directory that does no exist
 */

/*!
  \example imageDiskRW.cpp
  Reading and writting of PPM image using vpImageIo class.

  Example of exception handling the exception :
  read an image that does not exist,
  write in a directory that does no exist
 */

#include <stdio.h>
#include <stdlib.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>
// List of allowed command line options
#define GETOPTARGS "i:o:h"

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
Read and write PGM images on the disk. Also test exceptions.\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] [-o <output image path>]\n\
     [-h]\n						      \
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
     Klimt_grey.pgm output image is written.\n\
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

    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << "  imageDiskRW.cpp" << std::endl << std::endl;

    std::cout << "  reading and writting of PPM image" << std::endl;
    std::cout << "  read an image that does not exist" << std::endl;
    std::cout << "  write in a directory that does no exist" << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << std::endl;

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
    if (getOptions(argc, argv, opt_ipath, opt_opath, username) == false) {
      return EXIT_SUCCESS;
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
        return EXIT_FAILURE;
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
      return EXIT_SUCCESS;
    }

    /////////////////////////////////////////////////////////////////////

    // First we wanted to have gray level image (8bits)
    // vpImage is a template class you can declare vpImage of ...
    // everything...
    vpImage<unsigned char> I;

    // Although I is a gray level image you can read and write
    // color image. Obviously the color will be translated as a gray level

    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");
    vpImageIo::read(I, filename);

    filename = vpIoTools::createFilePath(dirname, "IoPPM.Klimt_char.ppm");
    vpImageIo::write(I, filename);

    // test io error
    // if the image you want to read on the disk does not exist
    // an exception is thrown
    // Try to load a non existing image
    try {
      filename = vpIoTools::createFilePath(ipath, "image-that-does-not-exist.ppm");
      vpImageIo::read(I, filename);
    } catch (const vpException &e) {
      std::cout << "Catch an expected exception: " << e << std::endl;
    }

    // same thing if you to write in a directory that does not exist
    // or where you are not allowd to write.
    try {
      filename = vpIoTools::createFilePath(dirname, "directory-that-does-not-exist/Klimt.ppm");
      vpImageIo::write(I, filename);
    } catch (const vpException &e) {
      std::cout << "Catch an expected exception: " << e << std::endl;
    }

    std::cout << "----------------------------------------------------" << std::endl;

    // Let's consider that the image is now a color image (32 bits RGBa)
    vpImage<vpRGBa> Irgba;

    // read write unsigned char ppm image.

    // Load a color image from the disk
    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");
    vpImageIo::read(Irgba, filename);

    // Write the content of the color image on the disk
    filename = vpIoTools::createFilePath(dirname, "IoPGM.Klimt_rgba.ppm");
    vpImageIo::write(Irgba, filename);

    // test io error
    try {
      filename = vpIoTools::createFilePath(ipath, "image-that-does-not-exist.ppm");
      vpImageIo::read(Irgba, filename);
    } catch (const vpException &e) {
      std::cout << "Catch an expected exception: " << e << std::endl;
    }

    // test io error
    try {
      filename = vpIoTools::createFilePath(dirname, "directory-that-does-not-exist/Klimt.ppm");
      vpImageIo::write(Irgba, filename);
    }
    catch (const vpException &e) {
      std::cout << "Catch an expected exception: " << e << std::endl;
    }
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an unexpected exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
