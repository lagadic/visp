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
 * Test for image undistortion.
 *
 * Authors:
 * Anthony Saunier
 *
 *****************************************************************************/

#include <stdlib.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/core/vpTime.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>
/*!
  \example testUndistortImage.cpp

  \brief Undistort an image.

  Read an image from the disk, undistort it and save the
  undistorted image on the disk.

 */

// List of allowed command line options
#define GETOPTARGS "cdi:o:t:s:h"

/*
  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath: Input image path.
  \param opath : Output image path.
  \param user : Username.
 */
void usage(const char *name, const char *badparam, const std::string &ipath, const std::string &opath,
           const std::string &user)
{
  fprintf(stdout, "\n\
Read an image from the disk, undistort it \n\
and save the undistorted image on the disk.\n\
(grid36-01_undistorted.pgm).\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] [-o <output image path>] [-t <nThreads>] [-s <scale>]\n\
     [-h]\n\
          ", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read \"calibration/grid36-01.pgm\"\n\
     image.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
     \n\
  -o <output image path>                               %s\n\
     Set image output path.\n\
     From this directory, creates the \"%s\"\n\
     subdirectory depending on the username, where \n\
     grid36-01_undistorted.pgm output image is written.\n\
\n\
  -t <nThreads>                                 \n\
     Set the number of threads to use for vpImageTools::undistort().\n\
\n\
  -s <scale>                                 \n\
     Resize the image by the specified scale factor.\n\
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
  \param nThreads : Nb threads for vpImageTools::undistort().
  \param scale : Scale factor to resize the image.
  \return false if the program has to be stopped, true otherwise.
 */
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &opath,
                const std::string &user, unsigned int &nThreads, unsigned int &scale)
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
    case 't':
      nThreads = atoi(optarg_);
      break;
    case 's':
      scale = atoi(optarg_);
      break;
    case 'h':
      usage(argv[0], NULL, ipath, opath, user);
      return false;

    case 'c':
    case 'd':
      break;

    default:
      usage(argv[0], optarg_, ipath, opath, user);
      return false;
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
    unsigned int nThreads = 2;
    unsigned int scale = 1;

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
    if (getOptions(argc, argv, opt_ipath, opt_opath, username, nThreads, scale) == false) {
      exit(-1);
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
      usage(argv[0], NULL, ipath, opt_opath, username);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
                << "  environment variable to specify the location of the " << std::endl
                << "  image path where test images are located." << std::endl
                << std::endl;
      exit(-1);
    }

//
// Here starts really the test
//
    vpImage<vpRGBa> I, I_; // Input image
    vpImage<unsigned char> I_gray;
    vpImage<vpRGBa> U; // undistorted output image
    vpImage<unsigned char> U_gray; // undistorted output image
    vpImage<vpRGBa> U_remap; // undistorted output image
    vpImage<unsigned char> U_remap_gray; // undistorted output image

    vpCameraParameters cam;
    cam.initPersProjWithDistortion(600, 600, 320, 240, -0.17, 0.17);
    // Read the input grey image from the disk
    filename = vpIoTools::createFilePath(ipath, "calibration/grid36-01.pgm");
    std::cout << "Read image: " << filename << std::endl;
    vpImageIo::read(I_, filename);
    if (scale > 1) {
      std::cout << "Scale the image by a factor of " << scale << std::endl;
      vpImageTools::resize(I_, I, I_.getWidth()*scale, I_.getHeight()*scale);
    } else {
      I = I_;
    }
    std::cout << "Input image: " << I.getWidth() << "x" << I.getHeight() << std::endl;
    vpImageConvert::convert(I, I_gray);

    std::cout << "Nb threads to use for vpImageTools::undistort(): " << nThreads << std::endl;

    double t_undistort = 0.0, t_remap = 0.0;
    {
      std::cout << "\nUndistortion in process (color image)... " << std::endl;
      vpImageTools::undistort(I, cam, U, nThreads);

      double begintime = vpTime::measureTimeMs();

      // For the test, to have a significant time measure we repeat the
      // undistortion 10 times
      for (unsigned int i = 0; i < 10; i++)
        // Create the undistorted image
        vpImageTools::undistort(I, cam, U, nThreads);

      double endtime = vpTime::measureTimeMs();
      t_undistort = endtime - begintime;

      std::cout << "Time for 10 color image undistortion (ms): " << t_undistort << std::endl;
    }

    {
      std::cout << "Undistortion in process with remap (color image)... " << std::endl;

      double begintime = vpTime::measureTimeMs();

      // For the test, to have a significant time measure we repeat the
      // undistortion 10 times
      vpArray2D<int> mapU, mapV;
      vpArray2D<float> mapDu, mapDv;
      for (unsigned int i = 0; i < 10; i++) {
        if (i == 0) {
          vpImageTools::initUndistortMap(cam, I.getWidth(), I.getHeight(), mapU, mapV, mapDu, mapDv);
        }
        vpImageTools::remap(I, mapU, mapV, mapDu, mapDv, U_remap);
      }

      double endtime = vpTime::measureTimeMs();
      t_remap = endtime - begintime;

      std::cout << "Time for 10 color image undistortion with remap (ms): " << t_remap << std::endl;
      std::cout << "Speed-up: " << t_undistort / t_remap << "X" << std::endl;
    }

    {
      std::cout << "\nUndistortion in process (gray image)... " << std::endl;
      vpImageTools::undistort(I_gray, cam, U_gray, nThreads);

      double begintime = vpTime::measureTimeMs();

      // For the test, to have a significant time measure we repeat the
      // undistortion 100 times
      for (unsigned int i = 0; i < 100; i++)
        // Create the undistorted image
        vpImageTools::undistort(I_gray, cam, U_gray, nThreads);

      double endtime = vpTime::measureTimeMs();
      t_undistort = endtime - begintime;

      std::cout << "Time for 100 gray image undistortion (ms): " << t_undistort << std::endl;
    }

    {
      std::cout << "Undistortion in process with remap (gray image)... " << std::endl;

      double begintime = vpTime::measureTimeMs();

      // For the test, to have a significant time measure we repeat the
      // undistortion 100 times
      vpArray2D<int> mapU, mapV;
      vpArray2D<float> mapDu, mapDv;
      for (unsigned int i = 0; i < 10; i++) {
        if (i == 0) {
          vpImageTools::initUndistortMap(cam, I.getWidth(), I.getHeight(), mapU, mapV, mapDu, mapDv);
        }
        vpImageTools::remap(I_gray, mapU, mapV, mapDu, mapDv, U_remap_gray);
      }

      double endtime = vpTime::measureTimeMs();
      t_remap = endtime - begintime;

      std::cout << "Time for 100 gray image undistortion with remap (ms): " << t_remap << std::endl;
      std::cout << "Speed-up: " << t_undistort / t_remap << "X" << std::endl;
    }

    // Write the undistorted images on the disk
    filename = vpIoTools::path(vpIoTools::createFilePath(opath, "grid36-01_undistorted.ppm"));
    std::cout << "\nWrite undistorted image: " << filename << std::endl;
    vpImageIo::write(U, filename);

    filename = vpIoTools::path(vpIoTools::createFilePath(opath, "grid36-01_undistorted.pgm"));
    std::cout << "Write undistorted image: " << filename << std::endl;
    vpImageIo::write(U_gray, filename);

    filename = vpIoTools::path(vpIoTools::createFilePath(opath, "grid36-01_undistorted_remap.ppm"));
    std::cout << "\nWrite undistorted image with remap: " << filename << std::endl;
    vpImageIo::write(U_remap, filename);

    filename = vpIoTools::path(vpIoTools::createFilePath(opath, "grid36-01_undistorted_remap.pgm"));
    std::cout << "Write undistorted image with remap: " << filename << std::endl;
    vpImageIo::write(U_remap_gray, filename);

    // Write the undistorted difference images on the disk
    vpImage<vpRGBa> U_diff;
    vpImage<unsigned char> U_diff_gray;
    vpImageTools::imageDifference(U, U_remap, U_diff);
    filename = vpIoTools::path(vpIoTools::createFilePath(opath, "grid36-01_undistorted_diff.ppm"));
    std::cout << "\nWrite undistorted image: " << filename << std::endl;
    vpImageIo::write(U_diff, filename);

    vpImageTools::imageDifference(U_gray, U_remap_gray, U_diff_gray);
    filename = vpIoTools::path(vpIoTools::createFilePath(opath, "grid36-01_undistorted_diff.pgm"));
    std::cout << "Write undistorted image: " << filename << std::endl;
    vpImageIo::write(U_diff_gray, filename);

    return 0;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}
