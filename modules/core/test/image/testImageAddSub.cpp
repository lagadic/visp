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
 * Test images addition / substraction.
 *
 * Author:
 * Souriya Trinh
 *
 *****************************************************************************/

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>

/*!
  \example testImageAddSub.cpp

  \brief Test images addition / substraction.
*/

// List of allowed command line options
#define GETOPTARGS "cdi:o:n:h"

/*
  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath: Input image path.
  \param opath : Output image path.
  \param user : Username.
  \param nbiter : Number of benchmark iterations.
 */
void usage(const char *name, const char *badparam, const std::string &ipath, const std::string &opath,
           const std::string &user, int nbiter)
{
  fprintf(stdout, "\n\
Test images addition / substraction.\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] [-o <output image path>] [-n <nb iterations>]\n\
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
     result output images are written.\n\
\n\
  -n <nb iterations>                                   %d\n\
     Set the number of benchmark iterations.\n\
\n\
  -h\n\
     Print the help.\n\n", ipath.c_str(), opath.c_str(), user.c_str(), nbiter);

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
  \param nbiter : Number of benchmark iterations.
  \return false if the program has to be stopped, true otherwise.
*/
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &opath, const std::string &user,
                int &nbiter)
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
    case 'n':
      nbiter = atoi(optarg_);
      break;
    case 'h':
      usage(argv[0], NULL, ipath, opath, user, nbiter);
      return false;
      break;

    case 'c':
    case 'd':
      break;

    default:
      usage(argv[0], optarg_, ipath, opath, user, nbiter);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath, opath, user, nbiter);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

void regularImageAdd(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2, vpImage<unsigned char> &Ires,
                     const bool saturate)
{
  if ((I1.getHeight() != I2.getHeight()) || (I1.getWidth() != I2.getWidth())) {
    throw(vpException(vpException::dimensionError, "The two images do not have the same size"));
  }

  if ((I1.getHeight() != Ires.getHeight()) || (I1.getWidth() != Ires.getWidth())) {
    Ires.resize(I1.getHeight(), I1.getWidth());
  }

  unsigned char *ptr_I1 = I1.bitmap;
  unsigned char *ptr_I2 = I2.bitmap;
  unsigned char *ptr_Ires = Ires.bitmap;

  for (unsigned int cpt = 0; cpt < Ires.getSize(); cpt++, ++ptr_I1, ++ptr_I2, ++ptr_Ires) {
    *ptr_Ires = saturate ? vpMath::saturate<unsigned char>((short int)*ptr_I1 + (short int)*ptr_I2) : *ptr_I1 + *ptr_I2;
  }
}

void regularImageSubtract(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                          vpImage<unsigned char> &Ires, const bool saturate)
{
  if ((I1.getHeight() != I2.getHeight()) || (I1.getWidth() != I2.getWidth())) {
    throw(vpException(vpException::dimensionError, "The two images do not have the same size"));
  }

  if ((I1.getHeight() != Ires.getHeight()) || (I1.getWidth() != Ires.getWidth())) {
    Ires.resize(I1.getHeight(), I1.getWidth());
  }

  unsigned char *ptr_I1 = I1.bitmap;
  unsigned char *ptr_I2 = I2.bitmap;
  unsigned char *ptr_Ires = Ires.bitmap;

  for (unsigned int cpt = 0; cpt < Ires.getSize(); cpt++, ++ptr_I1, ++ptr_I2, ++ptr_Ires) {
    *ptr_Ires = saturate ? vpMath::saturate<unsigned char>((short int)*ptr_I1 - (short int)*ptr_I2) : *ptr_I1 - *ptr_I2;
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
    int nbIterations = 100;

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
    if (getOptions(argc, argv, opt_ipath, opt_opath, username, nbIterations) == false) {
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
        usage(argv[0], NULL, ipath, opt_opath, username, nbIterations);
        std::cerr << std::endl << "ERROR:" << std::endl;
        std::cerr << "  Cannot create " << opath << std::endl;
        std::cerr << "  Check your -o " << opt_opath << " option " << std::endl;
        exit(EXIT_FAILURE);
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
      usage(argv[0], NULL, ipath, opt_opath, username, nbIterations);
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
    vpImage<unsigned char> I; // Input image

    // Read the input grey image from the disk
    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");
    std::cout << "Read image: " << filename << std::endl << std::endl;
    vpImageIo::read(I, filename);

    // Test add image
    vpImage<unsigned char> Inull(I.getHeight(), I.getWidth(), 0);
    vpImage<unsigned char> Iadd;
    vpImageTools::imageAdd(I, Inull, Iadd);
    if (Iadd != I) {
      throw vpException(vpException::fatalError, "Problem with vpImageTools::imageAdd (Iadd != I)!");
    } else {
      std::cout << "(Iadd == I)? " << (Iadd == I) << std::endl;
    }

    // Test in-place add
    Iadd = 0;
    vpImageTools::imageAdd(I, Iadd, Iadd);
    if (Iadd != I) {
      throw vpException(vpException::fatalError, "Problem with in-place vpImageTools::imageAdd (Iadd != I)!");
    } else {
      std::cout << "In-place (Iadd == I)? " << (Iadd == I) << std::endl;
    }

    // Test subtract image
    Inull = 0;
    vpImage<unsigned char> Isub;
    vpImageTools::imageSubtract(I, Inull, Isub);
    if (Isub != I) {
      throw vpException(vpException::fatalError, "Problem with vpImageTools::imageSubtract (Iadd != I)!");
    } else {
      std::cout << "(Isub == I)? " << (Isub == I) << std::endl;
    }

    // Test in-place subtract
    Isub = 0;
    vpImageTools::imageSubtract(I, Isub, Isub);
    if (Isub != I) {
      throw vpException(vpException::fatalError, "Problem with in-place vpImageTools::imageSubtract (Isub != I)!");
    } else {
      std::cout << "In-place (Isub == I)? " << (Isub == I) << std::endl;
    }

    // Test add image saturation
    vpImage<unsigned char> I2(I.getHeight(), I.getWidth());
    for (unsigned int cpt = 0; cpt < I2.getSize(); cpt++) {
      I2.bitmap[cpt] = (unsigned char)cpt;
    }

    // No saturation
    vpImageTools::imageAdd(I, I2, Iadd, false);
    vpImage<unsigned char> Iadd_regular;
    regularImageAdd(I, I2, Iadd_regular, false);
    if (Iadd != Iadd_regular) {
      throw vpException(vpException::fatalError, "Problem with vpImageTools::imageAdd(I, I2, Iadd, "
                                                 "false) (Iadd != Iadd_regular)!");
    } else {
      std::cout << "\nNo saturation (Iadd == Iadd_regular)? " << (Iadd == Iadd_regular) << std::endl;
    }

    // Saturation
    vpImageTools::imageAdd(I, I2, Iadd, true);
    regularImageAdd(I, I2, Iadd_regular, true);
    if (Iadd != Iadd_regular) {
      throw vpException(vpException::fatalError, "Problem with vpImageTools::imageAdd(I, I2, Iadd, "
                                                 "true) (Iadd != Iadd_regular)!");
    } else {
      std::cout << "Saturation (Iadd == Iadd_regular)? " << (Iadd == Iadd_regular) << std::endl;
    }

    // Test subtract image saturation
    // No saturation
    vpImageTools::imageSubtract(I, I2, Isub, false);
    vpImage<unsigned char> Isub_regular;
    regularImageSubtract(I, I2, Isub_regular, false);
    if (Isub != Isub_regular) {
      throw vpException(vpException::fatalError, "Problem with vpImageTools::imageSubtract(I, I2, "
                                                 "Isub, false) (Isub != Isub_regular)!");
    } else {
      std::cout << "\nNo saturation (Isub == Isub_regular)? " << (Isub == Isub_regular) << std::endl;
    }

    // Saturation
    vpImageTools::imageSubtract(I, I2, Isub, true);
    regularImageSubtract(I, I2, Isub_regular, true);
    if (Isub != Isub_regular) {
      throw vpException(vpException::fatalError, "Problem with vpImageTools::imageSubtract(I, I2, "
                                                 "Isub, true) (Isub != Isub_regular)!");
    } else {
      std::cout << "Saturation (Isub == Isub_regular)? " << (Isub == Isub_regular) << std::endl;
    }

    // Benchmark
    // Benchmark add no saturation
    Iadd = 0;
    double t_sse = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      vpImageTools::imageAdd(I, Iadd, Iadd, false);
    }
    t_sse = vpTime::measureTimeMs() - t_sse;
    std::cout << "\nAdd no saturation ; t_sse (" << nbIterations << " iterations)=" << t_sse << " ms" << std::endl;

    Iadd_regular = 0;
    double t = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      regularImageAdd(I, Iadd_regular, Iadd_regular, false);
    }
    t = vpTime::measureTimeMs() - t;
    std::cout << "Add regular no saturation ; t (" << nbIterations << " iterations)=" << t << " ms"
              << " ; Speed-up: " << (t / t_sse) << "X" << std::endl;
    std::cout << "(Iadd == Iadd_regular)? " << (Iadd == Iadd_regular) << std::endl;

    // Write add no saturation
    filename = vpIoTools::createFilePath(opath, "Klimt_add_no_sat_sse.pgm");
    std::cout << "\nWrite: " << filename << std::endl;
    vpImageIo::write(Iadd, filename);

    filename = vpIoTools::createFilePath(opath, "Klimt_add_no_sat.pgm");
    std::cout << "Write: " << filename << std::endl;
    vpImageIo::write(Iadd_regular, filename);

    // Benchmark add saturation
    Iadd = 0;
    t_sse = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      vpImageTools::imageAdd(I, Iadd, Iadd, true);
    }
    t_sse = vpTime::measureTimeMs() - t_sse;
    std::cout << "\nAdd saturation ; t_sse (" << nbIterations << " iterations)=" << t_sse << " ms" << std::endl;

    Iadd_regular = 0;
    t = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      regularImageAdd(I, Iadd_regular, Iadd_regular, true);
    }
    t = vpTime::measureTimeMs() - t;
    std::cout << "Add saturation ; t (" << nbIterations << " iterations)=" << t << " ms"
              << " ; Speed-up: " << (t / t_sse) << "X" << std::endl;
    std::cout << "(Iadd == Iadd_regular)? " << (Iadd == Iadd_regular) << std::endl;

    // Write add no saturation
    filename = vpIoTools::createFilePath(opath, "Klimt_add_sat_sse.pgm");
    std::cout << "\nWrite: " << filename << std::endl;
    vpImageIo::write(Iadd, filename);

    filename = vpIoTools::createFilePath(opath, "Klimt_add_sat.pgm");
    std::cout << "Write: " << filename << std::endl;
    vpImageIo::write(Iadd_regular, filename);

    // Benchmark subtract no saturation
    Isub = I2;
    t_sse = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      vpImageTools::imageSubtract(I, Isub, Isub, false);
    }
    t_sse = vpTime::measureTimeMs() - t_sse;
    std::cout << "\nSubtract no saturation ; t_sse (" << nbIterations << " iterations)=" << t_sse << " ms" << std::endl;

    Isub_regular = I2;
    t = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      regularImageSubtract(I, Isub_regular, Isub_regular, false);
    }
    t = vpTime::measureTimeMs() - t;
    std::cout << "Subtract regular no saturation ; t (" << nbIterations << " iterations)=" << t << " ms"
              << " ; Speed-up: " << (t / t_sse) << "X" << std::endl;
    std::cout << "(Isub == Isub_regular)? " << (Isub == Isub_regular) << std::endl;

    // Write subtract no saturation
    filename = vpIoTools::createFilePath(opath, "Klimt_subtract_no_sat_sse.pgm");
    std::cout << "\nWrite: " << filename << std::endl;
    vpImageIo::write(Isub, filename);

    filename = vpIoTools::createFilePath(opath, "Klimt_subtract_no_sat.pgm");
    std::cout << "Write: " << filename << std::endl;
    vpImageIo::write(Isub_regular, filename);

    // Benchmark subtract saturation
    Isub = I2;
    t_sse = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      vpImageTools::imageSubtract(I, Isub, Isub, true);
    }
    t_sse = vpTime::measureTimeMs() - t_sse;
    std::cout << "\nSubtract saturation ; t_sse (" << nbIterations << " iterations)=" << t_sse << " ms" << std::endl;

    Isub_regular = I2;
    t = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      regularImageSubtract(I, Isub_regular, Isub_regular, true);
    }
    t = vpTime::measureTimeMs() - t;
    std::cout << "Subtract saturation ; t (" << nbIterations << " iterations)=" << t << " ms"
              << " ; Speed-up: " << (t / t_sse) << "X" << std::endl;
    std::cout << "(Isub == Isub_regular)? " << (Isub == Isub_regular) << std::endl;

    // Write subtract no saturation
    filename = vpIoTools::createFilePath(opath, "Klimt_subtract_sat_sse.pgm");
    std::cout << "\nWrite: " << filename << std::endl;
    vpImageIo::write(Isub, filename);

    filename = vpIoTools::createFilePath(opath, "Klimt_subtract_sat.pgm");
    std::cout << "Write: " << filename << std::endl;
    vpImageIo::write(Isub_regular, filename);

    // Invert Klimt with image cropped
    if (I.getWidth() >= 411 && I.getHeight() >= 507) {
      vpRect r_crop(0, 0, 411, 507);
      vpImage<unsigned char> I_crop;
      vpImageTools::crop(I, r_crop, I_crop);
      std::cout << "\nI_crop=" << I_crop.getWidth() << "x" << I_crop.getHeight() << std::endl;

      vpImage<unsigned char> I_invert(I_crop.getHeight(), I_crop.getWidth(), 255);
      vpImageTools::imageSubtract(I_invert, I_crop, I_invert);

      vpImage<unsigned char> I_invert_regular(I_crop.getHeight(), I_crop.getWidth(), 255);
      regularImageSubtract(I_invert_regular, I_crop, I_invert_regular, false);
      std::cout << "(I_invert == I_invert_regular)? " << (I_invert == I_invert_regular) << std::endl;

      vpImage<unsigned char> I_white(I_crop.getHeight(), I_crop.getWidth(), 255);
      vpImage<unsigned char> I_invert2 = I_invert;
      vpImageTools::imageAdd(I_invert2, I_crop, I_invert2);
      std::cout << "(I_invert2 == I_white)? " << (I_invert2 == I_white) << std::endl;

      filename = vpIoTools::createFilePath(opath, "Klimt_invert_crop.pgm");
      std::cout << "Write: " << filename << std::endl;
      vpImageIo::write(I_invert, filename);
    }

    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
