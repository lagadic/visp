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
 * Test imgproc functions.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

#include <cstdio>
#include <cstdlib>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/imgproc/vpImgproc.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>

/*!
  \example testImgproc.cpp

  \brief Test imgproc functions.
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
Test imgproc functions.\n\
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
     Klimt_grey.pgm output image is written.\n\
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
      return EXIT_FAILURE;
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
      usage(argv[0], NULL, ipath, opt_opath, username);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
                << "  environment variable to specify the location of the " << std::endl
                << "  image path where test images are located." << std::endl
                << std::endl;
      return EXIT_FAILURE;
    }

    //
    // Here starts really the test
    //

    //
    // Test color functions using Klimt.ppm
    //

    // Read Klimt.ppm
    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");
    vpImage<vpRGBa> I_color, Iinput_color;
    std::cout << "Read image: " << filename << std::endl;
    vpImageIo::read(Iinput_color, filename);
    Iinput_color.halfSizeImage(I_color);
    std::cout << "Image: " << I_color.getWidth() << "x" << I_color.getHeight() << std::endl;

    // Adjust
    double alpha = 1.5, beta = -10.0;
    vpImage<vpRGBa> I_color_adjust;
    double t = vpTime::measureTimeMs();
    vp::adjust(I_color, I_color_adjust, alpha, beta);
    t = vpTime::measureTimeMs() - t;
    std::cout << "Time to do color adjust: " << t << " ms" << std::endl;

    // Save adjust
    filename = vpIoTools::createFilePath(opath, "Klimt_adjust.ppm");
    vpImageIo::write(I_color_adjust, filename);

    // Equalize Histogram
    vpImage<vpRGBa> I_color_equalize_histogram;
    t = vpTime::measureTimeMs();
    vp::equalizeHistogram(I_color, I_color_equalize_histogram);
    t = vpTime::measureTimeMs() - t;
    std::cout << "Time to do color histogram equalization: " << t << " ms" << std::endl;

    // Save equalizeHistogram
    filename = vpIoTools::createFilePath(opath, "Klimt_equalize_histogram.ppm");
    vpImageIo::write(I_color_equalize_histogram, filename);

    // Gamma correction
    vpImage<vpRGBa> I_color_gamma_correction;
    double gamma = 2.2;
    t = vpTime::measureTimeMs();
    vp::gammaCorrection(I_color, I_color_gamma_correction, gamma);
    t = vpTime::measureTimeMs() - t;
    std::cout << "Time to do color gamma correction: " << t << " ms" << std::endl;

    // Save gammaCorrection
    filename = vpIoTools::createFilePath(opath, "Klimt_gamma_correction.ppm");
    vpImageIo::write(I_color_gamma_correction, filename);

    // Retinex
    vpImage<vpRGBa> I_color_retinex;
    t = vpTime::measureTimeMs();
    vp::retinex(I_color, I_color_retinex);
    t = vpTime::measureTimeMs() - t;
    std::cout << "Time to do color retinex: " << t << " ms" << std::endl;

    // Save retinex
    filename = vpIoTools::createFilePath(opath, "Klimt_retinex.ppm");
    vpImageIo::write(I_color_retinex, filename);

    // Stretch contrast
    vpImage<vpRGBa> I_color_stretch_contrast;
    t = vpTime::measureTimeMs();
    vp::stretchContrast(I_color, I_color_stretch_contrast);
    t = vpTime::measureTimeMs() - t;
    std::cout << "Time to do color contrast stretching: " << t << " ms" << std::endl;

    // Save stretchContrast
    filename = vpIoTools::createFilePath(opath, "Klimt_stretch_contrast.ppm");
    vpImageIo::write(I_color_stretch_contrast, filename);

    // Stretch Contrast HSV
    vpImage<vpRGBa> I_color_stretch_contrast_HSV;
    t = vpTime::measureTimeMs();
    vp::stretchContrastHSV(I_color, I_color_stretch_contrast_HSV);
    t = vpTime::measureTimeMs() - t;
    std::cout << "Time to do color HSV contrast stretching: " << t << " ms" << std::endl;

    // Save stretchContrastHSV
    filename = vpIoTools::createFilePath(opath, "Klimt_stretch_contrast_HSV.ppm");
    vpImageIo::write(I_color_stretch_contrast_HSV, filename);

    // Unsharp Mask
    vpImage<vpRGBa> I_color_unsharp_mask;
    t = vpTime::measureTimeMs();
    vp::unsharpMask(I_color, I_color_unsharp_mask);
    t = vpTime::measureTimeMs() - t;
    std::cout << "Time to do color unsharp mask: " << t << " ms" << std::endl;

    // Save unsharpMask
    filename = vpIoTools::createFilePath(opath, "Klimt_unsharp_mask.ppm");
    vpImageIo::write(I_color_unsharp_mask, filename);

    // CLAHE
    vpImage<vpRGBa> I_color_clahe;
    t = vpTime::measureTimeMs();
    vp::clahe(I_color, I_color_clahe, 50);
    t = vpTime::measureTimeMs() - t;
    std::cout << "Time to do color CLAHE: " << t << " ms" << std::endl;

    // Save CLAHE
    filename = vpIoTools::createFilePath(opath, "Klimt_CLAHE.ppm");
    vpImageIo::write(I_color_clahe, filename);

    //
    // Test grayscale function using image0000.pgm
    //

    // Read image0000.pgm
    filename = vpIoTools::createFilePath(ipath, "mbt/cube/image0000.pgm");
    vpImage<unsigned char> Iinput, I;
    std::cout << "\nRead image: " << filename << std::endl;
    vpImageIo::read(Iinput, filename);
    Iinput.halfSizeImage(I);
    std::cout << "Image: " << I.getWidth() << "x" << I.getHeight() << std::endl;

    // Adjust
    vpImage<unsigned char> I_adjust;
    beta = -20.0;
    t = vpTime::measureTimeMs();
    vp::adjust(I, I_adjust, alpha, beta);
    t = vpTime::measureTimeMs() - t;
    std::cout << "Time to do grayscale adjust: " << t << " ms" << std::endl;

    // Save adjust
    filename = vpIoTools::createFilePath(opath, "image0000_adjust.pgm");
    vpImageIo::write(I_adjust, filename);

    // Equalize Histogram
    vpImage<unsigned char> I_equalize_histogram;
    t = vpTime::measureTimeMs();
    vp::equalizeHistogram(I, I_equalize_histogram);
    t = vpTime::measureTimeMs() - t;
    std::cout << "Time to do grayscale histogram equalization: " << t << " ms" << std::endl;

    // Save equalizeHistogram
    filename = vpIoTools::createFilePath(opath, "image0000_equalize_histogram.pgm");
    vpImageIo::write(I_equalize_histogram, filename);

    // Gamma correction
    vpImage<unsigned char> I_gamma_correction;
    gamma = 1.8;
    t = vpTime::measureTimeMs();
    vp::gammaCorrection(I, I_gamma_correction, gamma);
    t = vpTime::measureTimeMs() - t;
    std::cout << "Time to do grayscale gamma correction: " << t << " ms" << std::endl;

    // Save gammaCorrection
    filename = vpIoTools::createFilePath(opath, "image0000_gamma_correction.pgm");
    vpImageIo::write(I_gamma_correction, filename);

    // Stretch contrast
    vpImage<unsigned char> I_stretch_contrast;
    t = vpTime::measureTimeMs();
    vp::stretchContrast(I, I_stretch_contrast);
    t = vpTime::measureTimeMs() - t;
    std::cout << "Time to do grayscale contrast stretching: " << t << " ms" << std::endl;

    // Save stretchContrast
    filename = vpIoTools::createFilePath(opath, "image0000_stretch_contrast.pgm");
    vpImageIo::write(I_stretch_contrast, filename);

    // Unsharp Mask
    vpImage<unsigned char> I_unsharp_mask;
    t = vpTime::measureTimeMs();
    vp::unsharpMask(I, I_unsharp_mask);
    t = vpTime::measureTimeMs() - t;
    std::cout << "Time to do grayscale unsharp mask: " << t << " ms" << std::endl;

    // Save unsharpMask
    filename = vpIoTools::createFilePath(opath, "image0000_unsharp_mask.pgm");
    vpImageIo::write(I_unsharp_mask, filename);

    // CLAHE
    vpImage<unsigned char> I_clahe;
    t = vpTime::measureTimeMs();
    vp::clahe(I, I_clahe, 50);
    t = vpTime::measureTimeMs() - t;
    std::cout << "Time to do grayscale CLAHE: " << t << " ms" << std::endl;

    // Save CLAHE
    filename = vpIoTools::createFilePath(opath, "image0000_CLAHE.pgm");
    vpImageIo::write(I_clahe, filename);

    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
}
